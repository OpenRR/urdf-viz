#[cfg(not(target_arch = "wasm32"))]
pub(crate) use native::*;
#[cfg(target_arch = "wasm32")]
pub use wasm::*;

#[cfg(not(target_arch = "wasm32"))]
mod native {
    use std::sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    };

    use tracing::error;

    #[derive(Debug)]
    pub(crate) struct RobotModel {
        robot: urdf_rs::Robot,
        needs_reload: Arc<AtomicBool>,
    }

    impl RobotModel {
        pub(crate) fn new(robot: urdf_rs::Robot) -> (Self, Arc<AtomicBool>) {
            let needs_reload = Arc::new(AtomicBool::new(false));
            (
                Self {
                    robot,
                    needs_reload: needs_reload.clone(),
                },
                needs_reload,
            )
        }

        pub(crate) fn get(&mut self) -> &urdf_rs::Robot {
            &self.robot
        }

        pub(crate) fn request_reload(&mut self, input_file: impl AsRef<str>) {
            match urdf_rs::utils::read_urdf_or_xacro(input_file.as_ref()) {
                Ok(robot) => {
                    self.robot = robot;
                    self.needs_reload.store(true, Ordering::Relaxed);
                }
                Err(e) => {
                    error!("{}", e);
                }
            }
        }
    }
}

#[cfg(target_arch = "wasm32")]
mod wasm {
    use std::{
        io::Cursor,
        path::Path,
        str,
        sync::{
            atomic::{AtomicBool, Ordering},
            Arc,
        },
    };

    use futures::future::FutureExt;
    use js_sys::Uint8Array;
    use serde::{Deserialize, Serialize};
    use tokio::sync::{mpsc, watch};
    use tracing::{debug, error};
    use wasm_bindgen::JsCast;
    use wasm_bindgen_futures::JsFuture;
    use web_sys::Response;

    use crate::{Error, Result};

    #[derive(Serialize, Deserialize)]
    pub(crate) struct Mesh {
        pub(crate) kind: MeshKind,
        pub(crate) path: String,
        data: MeshData,
    }

    impl Mesh {
        pub(crate) fn decode(data: &str) -> Result<Self> {
            let mut mesh: Self = serde_json::from_str(data).map_err(|e| e.to_string())?;
            match &mesh.data {
                MeshData::None => {}
                MeshData::Base64(s) => {
                    mesh.data = MeshData::Bytes(base64::decode(s).map_err(|e| e.to_string())?);
                }
                MeshData::Bytes(_) => unreachable!(),
            }
            Ok(mesh)
        }

        pub(crate) fn reader(&self) -> Option<Cursor<&[u8]>> {
            match &self.data {
                MeshData::None => None,
                MeshData::Bytes(bytes) => Some(Cursor::new(bytes)),
                MeshData::Base64(_) => unreachable!(),
            }
        }

        pub(crate) fn string(&self) -> Option<&str> {
            match &self.data {
                MeshData::None => None,
                MeshData::Bytes(s) => str::from_utf8(s).ok(),
                MeshData::Base64(..) => unreachable!(),
            }
        }
    }

    #[derive(Serialize, Deserialize)]
    enum MeshData {
        Base64(String),
        Bytes(Vec<u8>),
        None,
    }

    #[derive(Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    pub(crate) enum MeshKind {
        Obj,
        Stl,
        Dae,
        Other,
    }

    pub fn window() -> Result<web_sys::Window> {
        Ok(web_sys::window().ok_or("failed to get window")?)
    }

    async fn fetch(input_file: &str) -> Result<Response> {
        let promise = window()?.fetch_with_str(input_file);

        let response = JsFuture::from(promise)
            .await?
            .dyn_into::<Response>()
            .unwrap();

        Ok(response)
    }

    pub async fn read_to_string(input_file: impl AsRef<str>) -> Result<String> {
        let promise = fetch(input_file.as_ref()).await?.text()?;

        let s = JsFuture::from(promise).await?;

        Ok(s.as_string()
            .ok_or_else(|| format!("{} is not string", input_file.as_ref()))?)
    }

    pub async fn read(input_file: impl AsRef<str>) -> Result<Vec<u8>> {
        let promise = fetch(input_file.as_ref()).await?.array_buffer()?;

        let bytes = JsFuture::from(promise).await?;

        Ok(Uint8Array::new(&bytes).to_vec())
    }

    pub async fn read_urdf(input_file: impl AsRef<str>) -> Result<urdf_rs::Robot> {
        let s = read_to_string(input_file).await?;
        Ok(urdf_rs::read_from_string(&s)?)
    }

    pub async fn load_mesh(robot: &mut urdf_rs::Robot, urdf_path: impl AsRef<Path>) -> Result<()> {
        let urdf_path = urdf_path.as_ref();
        for geometry in robot.links.iter_mut().flat_map(|link| {
            link.visual
                .iter_mut()
                .map(|v| &mut v.geometry)
                .chain(link.collision.iter_mut().map(|c| &mut c.geometry))
        }) {
            if let urdf_rs::Geometry::Mesh { filename, .. } = geometry {
                let input_file =
                    if filename.starts_with("https://") || filename.starts_with("http://") {
                        filename.clone()
                    } else if filename.starts_with("package://") {
                        return Err(Error::from(format!(
                            "ros package ({}) is not supported in wasm",
                            filename
                        )));
                    } else {
                        // We don't use url::Url::path/set_path here, because
                        // urdf_path may be a relative path to a file bundled
                        // with the server. Path::with_file_name works for wasm
                        // where the separator is /, so we use it.
                        urdf_path
                            .with_file_name(&filename)
                            .to_str()
                            .unwrap()
                            .to_string()
                    };

                let kind = if input_file.ends_with(".obj") || input_file.ends_with(".OBJ") {
                    MeshKind::Obj
                } else if input_file.ends_with(".stl") || input_file.ends_with(".STL") {
                    MeshKind::Stl
                } else if input_file.ends_with(".dae") || input_file.ends_with(".DAE") {
                    MeshKind::Dae
                } else {
                    MeshKind::Other
                };

                let data = if kind != MeshKind::Other {
                    debug!("loading {}", input_file);
                    MeshData::Base64(base64::encode(read(&input_file).await?))
                } else {
                    MeshData::None
                };

                let new = serde_json::to_string(&Mesh {
                    kind,
                    path: filename.clone(),
                    data,
                })
                .unwrap();
                *filename = new;
            }
        }
        Ok(())
    }

    #[derive(Debug)]
    pub(crate) struct RobotModel {
        request_sender: mpsc::UnboundedSender<String>,
        response_receiver: watch::Receiver<Arc<urdf_rs::Robot>>,
        cache: Arc<urdf_rs::Robot>,
    }

    impl RobotModel {
        pub(crate) fn new(robot: urdf_rs::Robot) -> (Self, Arc<AtomicBool>) {
            let robot = Arc::new(robot);
            let (request_sender, mut request_receiver) = mpsc::unbounded_channel();
            let (response_sender, response_receiver) = watch::channel(robot.clone());
            let needs_reload = Arc::new(AtomicBool::new(false));
            let needs_reload_clone = needs_reload.clone();
            wasm_bindgen_futures::spawn_local(async move {
                while let Some(input_file) = request_receiver.recv().await {
                    match read_urdf(&input_file).await {
                        Ok(mut robot) => {
                            if let Err(e) = load_mesh(&mut robot, input_file).await {
                                error!("{}", e);
                            } else if response_sender.send(Arc::new(robot)).is_ok() {
                                needs_reload_clone.store(true, Ordering::Relaxed);
                            }
                        }
                        Err(e) => {
                            error!("{}", e);
                        }
                    }
                }
            });
            (
                Self {
                    request_sender,
                    response_receiver,
                    cache: robot,
                },
                needs_reload,
            )
        }

        pub(crate) fn get(&mut self) -> &urdf_rs::Robot {
            // Update cache
            // https://github.com/tokio-rs/tokio/issues/3666
            if let Some(Ok(())) = self.response_receiver.changed().now_or_never() {
                self.cache = self.response_receiver.borrow().clone();
            }
            &self.cache
        }

        pub(crate) fn request_reload(&self, input_file: impl AsRef<str>) {
            let _ = self.request_sender.send(input_file.as_ref().to_owned());
        }
    }
}
