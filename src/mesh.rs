use crate::errors::{Error, Result};
use k::nalgebra as na;
use kiss3d::{resource::Mesh, scene::SceneNode};
use std::{cell::RefCell, io, rc::Rc};
use tracing::*;

type RefCellMesh = Rc<RefCell<Mesh>>;

#[cfg(feature = "assimp")]
pub fn load_mesh(
    filename: impl AsRef<str>,
    scale: na::Vector3<f32>,
    opt_urdf_color: &Option<na::Point3<f32>>,
    group: &mut SceneNode,
    use_texture: bool,
) -> Result<SceneNode> {
    use crate::assimp_utils::*;
    use std::{ffi::OsStr, path::Path};

    let file_string = filename.as_ref();
    let filename = Path::new(file_string);

    let mut base = group.add_group();
    let mut importer = assimp::Importer::new();
    importer.pre_transform_vertices(|x| x.enable = true);
    importer.collada_ignore_up_direction(true);
    let (meshes, textures, colors) =
        convert_assimp_scene_to_kiss3d_mesh(&importer.read_file(file_string)?);
    info!(
        "num mesh, texture, colors = {} {} {}",
        meshes.len(),
        textures.len(),
        colors.len()
    );
    let mesh_scenes = meshes
        .into_iter()
        .map(|mesh| {
            let mut scene = base.add_mesh(mesh, scale);
            // use urdf color as default
            if let Some(urdf_color) = *opt_urdf_color {
                scene.set_color(urdf_color[0], urdf_color[1], urdf_color[2]);
            }
            scene
        })
        .collect::<Vec<_>>();
    // use texture only for dae (collada)
    let is_collada = matches!(
        filename.extension().and_then(OsStr::to_str),
        Some("dae" | "DAE")
    );
    // do not use texture, use only color in urdf file.
    if !use_texture || !is_collada {
        return Ok(base);
    }

    // Size of color and mesh are same, use each color for mesh
    if mesh_scenes.len() == colors.len() {
        for (count, (mut mesh_scene, color)) in
            mesh_scenes.into_iter().zip(colors.into_iter()).enumerate()
        {
            mesh_scene.set_color(color[0], color[1], color[2]);
            // Is this OK?
            if count < textures.len() {
                let mut texture_path = filename.to_path_buf();
                texture_path.set_file_name(textures[count].clone());
                debug!("using texture={}", texture_path.display());
                if texture_path.exists() {
                    mesh_scene.set_texture_from_file(&texture_path, texture_path.to_str().unwrap());
                }
            }
        }
    } else {
        // When size of mesh and color mismatch, use only first color/texture for all meshes.
        // If no color found, use urdf color instead.
        for mut mesh_scene in mesh_scenes {
            if !textures.is_empty() {
                let mut texture_path = filename.to_path_buf();
                texture_path.set_file_name(textures[0].clone());
                debug!("texture={}", texture_path.display());
                if texture_path.exists() {
                    mesh_scene.set_texture_from_file(&texture_path, texture_path.to_str().unwrap());
                }
            }
            if !colors.is_empty() {
                let color = colors[0];
                mesh_scene.set_color(color[0], color[1], color[2]);
            }
        }
    }
    Ok(base)
}

#[cfg(not(feature = "assimp"))]
#[cfg(not(target_arch = "wasm32"))]
pub fn load_mesh(
    filename: impl AsRef<str>,
    scale: na::Vector3<f32>,
    opt_color: &Option<na::Point3<f32>>,
    group: &mut SceneNode,
    _use_texture: bool,
) -> Result<SceneNode> {
    use std::{ffi::OsStr, fs::File, path::Path};

    let filename = Path::new(filename.as_ref());

    match filename.extension().and_then(OsStr::to_str) {
        Some("obj" | "OBJ") => {
            let mtl_path = filename.parent().unwrap_or_else(|| Path::new("."));
            debug!(
                "load obj: path = {}, mtl_path = {}",
                filename.display(),
                mtl_path.display()
            );
            let mut base = group.add_obj(filename, mtl_path, scale);
            if let Some(color) = *opt_color {
                base.set_color(color[0], color[1], color[2]);
            }
            Ok(base)
        }
        Some("stl" | "STL") => {
            debug!("load stl: path = {}", filename.display());
            let mesh = read_stl(File::open(filename)?)?;
            let mut base = group.add_mesh(mesh, scale);
            if let Some(color) = *opt_color {
                base.set_color(color[0], color[1], color[2]);
            }
            Ok(base)
        }
        _ => Err(crate::errors::Error::from(format!(
            "{} is not supported, because assimp feature is disabled",
            filename.display()
        ))),
    }
}

/// NOTE: Unlike other platforms, the first argument should be the data loaded
/// by [`utils::load_mesh`](crate::utils::load_mesh), not the path.
#[cfg(not(feature = "assimp"))]
#[cfg(target_arch = "wasm32")]
pub fn load_mesh(
    data: impl AsRef<str>,
    scale: na::Vector3<f32>,
    opt_color: &Option<na::Point3<f32>>,
    group: &mut SceneNode,
    _use_texture: bool,
) -> Result<SceneNode> {
    use crate::utils::MeshKind;

    let data = crate::utils::Mesh::decode(data.as_ref())?;

    match data.kind {
        MeshKind::Obj => {
            debug!("load obj: path = {}", data.path);
            let mut base = add_obj(group, &data, scale);
            if let Some(color) = *opt_color {
                base.set_color(color[0], color[1], color[2]);
            }
            Ok(base)
        }
        MeshKind::Stl => {
            debug!("load stl: path = {}", data.path);
            let mesh = read_stl(data.reader().unwrap())?;
            let mut base = group.add_mesh(mesh, scale);
            if let Some(color) = *opt_color {
                base.set_color(color[0], color[1], color[2]);
            }
            Ok(base)
        }
        MeshKind::Other => Err(crate::errors::Error::from(format!(
            "{} is not supported, because assimp feature is disabled",
            data.path
        ))),
    }
}

// Refs: https://github.com/sebcrozet/kiss3d/blob/73ff15dc40aaf994f3e8e240c23bb660be71a6cd/src/scene/scene_node.rs#L807-L866
#[cfg(not(feature = "assimp"))]
#[cfg(target_arch = "wasm32")]
fn add_obj(group: &mut SceneNode, data: &crate::utils::Mesh, scale: na::Vector3<f32>) -> SceneNode {
    let tex = kiss3d::resource::TextureManager::get_global_manager(|tm| tm.get_default());
    let mat = kiss3d::resource::MaterialManager::get_global_manager(|mm| mm.get_default());

    // TODO: mtl
    let objs = kiss3d::loader::obj::parse(data.string().unwrap(), ".".as_ref(), &data.path);
    let mut root;

    let self_root = objs.len() == 1;
    let child_scale;

    if self_root {
        root = group.clone();
        child_scale = scale;
    } else {
        root = SceneNode::new(scale, na::one(), None);
        group.add_child(root.clone());
        child_scale = na::Vector3::from_element(1.0);
    }

    let mut last = None;
    for (_, mesh, _mtl) in objs {
        let mesh = Rc::new(RefCell::new(mesh));
        let object = kiss3d::scene::Object::new(mesh, 1.0, 1.0, 1.0, tex.clone(), mat.clone());

        // TODO: mtl

        last = Some(root.add_object(child_scale, na::one(), object));
    }

    if self_root {
        last.expect("there was nothing on this obj file")
    } else {
        root
    }
}

pub fn read_stl(mut reader: impl io::Read + io::Seek) -> Result<RefCellMesh> {
    // TODO: Once https://github.com/hmeyer/stl_io/pull/14 is merged and released, compare with stl_io.
    let mesh: nom_stl::IndexMesh = nom_stl::parse_stl(&mut reader)
        .map_err(|e| match e {
            nom_stl::Error::IOError(e) => Error::IoError(e),
            nom_stl::Error::ParseError(e) => Error::Other(e),
        })?
        .into();

    let vertices = mesh
        .vertices()
        .iter()
        .map(|v| na::Point3::new(v[0], v[1], v[2]))
        .collect();

    let indices = mesh
        .triangles()
        .iter()
        .map(|face| {
            na::Point3::new(
                face.vertices_indices()[0] as u16,
                face.vertices_indices()[1] as u16,
                face.vertices_indices()[2] as u16,
            )
        })
        .collect();

    Ok(Rc::new(RefCell::new(kiss3d::resource::Mesh::new(
        vertices, indices, None, None, false,
    ))))
}
