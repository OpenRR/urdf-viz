use crate::errors::{Error, Result};
use collada::document::ColladaDocument;
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
    use_texture: bool,
) -> Result<SceneNode> {
    use std::{
        ffi::OsStr,
        fs::{self, File},
        path::Path,
    };

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
        Some("dae" | "DAE") => {
            debug!("load dae: path = {}", filename.display());
            let mut base = group.add_group();
            let meshes = read_dae(&fs::read_to_string(filename)?, use_texture)?;
            info!("num mesh = {}", meshes.len());
            for mesh in meshes {
                let mut scene = base.add_mesh(mesh, scale);
                if let Some(color) = *opt_color {
                    scene.set_color(color[0], color[1], color[2]);
                }
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
    use_texture: bool,
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
        MeshKind::Dae => {
            debug!("load dae: path = {}", data.path);
            let mut base = group.add_group();
            let meshes = read_dae(data.string().unwrap(), use_texture)?;
            info!("num mesh = {}", meshes.len());
            for mesh in meshes {
                let mut scene = base.add_mesh(mesh, scale);
                if let Some(color) = *opt_color {
                    scene.set_color(color[0], color[1], color[2]);
                }
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

/*
#[allow(clippy::type_complexity)]
pub fn read_dae(string: &str) -> Result<(Vec<RefCellMesh>, HashMap<String, na::Vector3<f32>>)> {
    let doc = ColladaDocument::from_str(string)?;

    // let ns = doc.root_element.ns.as_deref();
    // let material_to_effect = doc
    //     .root_element
    //     .get_child("library_materials", ns)
    //     .map(|_| doc.get_material_to_effect())
    //     .unwrap_or_default();
    // let effects = get_effect_library(&doc);

    // todo: remove unwrap
    let objects = doc.get_obj_set().unwrap();

    let meshes = objects
        .objects
        .iter()
        .map(|object| {
            let vertices = object
                .vertices
                .iter()
                .map(|v| na::Point3::new(v.x as f32, v.y as f32, v.z as f32))
                .collect();

            let mut indices = vec![];
            let mut materials = vec![];
            for geometry in &object.geometry {
                for mesh in &geometry.mesh {
                    match mesh {
                        collada::PrimitiveElement::Triangles(triangles) => {
                            indices.extend(
                                triangles.vertices.iter().map(|&(x, y, z)| {
                                    na::Point3::new(x as u16, y as u16, z as u16)
                                }),
                            );
                            materials.extend(triangles.material.as_ref());
                        }
                        collada::PrimitiveElement::Polylist(polylist) => {
                            // TODO
                            for &shape in &polylist.shapes {
                                if let collada::Shape::Triangle((x, ..), (y, ..), (z, ..)) = shape {
                                    indices.push(na::Point3::new(x as u16, y as u16, z as u16));
                                } else {
                                    debug!("{:?}", shape);
                                }
                            }
                            materials.extend(polylist.material.as_ref());
                        }
                    }
                }
            }

            Rc::new(RefCell::new(kiss3d::resource::Mesh::new(
                vertices, indices, None, None, false,
            )))
        })
        .collect();

    let colors = get_effect_library(&doc)
        .into_iter()
        .map(|(name, material)| {
            (
                name,
                na::Vector3::<f32>::new(
                    material.diffuse[0],
                    material.diffuse[1],
                    material.diffuse[2],
                ),
            )
        })
        .collect();

    Ok((meshes, colors))
}

// Based on ColladaDocument::get_effect_library.
// ColladaDocument::get_effect_library panics if elements does not exist.
fn get_effect_library(doc: &ColladaDocument) -> HashMap<String, collada::document::PhongEffect> {
    macro_rules! tri {
        ($expr:expr) => {
            match $expr {
                Some(v) => v,
                None => return Default::default(),
            }
        };
    }

    fn parse_string_to_vector<T: std::str::FromStr>(string: &str) -> Vec<T> {
        string
            .trim()
            .split(&[' ', '\n'][..])
            .map(|s| s.parse().ok().expect("Error parsing array in COLLADA file"))
            .collect()
    }

    fn get_color(el: &str) -> Option<[f32; 4]> {
        let v: Vec<f32> = parse_string_to_vector(el);
        if v.len() == 4 {
            Some([v[0], v[1], v[2], v[3]])
        } else {
            None
        }
    }

    let ns = doc.root_element.ns.as_deref();
    let lib_effs = tri!(doc.root_element.get_child("library_effects", ns));
    lib_effs
        .get_children("effect", ns)
        .flat_map(|el| -> Option<(String, collada::document::PhongEffect)> {
            let id = el.get_attribute("id", None)?;
            let prof = el.get_child("profile_COMMON", ns)?;
            let tech = prof.get_child("technique", ns)?;
            let phong = tech.get_child("phong", ns)?;
            let emission_color = phong.get_child("emission", ns)?.get_child("color", ns)?;
            let emission = get_color(&emission_color.content_str())?;
            let ambient_color = phong.get_child("ambient", ns)?.get_child("color", ns)?;
            let ambient = get_color(&ambient_color.content_str())?;
            let diffuse_color = phong.get_child("diffuse", ns)?.get_child("color", ns)?;
            let diffuse = get_color(&diffuse_color.content_str())?;
            let specular_color = phong.get_child("specular", ns)?.get_child("color", ns)?;
            let specular = get_color(&specular_color.content_str())?;
            let shininess: f32 = phong
                .get_child("shininess", ns)?
                .get_child("float", ns)?
                .content_str()
                .parse()
                .ok()?;
            let index_of_refraction: f32 = phong
                .get_child("index_of_refraction", ns)?
                .get_child("float", ns)?
                .content_str()
                .parse()
                .ok()?;
            Some((
                id.to_string(),
                collada::document::PhongEffect {
                    emission,
                    ambient,
                    diffuse,
                    specular,
                    shininess,
                    index_of_refraction,
                },
            ))
        })
        .collect()
}
*/

pub fn read_dae(string: &str, use_texture: bool) -> Result<Vec<RefCellMesh>> {
    let doc = ColladaDocument::from_str(string)?;
    let obj_set = doc.get_obj_set().ok_or("NoObjSet")?;
    if obj_set.objects.is_empty() {
        return Err("EmptyFile".into());
    }

    let mut meshes = vec![];
    for object in obj_set.objects {
        let p = &object.vertices;
        let n = &object.normals;
        let t = &object.tex_vertices;

        let mut positions = vec![];
        let mut normals = vec![];
        let mut texcoords = vec![];
        let mut indices = vec![];

        for geometry in &object.geometry {
            for primitive in &geometry.mesh {
                match primitive {
                    collada::PrimitiveElement::Triangles(triangles) => {
                        let normals_idx = triangles.normals.as_ref().unwrap();
                        let texcoords_idx = triangles.tex_vertices.as_ref().unwrap();
                        let positions_idx = &triangles.vertices;
                        assert_eq!(positions_idx.len(), normals_idx.len());
                        assert_eq!(positions_idx.len(), texcoords_idx.len());

                        let mut idx = 0u16;
                        for (&vertex_idx, (&normal_idx, &tx_idx)) in positions_idx
                            .iter()
                            .zip(normals_idx.iter().zip(texcoords_idx.iter()))
                        {
                            positions.push(na::Point3::new(
                                p[vertex_idx.0].x as f32,
                                p[vertex_idx.0].y as f32,
                                p[vertex_idx.0].z as f32,
                            ));
                            positions.push(na::Point3::new(
                                p[vertex_idx.1].x as f32,
                                p[vertex_idx.1].y as f32,
                                p[vertex_idx.1].z as f32,
                            ));
                            positions.push(na::Point3::new(
                                p[vertex_idx.2].x as f32,
                                p[vertex_idx.2].y as f32,
                                p[vertex_idx.2].z as f32,
                            ));
                            normals.push(na::Vector3::new(
                                n[normal_idx.0].x as f32,
                                n[normal_idx.0].y as f32,
                                n[normal_idx.0].z as f32,
                            ));
                            normals.push(na::Vector3::new(
                                n[normal_idx.1].x as f32,
                                n[normal_idx.1].y as f32,
                                n[normal_idx.1].z as f32,
                            ));
                            normals.push(na::Vector3::new(
                                n[normal_idx.2].x as f32,
                                n[normal_idx.2].y as f32,
                                n[normal_idx.2].z as f32,
                            ));
                            texcoords
                                .push(na::Point2::new(t[tx_idx.0].x as f32, t[tx_idx.0].y as f32));
                            texcoords
                                .push(na::Point2::new(t[tx_idx.1].x as f32, t[tx_idx.1].y as f32));
                            texcoords
                                .push(na::Point2::new(t[tx_idx.2].x as f32, t[tx_idx.2].y as f32));

                            indices.push(na::Point3::new(idx, idx + 1, idx + 2));
                            idx += 3;
                        }
                    }
                    _ => {
                        debug!("PrimitiveNotTriangles");
                    } // _ => return Err("PrimitiveNotTriangles".into()),
                }
            }
        }

        meshes.push(Rc::new(RefCell::new(kiss3d::resource::Mesh::new(
            positions,
            indices,
            None,
            if use_texture { Some(texcoords) } else { None },
            false,
        ))));
    }

    Ok(meshes)
}
