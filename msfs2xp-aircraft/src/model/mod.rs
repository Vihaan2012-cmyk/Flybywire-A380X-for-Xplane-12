//! 3D models: Asobo glTF in, X-Plane OBJ8 out, with node animations.
//!
//! A copy of the converter's model code (`msfs2xp::model3d`), extended for
//! aircraft: node hierarchy, animation clips and skins are read, and objects
//! can be written with keyframed animation, manipulators and hidden parts.

pub mod anim;
pub mod glb;
pub mod obj8;

pub use anim::{Animator, ClipIndex};
pub use glb::{load_glb, Model};
pub use obj8::{simplify_mesh, simplify_mesh_sloppy, write_obj8_animated, write_obj8_animated_lods, AnimLodLevel, LodMesh, MeshAnim, ObjOptions, DECAL_ALPHA_CUTOFF};
