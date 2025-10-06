use itertools::Itertools;
use parry3d::{
    math::{Isometry, Point},
    partitioning::Bvh,
    query::details::NormalConstraints,
    shape::{CompositeShape, Shape, Triangle, TypedCompositeShape},
};
use tracing::instrument;

use crate::{MeshGraph, error_none, utils::unwrap_or_return};

impl CompositeShape for MeshGraph {
    fn map_part_at(
        &self,
        shape_id: u32,
        f: &mut dyn FnMut(Option<&Isometry<f32>>, &dyn Shape, Option<&dyn NormalConstraints>),
    ) {
        let tri = self.triangle(shape_id);
        let normal_constraints = Default::default(); // self.triangle_normal_constraints(face_id);
        f(None, &tri, normal_constraints)
    }

    fn bvh(&self) -> &Bvh {
        &self.bvh
    }
}

impl TypedCompositeShape for MeshGraph {
    type PartShape = Triangle;
    type PartNormalConstraints = ();

    fn map_typed_part_at<T>(
        &self,
        shape_id: u32,
        mut f: impl FnMut(
            Option<&Isometry<f32>>,
            &Self::PartShape,
            Option<&Self::PartNormalConstraints>,
        ) -> T,
    ) -> Option<T> {
        let tri = self.triangle(shape_id);
        let pseudo_normals = None; // self.triangle_normal_constraints(face_id);
        Some(f(None, &tri, pseudo_normals.as_ref()))
    }

    fn map_untyped_part_at<T>(
        &self,
        shape_id: u32,
        mut f: impl FnMut(Option<&Isometry<f32>>, &dyn Shape, Option<&dyn NormalConstraints>) -> T,
    ) -> Option<T> {
        let tri = self.triangle(shape_id);
        let pseudo_normals = Default::default(); // self.triangle_normal_constraints(face_id);
        Some(f(None, &tri, pseudo_normals))
    }
}

impl MeshGraph {
    #[instrument(skip(self))]
    pub fn triangle(&self, shape_id: u32) -> Triangle {
        let face_id = unwrap_or_return!(
            self.index_to_face_id.get(shape_id as usize),
            "Index not found",
            Triangle::default()
        );

        let face = unwrap_or_return!(
            self.faces.get(*face_id),
            "Face not found",
            Triangle::default()
        );

        let pos = face
            .vertices(self)
            .filter_map(|v_id| {
                self.positions
                    .get(v_id)
                    .or_else(error_none!("Position not found"))
            })
            .collect_vec();

        if pos.len() < 3 {
            return Triangle::default();
        }

        Triangle::new(
            Point::new(pos[0].x, pos[0].y, pos[0].z),
            Point::new(pos[1].x, pos[1].y, pos[1].z),
            Point::new(pos[2].x, pos[2].y, pos[2].z),
        )
    }

    // TODO : is this necessary?
    // pub fn triangle_normal_constraints(&self, face_id: FaceId) -> Option<TrianglePseudoNormals> {
    //     if let Some(vertex_normals) = &self.vertex_normals {
    //         let triangle = self.triangle(face_id);
    //         let pseudo_normals = self.pseudo_normals.as_ref()?;
    //         let edges_pseudo_normals = pseudo_normals.edges_pseudo_normal[i as usize];

    //         // TODO: could the pseudo-normal be pre-normalized instead of having to renormalize
    //         //       every time we need them?
    //         Some(TrianglePseudoNormals {
    //             face: triangle.normal()?,
    //             edges: [
    //                 Unit::try_new(edges_pseudo_normals[0], 1.0e-6)?,
    //                 Unit::try_new(edges_pseudo_normals[1], 1.0e-6)?,
    //                 Unit::try_new(edges_pseudo_normals[2], 1.0e-6)?,
    //             ],
    //         })
    //     } else {
    //         None
    //     }
    // }
}
