use itertools::Itertools;
use parry3d::{
    math::{Isometry, Point, Vector},
    partitioning::Qbvh,
    query::{
        PointProjection, PointQuery, PointQueryWithLocation, Ray, RayCast, RayIntersection,
        details::{
            NormalConstraints, RayCompositeShapeToiAndNormalBestFirstVisitor,
            RayCompositeShapeToiBestFirstVisitor,
        },
        point::PointCompositeShapeProjWithLocationBestFirstVisitor,
    },
    shape::{FeatureId, Shape, Triangle, TypedSimdCompositeShape},
};

use crate::{Face, MeshGraph};

impl PointQuery for MeshGraph {
    #[inline]
    fn project_local_point(&self, point: &Point<f32>, solid: bool) -> PointProjection {
        self.project_local_point_and_get_location(point, solid).0
    }

    fn project_local_point_and_get_feature(
        &self,
        _point: &Point<f32>,
    ) -> (PointProjection, FeatureId) {
        unimplemented!("Not available")
    }
}

impl PointQueryWithLocation for MeshGraph {
    type Location = (Face, ());

    #[inline]
    fn project_local_point_and_get_location(
        &self,
        point: &Point<f32>,
        solid: bool,
    ) -> (PointProjection, Self::Location) {
        self.project_local_point_and_get_location_with_max_dist(point, solid, f32::MAX)
            .unwrap()
    }

    /// Projects a point on `self`, with a maximum projection distance.
    fn project_local_point_and_get_location_with_max_dist(
        &self,
        point: &Point<f32>,
        solid: bool,
        max_dist: f32,
    ) -> Option<(PointProjection, Self::Location)> {
        let mut visitor =
            PointCompositeShapeProjWithLocationBestFirstVisitor::new(self, point, solid);

        if let Some((_, (mut proj, (face, _)))) =
            self.qbvh
                .traverse_best_first_node(&mut visitor, 0, max_dist)
        {
            if let Some(vertex_normals) = &self.vertex_normals {
                let pseudo_normal = vertex_normals[self.halfedges[face.halfedge].end_vertex];

                let dpt = point - proj.point;
                proj.is_inside = dpt.dot(&Vector::new(
                    pseudo_normal.x,
                    pseudo_normal.y,
                    pseudo_normal.z,
                )) <= 0.0;
            }

            Some((proj, (face, ())))
        } else {
            None
        }
    }
}

impl RayCast for MeshGraph {
    #[inline]
    fn cast_local_ray(&self, ray: &Ray, max_time_of_impact: f32, solid: bool) -> Option<f32> {
        let mut visitor =
            RayCompositeShapeToiBestFirstVisitor::new(self, ray, max_time_of_impact, solid);

        self.qbvh
            .traverse_best_first(&mut visitor)
            .map(|(_, (_, toi))| toi)
    }

    #[inline]
    fn cast_local_ray_and_get_normal(
        &self,
        ray: &Ray,
        max_time_of_impact: f32,
        solid: bool,
    ) -> Option<RayIntersection> {
        let mut visitor = RayCompositeShapeToiAndNormalBestFirstVisitor::new(
            self,
            ray,
            max_time_of_impact,
            solid,
        );

        self.qbvh
            .traverse_best_first(&mut visitor)
            .map(|(_, (_, res))| res)
    }
}

impl TypedSimdCompositeShape for MeshGraph {
    type PartShape = Triangle;
    type PartNormalConstraints = (); // TODO : do we need TrianglePseudoNormals?
    type PartId = Face;

    #[inline(always)]
    fn map_typed_part_at(
        &self,
        face: Face,
        mut f: impl FnMut(
            Option<&Isometry<f32>>,
            &Self::PartShape,
            Option<&Self::PartNormalConstraints>,
        ),
    ) {
        let tri = self.triangle(face);
        let pseudo_normals = None; // self.triangle_normal_constraints(face_id);
        f(None, &tri, pseudo_normals.as_ref())
    }

    #[inline(always)]
    fn map_untyped_part_at(
        &self,
        face: Face,
        mut f: impl FnMut(Option<&Isometry<f32>>, &dyn Shape, Option<&dyn NormalConstraints>),
    ) {
        let tri = self.triangle(face);
        let pseudo_normals = Some(()); // self.triangle_normal_constraints(face_id);
        f(
            None,
            &tri,
            pseudo_normals.as_ref().map(|n| n as &dyn NormalConstraints),
        )
    }

    fn typed_qbvh(&self) -> &Qbvh<Face> {
        &self.qbvh
    }
}

impl MeshGraph {
    pub fn triangle(&self, face: Face) -> Triangle {
        let pos = face
            .vertices(self)
            .into_iter()
            .map(|v_id| self.positions[v_id])
            .collect_vec();

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
