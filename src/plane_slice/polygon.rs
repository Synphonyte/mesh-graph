use std::collections::VecDeque;

use glam::{Mat4, Vec2, Vec3, Vec4Swizzles};
use itertools::Itertools;

#[derive(Debug, Clone, Copy)]
pub enum PolygonTerminal {
    Start,
    End,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Polygon2 {
    pub vertices: VecDeque<Vec2>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Polygon3 {
    pub vertices: VecDeque<Vec3>,
}

impl Polygon2 {
    pub fn terminal(&self, terminal: PolygonTerminal) -> Vec2 {
        match terminal {
            PolygonTerminal::Start => self.vertices[0],
            PolygonTerminal::End => self.vertices[self.vertices.len() - 1],
        }
    }

    pub fn extend_by_line(&mut self, terminal: PolygonTerminal, other_line_end: Vec2) {
        match terminal {
            PolygonTerminal::Start => self.vertices.push_front(other_line_end),
            PolygonTerminal::End => self.vertices.push_back(other_line_end),
        }
    }

    /// This assumes that the polygon to be merged doesn't contain any vertices that are already part of this polygon.
    pub fn merge_polygon(
        &mut self,
        my_terminal: PolygonTerminal,
        mut other: Polygon2,
        other_terminal: PolygonTerminal,
    ) {
        let mut vertices = VecDeque::new();

        match my_terminal {
            PolygonTerminal::Start => {
                match other_terminal {
                    PolygonTerminal::Start => vertices.extend(other.vertices.into_iter().rev()),
                    PolygonTerminal::End => vertices.append(&mut other.vertices),
                }

                vertices.append(&mut self.vertices);
            }
            PolygonTerminal::End => {
                vertices.append(&mut self.vertices);

                match other_terminal {
                    PolygonTerminal::Start => vertices.append(&mut other.vertices),
                    PolygonTerminal::End => vertices.extend(other.vertices.into_iter().rev()),
                }
            }
        }

        self.vertices = vertices;
    }

    pub fn is_closed(&self) -> bool {
        if let (Some(front), Some(back)) = (self.vertices.front(), self.vertices.back()) {
            front.distance_squared(*back) < 1e-6
        } else {
            false
        }
    }

    pub fn close(&mut self) {
        if !self.is_closed() {
            self.vertices.push_back(self.vertices[0]);
        }
    }

    pub fn length(&self) -> f32 {
        self.vertices
            .iter()
            .tuple_windows()
            .map(|(a, b)| a.distance(*b))
            .sum()
    }

    #[cfg(feature = "rerun")]
    pub fn log_rerun(&self, name: &str) {
        use crate::utils::vec2_array;

        crate::RR
            .log(
                name,
                &rerun::LineStrips3D::new([self.vertices.iter().copied().map(vec2_array)]),
            )
            .unwrap();
    }
}

impl Polygon3 {
    pub fn from_polygon2_with_transform(polygon: Polygon2, transform: Mat4) -> Self {
        let vertices = polygon
            .vertices
            .into_iter()
            .map(|v| (transform * v.extend(0.0).extend(1.0)).xyz())
            .collect();

        Self { vertices }
    }

    pub fn length(&self) -> f32 {
        self.vertices
            .iter()
            .tuple_windows()
            .map(|(a, b)| a.distance(*b))
            .sum()
    }

    #[cfg(feature = "rerun")]
    pub fn log_rerun<'a>(
        rr: &rerun::RecordingStream,
        name: &str,
        polygons: impl IntoIterator<Item = &'a Self>,
    ) {
        use crate::utils::vec3_array;

        rr.log(
            name,
            &rerun::LineStrips3D::new(
                polygons
                    .into_iter()
                    .map(|p| p.vertices.iter().copied().map(vec3_array)),
            ),
        )
        .unwrap();
    }
}
