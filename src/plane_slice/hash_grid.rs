use glam::{IVec2, Vec2};
use hashbrown::{HashMap, HashSet};
use slotmap::{SecondaryMap, SlotMap, new_key_type};

use crate::plane_slice::{Polygon2, PolygonTerminal};
#[cfg(feature = "rerun")]
use crate::utils::vec2_array;

new_key_type! { pub struct PolygonId; }

#[derive(Default, Debug, Clone)]
pub struct HashGrid {
    map: HashMap<IVec2, Vec<(PolygonTerminal, PolygonId)>>,
    min_bounds: Vec2,
    cell_fac: Vec2,
    polygons: SlotMap<PolygonId, Polygon2>,
    polygon_cells: SecondaryMap<PolygonId, HashSet<IVec2>>,
}

impl HashGrid {
    pub fn new(min_bounds: Vec2, max_bounds: Vec2) -> Self {
        let width = (max_bounds.x - min_bounds.x).max(0.0001);
        let height = (max_bounds.y - min_bounds.y).max(0.0001);

        let grid_size_x = width.sqrt().ceil();
        let grid_size_y = height.sqrt().ceil();

        let cell_fac = Vec2::new(grid_size_x / width, grid_size_y / height);

        HashGrid {
            map: HashMap::new(),
            min_bounds,
            cell_fac,
            polygons: SlotMap::default(),
            polygon_cells: SecondaryMap::default(),
        }
    }

    pub fn try_take_connecting_polygon(
        &mut self,
        point: Vec2,
    ) -> Option<(PolygonTerminal, PolygonId)> {
        let key = self.vec2_to_grid(point);

        let mut result = None;

        self.map.entry(key).and_modify(|e| {
            let mut index = None;

            for (i, (terminal, polygon_id)) in e.iter().enumerate() {
                let polygon = &self.polygons[*polygon_id];
                let existing_point = polygon.terminal(*terminal);

                if existing_point.distance_squared(point) < 1e-6 {
                    index = Some(i);
                    break;
                }
            }

            if let Some(index) = index {
                let (terminal, polygon_id) = e.remove(index);
                result = Some((terminal, polygon_id));

                // check if the other end of the polygon is still in the cell
                if !e.iter().any(|(_, id)| *id == polygon_id) {
                    self.polygon_cells[polygon_id].retain(|k| *k != key);
                }
            }
        });

        result
    }

    pub fn remove_polygon_from_cell(&mut self, polygon_id: PolygonId, cell: IVec2) {
        self.map.entry(cell).and_modify(|e| {
            e.retain(|(_, id)| *id != polygon_id);
        });
    }

    pub fn insert_polygon_by_terminal(
        &mut self,
        terminal: PolygonTerminal,
        polygon_id: PolygonId,
        terminal_point: Vec2,
    ) {
        let key = self.vec2_to_grid(terminal_point);

        self.map
            .entry(key)
            .or_default()
            .push((terminal, polygon_id));

        self.polygon_cells
            .entry(polygon_id)
            .unwrap()
            .or_default()
            .insert(key);
    }

    pub fn insert_line(&mut self, point1: Vec2, point2: Vec2) {
        #[cfg(feature = "rerun")]
        {
            crate::RR
                .log("insert_line", &rerun::Clear::recursive())
                .unwrap();
            crate::RR
                .log(
                    "insert_line/line",
                    &rerun::LineStrips3D::new([[vec2_array(point1), vec2_array(point2)]]),
                )
                .unwrap();
        }

        if let Some((terminal1, polygon_id1)) = self.try_take_connecting_polygon(point1) {
            #[cfg(feature = "rerun")]
            self.polygons[polygon_id1].log_rerun("insert_line/existing_poly1");

            if let Some((terminal2, polygon_id2)) = self.try_take_connecting_polygon(point2) {
                #[cfg(feature = "rerun")]
                self.polygons[polygon_id2].log_rerun("insert_line/another_existing_poly2");

                self.merge_polygons(polygon_id1, terminal1, polygon_id2, terminal2);
            } else {
                self.extend_polygon(polygon_id1, terminal1, point2);
            }
        } else if let Some((terminal2, polygon_id2)) = self.try_take_connecting_polygon(point2) {
            #[cfg(feature = "rerun")]
            self.polygons[polygon_id2].log_rerun("insert_line/existing_poly2");

            self.extend_polygon(polygon_id2, terminal2, point1);
        } else {
            let new_polygon = Polygon2 {
                vertices: [point1, point2].into(),
            };
            let new_polygon_id = self.polygons.insert(new_polygon);

            self.insert_polygon_by_terminal(PolygonTerminal::Start, new_polygon_id, point1);
            self.insert_polygon_by_terminal(PolygonTerminal::End, new_polygon_id, point2);
        }
    }

    fn merge_polygons(
        &mut self,
        polygon_id1: PolygonId,
        terminal1: PolygonTerminal,
        polygon_id2: PolygonId,
        terminal2: PolygonTerminal,
    ) {
        if polygon_id1 == polygon_id2 {
            let polygon = &mut self.polygons[polygon_id1];
            polygon.close();

            return;
        }

        for cell in self.polygon_cells[polygon_id2].clone() {
            self.remove_polygon_from_cell(polygon_id2, cell);
        }
        self.polygon_cells.remove(polygon_id2);
        let polygon2 = self.polygons.remove(polygon_id2).unwrap();

        for cell in self.polygon_cells[polygon_id1].clone() {
            self.remove_polygon_from_cell(polygon_id1, cell);
        }
        let polygon1 = &mut self.polygons[polygon_id1];

        polygon1.merge_polygon(terminal1, polygon2, terminal2);

        let start_point = polygon1.terminal(PolygonTerminal::Start);
        let end_point = polygon1.terminal(PolygonTerminal::End);

        #[cfg(feature = "rerun")]
        {
            crate::RR
                .log("insert_line", &rerun::Clear::recursive())
                .unwrap();
            polygon1.log_rerun("insert_line/merged");
        }

        self.insert_polygon_by_terminal(PolygonTerminal::Start, polygon_id1, start_point);
        self.insert_polygon_by_terminal(PolygonTerminal::End, polygon_id1, end_point);
    }

    fn extend_polygon(
        &mut self,
        polygon_id: PolygonId,
        terminal: PolygonTerminal,
        new_point: Vec2,
    ) {
        let polygon = &mut self.polygons[polygon_id];
        polygon.extend_by_line(terminal, new_point);

        #[cfg(feature = "rerun")]
        {
            crate::RR
                .log("insert_line", &rerun::Clear::recursive())
                .unwrap();
            polygon.log_rerun("insert_line/extended");
        }

        self.insert_polygon_by_terminal(terminal, polygon_id, new_point);
    }

    #[inline]
    fn vec2_to_grid(&self, point: Vec2) -> IVec2 {
        ((point - self.min_bounds) * self.cell_fac)
            .floor()
            .as_ivec2()
    }

    pub fn into_polygons(self) -> impl Iterator<Item = Polygon2> {
        self.polygons.into_iter().map(|(_, polygon)| polygon)
    }
}
