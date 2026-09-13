use crate::{FaceId, HalfedgeId, MeshGraph, VertexId};

/// Records the elements created while recording is on, so selection-scoped
/// operations can learn exactly what they added.
///
/// Additions need a journal because "newness" is not observable after the fact:
/// a `SlotMap` key created during an operation is indistinguishable from one that
/// existed before it. Removals need no such thing — the keys are generational, so a
/// dead id is simply one `contains_key` rejects. See [`crate::Selection::retain_live`].
///
/// Recording is off by default and costs one predictable branch per element created.
#[derive(Clone, Default, Debug)]
pub(crate) struct CreationJournal {
    recording: bool,
    vertices: Vec<VertexId>,
    halfedges: Vec<HalfedgeId>,
    faces: Vec<FaceId>,
}

impl CreationJournal {
    /// Begins recording, discarding anything left from a previous run.
    fn start(&mut self) {
        debug_assert!(
            !self.recording,
            "CreationJournal::start while already recording - scoped ops do not nest"
        );
        self.vertices.clear();
        self.halfedges.clear();
        self.faces.clear();
        self.recording = true;
    }

    /// Stops recording and yields what was created.
    fn take(&mut self) -> (Vec<VertexId>, Vec<HalfedgeId>, Vec<FaceId>) {
        self.recording = false;
        (
            std::mem::take(&mut self.vertices),
            std::mem::take(&mut self.halfedges),
            std::mem::take(&mut self.faces),
        )
    }
}

impl MeshGraph {
    /// Begins recording created elements. Pair with [`Self::end_recording_creations`].
    pub(crate) fn start_recording_creations(&mut self) {
        self.creation_journal.start();
    }

    /// Stops recording and returns the elements created since
    /// [`Self::start_recording_creations`], in creation order.
    ///
    /// Elements that were created *and* then removed within the recorded window are
    /// still listed; callers that only want live ones filter by `contains_key`.
    pub(crate) fn end_recording_creations(
        &mut self,
    ) -> (Vec<VertexId>, Vec<HalfedgeId>, Vec<FaceId>) {
        self.creation_journal.take()
    }

    #[inline]
    pub(crate) fn record_created_vertex(&mut self, vertex_id: VertexId) {
        if self.creation_journal.recording {
            self.creation_journal.vertices.push(vertex_id);
        }
    }

    #[inline]
    pub(crate) fn record_created_halfedge(&mut self, halfedge_id: HalfedgeId) {
        if self.creation_journal.recording {
            self.creation_journal.halfedges.push(halfedge_id);
        }
    }

    #[inline]
    pub(crate) fn record_created_face(&mut self, face_id: FaceId) {
        if self.creation_journal.recording {
            self.creation_journal.faces.push(face_id);
        }
    }
}
