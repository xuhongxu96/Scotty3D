#include <algorithm>
#include <functional>
#include <iostream>
#include <queue>
#include <set>
#include <unordered_map>
#include <unordered_set>

#include "../geometry/halfedge.h"
#include "debug.h"

/* Note on local operation return types:

    The local operations all return a std::optional<T> type. This is used so that your
    implementation can signify that it does not want to perform the operation for
    whatever reason (e.g. you don't want to allow the user to erase the last vertex).

    An optional can have two values: std::nullopt, or a value of the type it is
    parameterized on. In this way, it's similar to a pointer, but has two advantages:
    the value it holds need not be allocated elsewhere, and it provides an API that
    forces the user to check if it is null before using the value.

    In your implementation, if you have successfully performed the operation, you can
    simply return the required reference:

            ... collapse the edge ...
            return collapsed_vertex_ref;

    And if you wish to deny the operation, you can return the null optional:

            return std::nullopt;

    Note that the stubs below all reject their duties by returning the null optional.
*/

/*
    This method should replace the given vertex and all its neighboring
    edges and faces with a single face, returning the new face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_vertex(Halfedge_Mesh::VertexRef v) {

    (void)v;
    return std::nullopt;
}

/*
    This method should erase the given edge and return an iterator to the
    merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_edge(Halfedge_Mesh::EdgeRef e) {

    (void)e;
    return std::nullopt;
}

/*
    This method should collapse the given edge and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(Halfedge_Mesh::EdgeRef e) {
    // 1. collect

    // halfedges
    auto h = e->halfedge();
    auto ht = h->twin();

    auto hp = h->loop_to_prev();
    auto htp = ht->loop_to_prev();

    auto h1 = h->next();
    auto h2 = h1->next();
    auto h1t = h1->twin();

    auto ht1 = ht->next();
    auto ht2 = ht1->next();
    auto ht1t = ht1->twin();
    auto ht2t = ht2->twin();

    // edges
    auto e1 = h1->edge();
    auto et2 = ht2->edge();

    // faces
    auto f = h->face();
    auto ft = ht->face();

    // vertices
    auto v0 = h->vertex();
    auto v1 = ht->vertex();

    // when the edge <v0,v1> is an edge of 1 or 2 triangles:
    auto vo = vertices_end();  // triangle <v0,v1,vo>
    auto vot = vertices_end(); // triangle <v0,vot,v1>

    std::unordered_map<unsigned int, HalfedgeRef> v0_neis;
    v0->foreach_halfedges([&v0_neis](HalfedgeRef v) {
        v0_neis.insert({v->next()->vertex()->id(), v});
        return true;
    });

    v1->foreach_halfedges([&](HalfedgeRef v1h) { // v1h is from v1
        auto v = v1h->next()->vertex();
        auto it = v0_neis.find(v->id());
        if(v == v0 || it == v0_neis.end()) return true;

        // v1h->face() == f is not enough, because v0 -> v may be not in the face, so <v,v0,v1> is
        // not necessarily a triangle.
        //
        // E.g. v0 -> v below belongs to the triangle <v,p,v0>, but there is no triangle <v,v0,v1>.
        // In such case, v is not a vo. This may happen when two faces overlap. In below, rectangle
        // <v,q,v0,v1> overlaps with triangle <v,q,v0>.
        //
        //     v     v1
        //       ----
        //     /| \  |
        //     /|  \ |
        //    / |----|
        //    / q   / v0
        //   /    /
        //   /  /
        //  / /
        //  /
        // p

        if(v1h->face() == f && it->second->twin()->face() == f) {
            assert(vo == vertices_end()); // must happen only once
            vo = v;
        } else if(v1h->twin()->face() == ft && it->second->face() == ft) {
            assert(vot == vertices_end()); // must happen only once
            vot = v;
        }

        return true;
    });

    // 2. reassign

    // short path for edge with no face
    if(h->face() == ht->face()) {
        v0->pos = (v0->pos + v1->pos) * 0.5;
        if(h->next() == ht) {
            if(ht->next() == h) {
                // will be a single vertex, which cannot pass validation
                return {};
            } else {
                v0->halfedge() = ht1;
                hp->next() = ht1;
                f->halfedge() = ht1;
            }
        } else {
            if(ht->next() == h) {
                v0->halfedge() = h1;
                htp->next() = h1;
                h1->vertex() = v0;
                f->halfedge() = h1;
            } else {
                hp->next() = h1;
                htp->next() = ht1;
                h1->vertex() = v0;
                f->halfedge() = h1;
            }
        }

        erase(v1);
        erase(h);
        erase(ht);
        erase(e);

        return v0;
    }

    // faces
    f->halfedge() = hp;
    ft->halfedge() = ht->next();

    // halfedges
    v1->foreach_halfedges([&](HalfedgeRef v1h) { // v1h is from v1
        auto v = v1h->next()->vertex();

        // skip v0, vo, vot
        if(v == v0 || v == vo || v == vot) return true;

        // change the v1 endpoint to v0
        v1h->vertex() = v0;

        return true;
    });

    if(vo == vertices_end()) {
        hp->next() = h->next();
    } else {
        // is triangle

        // halfedges
        h1t->loop_to_prev()->next() = h2;
        h2->next() = h1t->next() == ht ? ht1 : h1t->next();
        h2->face() = h1t->face();
        h2->face()->halfedge() = h2;

        // vertices
        vo->halfedge() = h2;

        // delete
        erase(h1);
        erase(h1t);
        erase(e1);
        erase(f);
    }

    if(vot == vertices_end()) {
        htp->next() = ht->next();
    } else {
        // is triangle

        // halfedges
        ht2t->loop_to_prev()->next() = ht1;
        ht1->next() = ht2t->next() == h ? h1 : ht2t->next();
        ht1->face() = ht2t->face();
        ht1->face()->halfedge() = ht1;

        // vertices
        vot->halfedge() = ht1t;

        // delete
        erase(ht2);
        erase(ht2t);
        erase(et2);
        erase(ft);
    }

    // vertices
    v0->pos = (v0->pos + v1->pos) * 0.5;
    v0->halfedge() = ht1;

    // delete
    erase(v1);
    erase(h);
    erase(ht);
    erase(e);

    return v0;
}

/*
    This method should collapse the given face and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(Halfedge_Mesh::FaceRef f) {

    (void)f;
    return std::nullopt;
}

/*
    This method should flip the given edge and return an iterator to the
    flipped edge.
*/
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(Halfedge_Mesh::EdgeRef e) {
    // 1. collect

    // halfedges
    auto h = e->halfedge();
    auto ht = h->twin();
    auto h1 = h->next();
    auto ht1 = ht->next();
    auto h2 = h1->next();
    auto ht2 = ht1->next();

    auto hp = h->loop_to_prev();
    auto htp = ht->loop_to_prev();

    // vertices
    auto v0 = h->vertex();
    auto v1 = ht->vertex();
    auto vh2 = h2->vertex();
    auto vht2 = ht2->vertex();

    // faces
    auto f = h->face();
    auto ft = ht->face();

    // 2. check

    // boundary?
    if(f->is_boundary() || ft->is_boundary()) return {};

    // will be non-manifold?
    bool no_existing_edge =
        vht2->foreach_neighbor([&vh2](VertexCRef v) { return v->id() != vh2->id(); });
    if(!no_existing_edge) return {};

    // 3. reassign

    // halfedges
    h->vertex() = vht2;
    h->next() = h2;

    ht->vertex() = vh2;
    ht->next() = ht2;

    h1->face() = ft;
    h1->next() = ht;

    ht1->face() = f;
    ht1->next() = h;

    hp->next() = ht1;
    htp->next() = h1;

    // vertices
    v0->halfedge() = ht1;
    v1->halfedge() = h1;

    // faces
    f->halfedge() = h;
    ft->halfedge() = ht;

    return e;
}

/*
    This method should split the given edge and return an iterator to the
    newly inserted vertex. The halfedge of this vertex should point along
    the edge that was split, rather than the new edges.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(Halfedge_Mesh::EdgeRef e) {
    // For triangle meshes only!

    // 1. collect

    // halfedges
    auto h = e->halfedge();
    if(h->is_boundary()) {
        h = h->twin();
    }

    auto h1 = h->next();
    auto h2 = h1->next();

    auto ht = h->twin();
    auto ht1 = ht->next();
    auto ht2 = ht1->next();

    bool is_boundary = ht->is_boundary();

    // vertices
    auto v0 = h->vertex();
    auto v1 = h1->vertex();
    auto v2 = h2->vertex();
    auto v2t = ht2->vertex();

    // faces
    auto f = h->face();
    auto ft = ht->face();

    // 2. new

    // vertices
    auto midv = new_vertex();
    midv->pos = (v0->pos + v1->pos) / 2.f;

    // faces
    auto newf = new_face();
    auto newft = is_boundary ? ft : new_face();

    // edges
    auto newedge0 = new_edge(); // mid - v2
    auto newedge1 = new_edge(); // mid - v0

    // halfedges
    auto newh0 = new_halfedge();  // v2 -> mid
    auto newht0 = new_halfedge(); // mid -> v2

    auto newh1 = new_halfedge();  // v0 -> mid
    auto newht1 = new_halfedge(); // mid -> v0

    EdgeRef newedge2;
    HalfedgeRef newh2, newht2;
    if(!is_boundary) {
        newedge2 = new_edge();   // mid - v0
        newh2 = new_halfedge();  // v2t -> mid
        newht2 = new_halfedge(); // mid -> v2t
    }

    // 3. reassign

    // vertices
    midv->halfedge() = h;
    v0->halfedge() = newh1;

    // halfedges
    h->vertex() = midv;
    ht->next() = is_boundary ? newht1 : newh2;
    h1->next() = newh0;
    h2->next() = newh1;
    h2->face() = newf;
    newh0->set_neighbors(h, newht0, v2, newedge0, f);
    newht0->set_neighbors(h2, newh0, midv, newedge0, newf);
    newh1->set_neighbors(newht0, newht1, v0, newedge1, newf);
    newht1->set_neighbors(ht1, newh1, midv, newedge1, newft);

    if(!is_boundary) {
        ht1->next() = newht2;
        ht1->face() = newft;
        newh2->set_neighbors(ht2, newht2, midv, newedge2, ft);
        newht2->set_neighbors(newht1, newh2, v2t, newedge2, newft);
    }

    // faces
    f->halfedge() = h;
    ft->halfedge() = ht;
    newf->halfedge() = h2;
    if(!is_boundary) {
        newft->halfedge() = ht1;
    }

    // edges
    newedge0->halfedge() = newh0;
    newedge1->halfedge() = newh1;
    if(!is_boundary) {
        newedge2->halfedge() = newh2;
    }

    return midv;
}

/* Note on the beveling process:

    Each of the bevel_vertex, bevel_edge, and bevel_face functions do not represent
    a full bevel operation. Instead, they should update the _connectivity_ of
    the mesh, _not_ the positions of newly created vertices. In fact, you should set
    the positions of new vertices to be exactly the same as wherever they "started from."

    When you click on a mesh element while in bevel mode, one of those three functions
    is called. But, because you may then adjust the distance/offset of the newly
    beveled face, we need another method of updating the positions of the new vertices.

    This is where bevel_vertex_positions, bevel_edge_positions, and
    bevel_face_positions come in: these functions are called repeatedly as you
    move your mouse, the position of which determins the normal and tangent offset
    parameters. These functions are also passed an array of the original vertex
    positions: for bevel_vertex, it has one element, the original vertex position,
    for bevel_edge, two for the two vertices, and for bevel_face, it has the original
    position of each vertex in order starting from face->halfedge. You should use these
    positions, as well as the normal and tangent offset fields to assign positions to
    the new vertices.

    Finally, note that the normal and tangent offsets are not relative values - you
    should compute a particular new position from them, not a delta to apply.
*/

/*
    This method should replace the vertex v with a face, corresponding to
    a bevel operation. It should return the new face.  NOTE: This method is
    only responsible for updating the *connectivity* of the mesh---it does not
    need to update the vertex positions. These positions will be updated in
    Halfedge_Mesh::bevel_vertex_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(Halfedge_Mesh::VertexRef v) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)v;
    return std::nullopt;
}

/*
    This method should replace the edge e with a face, corresponding to a
    bevel operation. It should return the new face. NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions. These positions will be updated in
    Halfedge_Mesh::bevel_edge_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(Halfedge_Mesh::EdgeRef e) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)e;
    return std::nullopt;
}

/*
    This method should replace the face f with an additional, inset face
    (and ring of faces around it), corresponding to a bevel operation. It
    should return the new face.  NOTE: This method is responsible for updating
    the *connectivity* of the mesh only---it does not need to update the vertex
    positions. These positions will be updated in
    Halfedge_Mesh::bevel_face_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_face(Halfedge_Mesh::FaceRef f) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    auto h = f->halfedge();
    auto new_center_f = new_face();

    HalfedgeRef last_rh;
    HalfedgeRef last_tht;
    HalfedgeRef first_lh;
    HalfedgeRef first_tht;

    VertexRef last_rv;
    VertexRef first_lv;

    bool first = true;

    while(first || h != f->halfedge()) {
        auto original_next = h->next();

        bool last = h->next() == f->halfedge();

        auto lv = h->vertex();
        auto rv = h->next()->vertex();

        // 1. new

        // faces
        auto new_f = new_face();

        // halfedges
        auto new_rh = new_halfedge();
        auto new_lh = new_halfedge();
        auto new_th = new_halfedge();
        auto new_tht = new_halfedge();

        // edges
        auto new_le = first ? new_edge() : last_rh->edge();
        auto new_re = last ? first_lh->edge() : new_edge();
        auto new_te = new_edge();

        // vertices
        auto new_lv = first ? new_vertex() : last_rv;
        auto new_rv = last ? first_lv : new_vertex();

        // 2. reassign

        // faces
        new_f->halfedge() = h;

        // halfedges
        h->next() = new_rh;
        h->face() = new_f;

        new_rh->vertex() = rv;
        new_rh->face() = new_f;
        new_rh->edge() = new_re;
        new_rh->next() = new_th;

        if(!first) last_rh->twin() = new_lh;
        if(last) new_rh->twin() = first_lh;

        new_th->set_neighbors(new_lh, new_tht, new_rv, new_te, new_f);

        new_lh->vertex() = new_lv;
        new_lh->face() = new_f;
        new_lh->edge() = new_le;
        new_lh->next() = h;
        if(!first) new_lh->twin() = last_rh;
        if(last) first_lh->twin() = new_rh;

        new_tht->vertex() = new_lv;
        new_tht->face() = new_center_f;
        new_tht->edge() = new_te;
        new_tht->twin() = new_th;

        if(!first) last_tht->next() = new_tht;
        if(last) new_tht->next() = first_tht;

        // edges
        new_le->halfedge() = new_lh;
        new_re->halfedge() = new_rh;
        new_te->halfedge() = new_th;

        // vertices
        if(first) {
            new_lv->pos = lv->pos;
            new_lv->halfedge() = new_tht;
        }

        if(!last) {
            new_rv->pos = rv->pos;
            new_rv->halfedge() = new_th;
        }

        // 3. memorize
        last_rh = new_rh;
        last_tht = new_tht;
        last_rv = new_rv;
        if(first) {
            first_lh = new_lh;
            first_lv = new_lv;
            first_tht = new_tht;
        }

        // 4. next
        first = false;
        h = original_next;
    }

    // 5. reassign faces
    new_center_f->halfedge() = first_tht;

    // 6. delete face
    erase(f);

    return new_center_f;
}

/*
    Compute new vertex positions for the vertices of the beveled vertex.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the original vertex position and its associated outgoing edge
    to compute a new vertex position along the outgoing edge.
*/
void Halfedge_Mesh::bevel_vertex_positions(const std::vector<Vec3>& start_positions,
                                           Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled edge.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the orig array) to compute an offset vertex position.

    Note that there is a 1-to-1 correspondence between halfedges in
    newHalfedges and vertex positions in start_positions. So, you can write
    loops of the form:

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vector3D pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_edge_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled face.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the start_positions array) to compute an offset vertex
    position.

    Note that there is a 1-to-1 correspondence between halfedges in
    new_halfedges and vertex positions in start_positions. So, you can write
    loops of the form:

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vec3 pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_face_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset,
                                         float normal_offset) {

    if(flip_orientation) normal_offset = -normal_offset;
    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    auto fnorm = face->normal().unit();

    size_t N = new_halfedges.size();

    for(size_t i = 0; i < N; ++i) {
        Vec3 pi = start_positions[i]; // get the original vertex
        Vec3 pni = start_positions[(i + 1) % N];
        Vec3 ppi = start_positions[(i + N - 1) % N];

        Vec3 bis = ((pni - pi).unit() + (ppi - pi).unit()).unit();
        float sinhalf = cross((pni - pi).unit(), bis).norm();

        auto newpos = pi - fnorm * normal_offset - bis * tangent_offset / sinhalf;

        new_halfedges[i]->vertex()->pos = newpos;
    }
}

struct VertexIndices {
    size_t size() const {
        return v_.size();
    }

    size_t operator[](size_t i) const {
        return v_[i];
    }

    size_t hash() const {
        return hash_;
    }

    void add(size_t i) {
        v_.push_back(i);

        auto h = std::hash<size_t>{}(i);
        hash_ <<= 1;
        hash_ ^= h;
    }

    void clear() {
        v_.clear();
    }

    bool operator==(const VertexIndices& o) const {
        if(size() != o.size()) return false;

        for(size_t i = 0; i < size(); ++i) {
            if((*this)[i] != o[i]) return false;
        }

        return true;
    }

private:
    using indices_type = std::vector<size_t>;
    indices_type v_;
    size_t hash_;
};

template<> struct std::hash<VertexIndices> {
    std::size_t operator()(VertexIndices const& indices) const noexcept {
        return indices.hash();
    }
};

/*
    Splits all non-triangular faces into triangles.
*/
void Halfedge_Mesh::triangulate() {
    std::unordered_map<VertexIndices, std::pair<float, size_t>> memory;
    std::vector<HalfedgeRef> domain;

    // edge length sum as weight
    auto calc_weight = [](VertexCRef v0, VertexCRef v1, VertexCRef v2) {
        auto l0 = (v1->pos - v0->pos).norm();
        auto l1 = (v2->pos - v0->pos).norm();
        auto l2 = (v2->pos - v1->pos).norm();

        return l0 + l1 + l2;
    };

    std::function<float(const VertexIndices&)> process_subdomain =
        [&](const VertexIndices& indices) -> float {
        if(indices.size() <= 2) {
            return 0.f;
        }

        auto it = memory.find(indices);
        if(it != memory.end()) {
            return it->second.first;
        }

        // 1. choose access edge arbitrarily
        auto v0 = domain[indices[0]]->vertex();
        auto v1 = domain[indices[1]]->vertex();

        // 2. for each other vertices
        float min_w = FLT_MAX;
        size_t best_i = 0;

        for(size_t i = 2; i < indices.size(); ++i) {
            auto v2 = domain[indices[i]]->vertex();

            auto wv = calc_weight(v0, v1, v2);

            VertexIndices d;
            for(size_t j = 1; j <= i; ++j) d.add(indices[j]);
            auto wd1 = process_subdomain(d);

            d.clear();
            for(size_t j = i; j < indices.size() + 1; ++j) d.add(indices[j % indices.size()]);
            auto wd2 = process_subdomain(d);

            float w = wv + wd1 + wd2;
            if(w < min_w) {
                min_w = w;
                best_i = i;
            }
        }

        memory.insert(it, {indices, {min_w, best_i}});
        return min_w;
    };

    std::function<void(const VertexIndices&)> triangulate_domain =
        [&](const VertexIndices& indices) {
            if(indices.size() <= 2) return;

            auto best = memory[indices].second;

            // 1. collect

            auto h0 = domain[indices[0]];
            auto v0 = h0->vertex();

            auto v1 = domain[indices[1]]->vertex();
            HalfedgeRef h1 = h0->next();
            while(h1->vertex() != v1) {
                h1 = h1->next();
            }

            // best vertex and halfedges
            auto vb = domain[indices[best]]->vertex();
            HalfedgeRef hb = h0->next();
            while(hb->vertex() != vb) {
                hb = hb->next();
            }

            if(indices.size() == 3) {
                auto nf = new_face();
                nf->halfedge() = h0;
                h0->face() = nf;
                h1->face() = nf;
                hb->face() = nf;
                return;
            }

            auto h0p = h0->loop_to_prev();
            auto hbp = hb->loop_to_prev();

            // 2. new

            // new face
            auto nf = new_face();

            EdgeRef ne_v0b;
            HalfedgeRef nh_v0b, nh_vb0;
            if(h0p != hb) {
                // new edges
                ne_v0b = new_edge();

                // new halfedges
                nh_v0b = new_halfedge();
                nh_vb0 = new_halfedge();

                // 3. reassign
                nh_vb0->set_neighbors(h0, nh_v0b, vb, ne_v0b, nf);

                nh_v0b->next() = hb;
                nh_v0b->twin() = nh_vb0;
                nh_v0b->vertex() = v0;
                nh_v0b->edge() = ne_v0b;

                ne_v0b->halfedge() = nh_v0b;
            } else {
                ne_v0b = hb->edge();
                nh_v0b = h0;
                nh_vb0 = hb;
            }

            EdgeRef ne_v1b;
            HalfedgeRef nh_v1b, nh_vb1;
            if(h1 != hbp) {
                // new edges
                ne_v1b = new_edge();

                // new halfedges
                nh_v1b = new_halfedge();
                nh_vb1 = new_halfedge();

                // 3. reassign
                nh_v1b->set_neighbors(nh_vb0, nh_vb1, v1, ne_v1b, nf);

                nh_vb1->next() = h1;
                nh_vb1->twin() = nh_v1b;
                nh_vb1->vertex() = vb;
                nh_vb1->edge() = ne_v1b;

                ne_v1b->halfedge() = nh_v1b;
            } else {
                ne_v1b = h1->edge();
                nh_v1b = h1;
                nh_vb1 = nh_vb0;
            }

            // 3. reassign

            // halfedges

            h0->face() = nf;
            h1->face() = nf;
            hb->face() = nf;

            h0->next() = nh_v1b;
            h0p->next() = nh_v0b;
            hbp->next() = nh_vb1;

            // faces
            nf->halfedge() = h0;

            VertexIndices d;
            for(size_t j = 1; j <= best; ++j) d.add(indices[j]);
            triangulate_domain(d);

            d.clear();
            for(size_t j = best; j < indices.size() + 1; ++j) d.add(indices[j % indices.size()]);
            triangulate_domain(d);
        };

    std::vector<FaceRef> old_faces;
    old_faces.reserve(faces.size());
    for(auto it = faces_begin(); it != faces_end(); ++it) {
        old_faces.push_back(it);
    }

    for(auto f : old_faces) {
        //{ auto f = faces_begin();
        domain.clear();
        memory.clear();

        auto h = f->halfedge();

        do {
            domain.push_back(h);
            h = h->next();
        } while(h != f->halfedge());

        VertexIndices d;
        for(size_t i = 0; i < domain.size(); ++i) d.add(i);

        process_subdomain(d);
        triangulate_domain(d);

        erase(f);
    }
}

/* Note on the quad subdivision process:

        Unlike the local mesh operations (like bevel or edge flip), we will perform
        subdivision by splitting *all* faces into quads "simultaneously."  Rather
        than operating directly on the halfedge data structure (which as you've
        seen is quite difficult to maintain!) we are going to do something a bit nicer:
           1. Create a raw list of vertex positions and faces (rather than a full-
              blown halfedge mesh).
           2. Build a new halfedge mesh from these lists, replacing the old one.
        Sometimes rebuilding a data structure from scratch is simpler (and even
        more efficient) than incrementally modifying the existing one.  These steps are
        detailed below.

  Step I: Compute the vertex positions for the subdivided mesh.
        Here we're going to do something a little bit strange: since we will
        have one vertex in the subdivided mesh for each vertex, edge, and face in
        the original mesh, we can nicely store the new vertex *positions* as
        attributes on vertices, edges, and faces of the original mesh. These positions
        can then be conveniently copied into the new, subdivided mesh.
        This is what you will implement in linear_subdivide_positions() and
        catmullclark_subdivide_positions().

  Steps II-IV are provided (see Halfedge_Mesh::subdivide()), but are still detailed
  here:

  Step II: Assign a unique index (starting at 0) to each vertex, edge, and
        face in the original mesh. These indices will be the indices of the
        vertices in the new (subdivided) mesh. They do not have to be assigned
        in any particular order, so long as no index is shared by more than one
        mesh element, and the total number of indices is equal to V+E+F, i.e.,
        the total number of vertices plus edges plus faces in the original mesh.
        Basically we just need a one-to-one mapping between original mesh elements
        and subdivided mesh vertices.

  Step III: Build a list of quads in the new (subdivided) mesh, as tuples of
        the element indices defined above. In other words, each new quad should be
        of the form (i,j,k,l), where i,j,k and l are four of the indices stored on
        our original mesh elements.  Note that it is essential to get the orientation
        right here: (i,j,k,l) is not the same as (l,k,j,i).  Indices of new faces
        should circulate in the same direction as old faces (think about the right-hand
        rule).

  Step IV: Pass the list of vertices and quads to a routine that clears
        the internal data for this halfedge mesh, and builds new halfedge data from
        scratch, using the two lists.
*/

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    simple linear interpolation, e.g., the edge midpoints and face
    centroids.
*/
void Halfedge_Mesh::linear_subdivide_positions() {

    // For each vertex, assign Vertex::new_pos to
    // its original position, Vertex::pos.

    // For each edge, assign the midpoint of the two original
    // positions to Edge::new_pos.

    // For each face, assign the centroid (i.e., arithmetic mean)
    // of the original vertex positions to Face::new_pos. Note
    // that in general, NOT all faces will be triangles!

    for(auto& f : faces) {
        f.new_pos = Vec3(0, 0, 0);

        float n = 0.f;
        auto h = f.halfedge();
        do {
            f.new_pos += h->vertex()->pos;
            ++n;
            h = h->next();
        } while(h != f.halfedge());
        f.new_pos /= n;
    }

    for(auto& e : edges) {
        auto h = e.halfedge();
        auto ht = h->twin();
        e.new_pos = (h->vertex()->pos + ht->vertex()->pos) / 2.f;
    }

    for(auto& v : vertices) {
        v.new_pos = v.pos;
    }
}

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos. The values of the positions are based on
    the Catmull-Clark rules for subdivision.

    Note: this will only be called on meshes without boundary
*/
void Halfedge_Mesh::catmullclark_subdivide_positions() {

    // The implementation for this routine should be
    // a lot like Halfedge_Mesh:linear_subdivide_positions:(),
    // except that the calculation of the positions themsevles is
    // slightly more involved, using the Catmull-Clark subdivision
    // rules. (These rules are outlined in the Developer Manual.)

    // Faces

    // Edges

    // Vertices
}

/*
    This routine should increase the number of triangles in the mesh
    using Loop subdivision. Note: this is will only be called on triangle meshes.
*/
void Halfedge_Mesh::loop_subdivide() {

    // Each vertex and edge of the original mesh can be associated with a
    // vertex in the new (subdivided) mesh.
    // Therefore, our strategy for computing the subdivided vertex locations is to
    // *first* compute the new positions
    // using the connectivity of the original (coarse) mesh. Navigating this mesh
    // will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse.  We
    // will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.

    // Compute new positions for all the vertices in the input mesh using
    // the Loop subdivision rule and store them in Vertex::new_pos.
    //    At this point, we also want to mark each vertex as being a vertex of the
    //    original mesh. Use Vertex::is_new for this.

    // Next, compute the subdivided vertex positions associated with edges, and
    // store them in Edge::new_pos.

    // Next, we're going to split every edge in the mesh, in any order.
    // We're also going to distinguish subdivided edges that came from splitting
    // an edge in the original mesh from new edges by setting the boolean Edge::is_new.
    // Note that in this loop, we only want to iterate over edges of the original mesh.
    // Otherwise, we'll end up splitting edges that we just split (and the
    // loop will never end!)

    // Now flip any new edge that connects an old and new vertex.

    // Finally, copy new vertex positions into the Vertex::pos.
}

/*
    Isotropic remeshing. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if this is not a triangle mesh)
*/
bool Halfedge_Mesh::isotropic_remesh() {

    // Compute the mean edge length.
    // Repeat the four main steps for 5 or 6 iterations
    // -> Split edges much longer than the target length (being careful about
    //    how the loop is written!)
    // -> Collapse edges much shorter than the target length.  Here we need to
    //    be EXTRA careful about advancing the loop, because many edges may have
    //    been destroyed by a collapse (which ones?)
    // -> Now flip each edge if it improves vertex degree
    // -> Finally, apply some tangential smoothing to the vertex positions

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate is called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}

/* Helper type for quadric simplification */
struct Edge_Record {
    Edge_Record() {
    }
    Edge_Record(std::unordered_map<Halfedge_Mesh::VertexRef, Mat4>& vertex_quadrics,
                Halfedge_Mesh::EdgeRef e)
        : edge(e) {

        // Compute the combined quadric from the edge endpoints.
        // -> Build the 3x3 linear system whose solution minimizes the quadric error
        //    associated with these two endpoints.
        // -> Use this system to solve for the optimal position, and store it in
        //    Edge_Record::optimal.
        // -> Also store the cost associated with collapsing this edge in
        //    Edge_Record::cost.
    }
    Halfedge_Mesh::EdgeRef edge;
    Vec3 optimal;
    float cost;
};

/* Comparison operator for Edge_Records so std::set will properly order them */
bool operator<(const Edge_Record& r1, const Edge_Record& r2) {
    if(r1.cost != r2.cost) {
        return r1.cost < r2.cost;
    }
    Halfedge_Mesh::EdgeRef e1 = r1.edge;
    Halfedge_Mesh::EdgeRef e2 = r2.edge;
    return &*e1 < &*e2;
}

/** Helper type for quadric simplification
 *
 * A PQueue is a minimum-priority queue that
 * allows elements to be both inserted and removed from the
 * queue.  Together, one can easily change the priority of
 * an item by removing it, and re-inserting the same item
 * but with a different priority.  A priority queue, for
 * those who don't remember or haven't seen it before, is a
 * data structure that always keeps track of the item with
 * the smallest priority or "score," even as new elements
 * are inserted and removed.  Priority queues are often an
 * essential component of greedy algorithms, where one wants
 * to iteratively operate on the current "best" element.
 *
 * PQueue is templated on the type T of the object
 * being queued.  For this reason, T must define a comparison
 * operator of the form
 *
 *    bool operator<( const T& t1, const T& t2 )
 *
 * which returns true if and only if t1 is considered to have a
 * lower priority than t2.
 *
 * Basic use of a PQueue might look
 * something like this:
 *
 *    // initialize an empty queue
 *    PQueue<myItemType> queue;
 *
 *    // add some items (which we assume have been created
 *    // elsewhere, each of which has its priority stored as
 *    // some kind of internal member variable)
 *    queue.insert( item1 );
 *    queue.insert( item2 );
 *    queue.insert( item3 );
 *
 *    // get the highest priority item currently in the queue
 *    myItemType highestPriorityItem = queue.top();
 *
 *    // remove the highest priority item, automatically
 *    // promoting the next-highest priority item to the top
 *    queue.pop();
 *
 *    myItemType nextHighestPriorityItem = queue.top();
 *
 *    // Etc.
 *
 *    // We can also remove an item, making sure it is no
 *    // longer in the queue (note that this item may already
 *    // have been removed, if it was the 1st or 2nd-highest
 *    // priority item!)
 *    queue.remove( item2 );
 *
 */
template<class T> struct PQueue {
    void insert(const T& item) {
        queue.insert(item);
    }
    void remove(const T& item) {
        if(queue.find(item) != queue.end()) {
            queue.erase(item);
        }
    }
    const T& top(void) const {
        return *(queue.begin());
    }
    void pop(void) {
        queue.erase(queue.begin());
    }
    size_t size() {
        return queue.size();
    }

    std::set<T> queue;
};

/*
    Mesh simplification. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if you can't simplify the mesh any
    further without destroying it.)
*/
bool Halfedge_Mesh::simplify() {

    std::unordered_map<VertexRef, Mat4> vertex_quadrics;
    std::unordered_map<FaceRef, Mat4> face_quadrics;
    std::unordered_map<EdgeRef, Edge_Record> edge_records;
    PQueue<Edge_Record> edge_queue;

    // Compute initial quadrics for each face by simply writing the plane equation
    // for the face in homogeneous coordinates. These quadrics should be stored
    // in face_quadrics
    // -> Compute an initial quadric for each vertex as the sum of the quadrics
    //    associated with the incident faces, storing it in vertex_quadrics
    // -> Build a priority queue of edges according to their quadric error cost,
    //    i.e., by building an Edge_Record for each edge and sticking it in the
    //    queue. You may want to use the above PQueue<Edge_Record> for this.
    // -> Until we reach the target edge budget, collapse the best edge. Remember
    //    to remove from the queue any edge that touches the collapsing edge
    //    BEFORE it gets collapsed, and add back into the queue any edge touching
    //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
    //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
    //    top of the queue.

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}
