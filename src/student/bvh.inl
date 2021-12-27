
#include "../rays/bvh.h"
#include "debug.h"
#include <cmath>
#include <functional>
#include <iostream>
#include <stack>

namespace PT {

template<typename Primitive>
void BVH<Primitive>::build(std::vector<Primitive>&& prims, size_t max_leaf_size) {

    // NOTE (PathTracer):
    // This BVH is parameterized on the type of the primitive it contains. This allows
    // us to build a BVH over any type that defines a certain interface. Specifically,
    // we use this to both build a BVH over triangles within each Tri_Mesh, and over
    // a variety of Objects (which might be Tri_Meshes, Spheres, etc.) in Pathtracer.
    //
    // The Primitive interface must implement these two functions:
    //      BBox bbox() const;
    //      Trace hit(const Ray& ray) const;
    // Hence, you may call bbox() and hit() on any value of type Primitive.
    //
    // Finally, also note that while a BVH is a tree structure, our BVH nodes don't
    // contain pointers to children, but rather indicies. This is because instead
    // of allocating each node individually, the BVH class contains a vector that
    // holds all of the nodes. Hence, to get the child of a node, you have to
    // look up the child index in this vector (e.g. nodes[node.l]). Similarly,
    // to create a new node, don't allocate one yourself - use BVH::new_node, which
    // returns the index of a newly added node.

    // Keep these
    nodes.clear();
    primitives = std::move(prims);

    // (PathTracer): Task 3
    // Construct a BVH from the given vector of primitives and maximum leaf
    // size configuration. The starter code builds a BVH with a
    // single leaf node (which is also the root) that encloses all the
    // primitives.
    static constexpr size_t N_BUCKET = 16;

    std::function<size_t(std::vector<Primitive>::iterator, std::vector<Primitive>::iterator)>
        partition_prims = [&partition_prims, this](std::vector<Primitive>::iterator begin,
                                                   std::vector<Primitive>::iterator end) -> size_t {
        size_t begin_i = begin - primitives.begin();

        BBox full_box;
        for(auto it = begin; it != end; ++it) full_box.enclose(it->bbox());

        auto node_i = new_node(full_box, begin_i, end - begin);

        if(begin + 1 == end) return node_i;

        // float sn = full_box.surface_area();

        size_t best_axis = 3;
        float best_partition_x = 0.f;
        float min_cost = INFINITY;

        for(size_t axis = 0; axis < 3; ++axis) {
            // init buckets
            BBox bucket_boxes[N_BUCKET];
            size_t bucket_prim_counts[N_BUCKET];
            memset(bucket_prim_counts, 0, sizeof(size_t) * N_BUCKET);

            float min_x = full_box.min.data[axis];
            float max_x = full_box.max.data[axis];
            float bucket_width = (max_x - min_x) / N_BUCKET;

            if(bucket_width < FLT_EPSILON) continue;

            for(auto it = begin; it != end; ++it) {
                BBox prim_box = it->bbox();
                float center = prim_box.center().data[axis];
                size_t bucket_index =
                    std::min(N_BUCKET - 1, static_cast<size_t>((center - min_x) / bucket_width));
                assert(bucket_index < N_BUCKET);

                bucket_boxes[bucket_index].enclose(prim_box);
                ++bucket_prim_counts[bucket_index];
            }

            for(size_t i = 0; i < N_BUCKET; ++i) {
                float partition_x = bucket_boxes[i].max.data[axis];

                float sa = .0f;
                size_t na = 0;
                for(size_t j = 0; j <= i; ++j) {
                    sa += bucket_boxes[j].surface_area();
                    na += bucket_prim_counts[j];
                }

                float sb = .0f;
                size_t nb = 0;
                for(size_t j = i + 1; j < N_BUCKET; ++j) {
                    sb += bucket_boxes[j].surface_area();
                    nb += bucket_prim_counts[j];
                }

                float cost = sa * na + sb * nb;
                if(cost < min_cost) {
                    min_cost = cost;
                    best_axis = axis;
                    best_partition_x = partition_x;
                }
            }
        }

        // recursively partition
        auto part_it = std::partition(begin, end, [&](const Primitive& p) {
            return p.bbox().center().data[best_axis] < best_partition_x;
        });

        if(part_it == begin || part_it == end) {
            return node_i;
        }

        nodes[node_i].l = partition_prims(begin, part_it);
        nodes[node_i].r = partition_prims(part_it, end);

        return node_i;
    };

    root_idx = partition_prims(primitives.begin(), primitives.end());
}

template<typename Primitive> Trace BVH<Primitive>::hit(const Ray& ray) const {

    // (PathTracer): Task 3
    // Implement ray - BVH intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.

    // The starter code simply iterates through all the primitives.
    // Again, remember you can use hit() on any Primitive value.

    std::function<void(const Ray&, const Node&, Trace&)> find_closest_hit =
        [this, &find_closest_hit](const Ray& ray, const Node& node, Trace& closest) {
            if(node.is_leaf()) {
                for(size_t i = node.start; i < node.start + node.size; ++i) {
                    Trace hit = primitives[i].hit(ray);
                    closest = Trace::min(closest, hit);
                }
                return;
            }

            Vec2 timesl(-INFINITY, INFINITY), timesr(-INFINITY, INFINITY);
            bool hitl = nodes[node.l].bbox.hit(ray, timesl);
            bool hitr = nodes[node.r].bbox.hit(ray, timesr);

            if(hitl && !hitr) {
                find_closest_hit(ray, nodes[node.l], closest);
            } else if(!hitl && hitr) {
                find_closest_hit(ray, nodes[node.r], closest);
            } else if(hitl && hitr) {
                if(timesl.x < timesr.x) {
                    find_closest_hit(ray, nodes[node.l], closest);
                    if(!closest.hit || timesr.x < closest.distance) {
                        find_closest_hit(ray, nodes[node.r], closest);
                    }
                } else {
                    find_closest_hit(ray, nodes[node.r], closest);
                    if(!closest.hit || timesl.x < closest.distance) {
                        find_closest_hit(ray, nodes[node.l], closest);
                    }
                }
            }
        };

    Trace ret;
    find_closest_hit(ray, nodes[root_idx], ret);
    return ret;
}

template<typename Primitive>
BVH<Primitive>::BVH(std::vector<Primitive>&& prims, size_t max_leaf_size) {
    build(std::move(prims), max_leaf_size);
}

template<typename Primitive> BVH<Primitive> BVH<Primitive>::copy() const {
    BVH<Primitive> ret;
    ret.nodes = nodes;
    ret.primitives = primitives;
    ret.root_idx = root_idx;
    return ret;
}

template<typename Primitive> bool BVH<Primitive>::Node::is_leaf() const {

    // A node is a leaf if l == r, since all interior nodes must have distinct children
    return l == r;
}

template<typename Primitive>
size_t BVH<Primitive>::new_node(BBox box, size_t start, size_t size, size_t l, size_t r) {
    Node n;
    n.bbox = box;
    n.start = start;
    n.size = size;
    n.l = l;
    n.r = r;
    nodes.push_back(n);
    return nodes.size() - 1;
}

template<typename Primitive> BBox BVH<Primitive>::bbox() const {
    return nodes[root_idx].bbox;
}

template<typename Primitive> std::vector<Primitive> BVH<Primitive>::destructure() {
    nodes.clear();
    return std::move(primitives);
}

template<typename Primitive> void BVH<Primitive>::clear() {
    nodes.clear();
    primitives.clear();
}

template<typename Primitive>
size_t BVH<Primitive>::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                                 const Mat4& trans) const {

    std::stack<std::pair<size_t, size_t>> tstack;
    tstack.push({root_idx, 0});
    size_t max_level = 0;

    if(nodes.empty()) return max_level;

    while(!tstack.empty()) {

        auto [idx, lvl] = tstack.top();
        max_level = std::max(max_level, lvl);
        const Node& node = nodes[idx];
        tstack.pop();

        Vec3 color = lvl == level ? Vec3(1.0f, 0.0f, 0.0f) : Vec3(1.0f);
        GL::Lines& add = lvl == level ? active : lines;

        BBox box = node.bbox;
        box.transform(trans);
        Vec3 min = box.min, max = box.max;

        auto edge = [&](Vec3 a, Vec3 b) { add.add(a, b, color); };

        edge(min, Vec3{max.x, min.y, min.z});
        edge(min, Vec3{min.x, max.y, min.z});
        edge(min, Vec3{min.x, min.y, max.z});
        edge(max, Vec3{min.x, max.y, max.z});
        edge(max, Vec3{max.x, min.y, max.z});
        edge(max, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, min.y, max.z});

        if(!node.is_leaf()) {
            tstack.push({node.l, lvl + 1});
            tstack.push({node.r, lvl + 1});
        } else {
            for(size_t i = node.start; i < node.start + node.size; i++) {
                size_t c = primitives[i].visualize(lines, active, level - lvl, trans);
                max_level = std::max(c + lvl, max_level);
            }
        }
    }
    return max_level;
}

} // namespace PT
