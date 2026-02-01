#pragma once

#include <cassert>
#include <cstring>
#include <optional>
#include <span>
#include <vector>

#include "wuzy.h"

namespace wuzy {
template <typename Vec3>
struct RayCastResult {
    Vec3 normal;
    Vec3 hit_position;
    float t;
};

template <typename Vec3>
struct CollisionResult {
    Vec3 normal;
    float depth;
};

struct Collider {
    wuzy_collider collider;

    ~Collider() = default;
    Collider() = default;
    Collider(const Collider&) = delete;
    Collider& operator=(const Collider&) = delete;
    Collider(Collider&&) = delete;
    Collider& operator=(Collider&&) = delete;

    void set_transform(const float m[16]) { wuzy_collider_set_transform(&collider, m); }

    template <typename Mat4>
    void set_transform(const Mat4& m)
    {
        wuzy_collider_set_transform(&collider, &m[0][0]);
    }

    template <typename Vec3>
    Vec3 support(const Vec3& v) const
    {
        float sup[3];
        wuzy_collider_support(&collider, &v[0], sup);
        return { sup[0], sup[1], sup[2] };
    }

    template <typename Vec3>
    auto ray_cast(const Vec3& start, const Vec3& dir) const -> std::optional<RayCastResult<Vec3>>
    {
        wuzy_ray_cast_result res;
        if (wuzy_collider_ray_cast(&collider, &start[0], &dir[0], &res)) {
            return RayCastResult<Vec3> {
                { res.normal[0], res.normal[1], res.normal[2] },
                { res.hit_position[0], res.hit_position[1], res.hit_position[2] },
                res.t,
            };
        } else {
            return std::nullopt;
        }
    }

    template <typename Vec3>
    std::pair<Vec3, Vec3> get_aabb() const
    {
        float min[3], max[3];
        wuzy_collider_get_aabb(&collider, min, max);
        return { Vec3(min[0], min[1], min[2]), Vec3(max[0], max[1], max[2]) };
    }
};

struct TriangleCollider : public Collider {
    wuzy_triangle_collider_userdata userdata;

    template <typename Vec3>
    TriangleCollider(const Vec3& v0, const Vec3& v1, const Vec3& v2)
    {
        // Not cool, but enough for now
        std::memcpy(userdata.vertices[0], &v0, sizeof(float) * 3);
        std::memcpy(userdata.vertices[1], &v1, sizeof(float) * 3);
        std::memcpy(userdata.vertices[2], &v2, sizeof(float) * 3);
        wuzy_triangle_collider_init(&collider, &userdata);
    }

    TriangleCollider(const TriangleCollider& other) : userdata(other.userdata)
    {
        collider = other.collider;
        collider.userdata = &userdata;
    }
};

struct SphereCollider : public Collider {
    wuzy_sphere_collider_userdata userdata;

    SphereCollider(float radius)
    {
        userdata.radius = radius;
        wuzy_sphere_collider_init(&collider, &userdata);
    }

    SphereCollider(const SphereCollider& other) : userdata(other.userdata)
    {
        collider = other.collider;
        collider.userdata = &userdata;
    }
};

struct CapsuleCollider : public Collider {
    wuzy_capsule_collider_userdata userdata;

    template <typename Vec3>
    CapsuleCollider(const Vec3& half_up, float radius)
    {
        std::memcpy(userdata.half_up, &half_up[0], sizeof(float) * 3);
        userdata.radius = radius;
        wuzy_capsule_collider_init(&collider, &userdata);
    }

    CapsuleCollider(const CapsuleCollider& other) : userdata(other.userdata)
    {
        collider = other.collider;
        collider.userdata = &userdata;
    }
};

struct ConvexPolyhedronCollider : public Collider {
    wuzy_convex_polyhedron_collider_userdata userdata;
    std::vector<float> vertices;
    std::vector<uint32_t> indices;
    std::vector<float> normals;

    ConvexPolyhedronCollider(std::span<const float> v, std::span<const uint32_t> i)
        : vertices(v.begin(), v.end())
        , indices(i.begin(), i.end())
    {
        normals.resize(i.size()); // /3*3
        userdata = {
            .vertices = vertices.data(),
            .num_vertices = vertices.size() / 3,
            .face_indices = indices.data(),
            .num_faces = indices.size() / 3,
            .normals = normals.data(),
        };
        wuzy_convex_polyhedron_collider_init(&collider, &userdata);
    }

    ConvexPolyhedronCollider(const ConvexPolyhedronCollider& other)
        : userdata(other.userdata)
        , vertices(other.vertices)
        , indices(other.indices)
        , normals(other.normals)
    {
        collider = other.collider;
        collider.userdata = &userdata;
    }
};

inline std::optional<wuzy_simplex3d> gjk(
    const Collider& a, const Collider& b, wuzy_gjk_debug* debug = nullptr)
{
    wuzy_simplex3d res;
    if (wuzy_gjk(&a.collider, &b.collider, &res, debug)) {
        return res;
    } else {
        return std::nullopt;
    }
}

inline bool test_collision(const Collider& a, const Collider& b, wuzy_gjk_debug* debug = nullptr)
{
    return wuzy_test_collision(&a.collider, &b.collider, debug);
}

inline wuzy_collision_result epa(const Collider& a, const Collider& b,
    const wuzy_simplex3d& simplex, wuzy_epa_debug* debug = nullptr)
{
    return wuzy_epa(&a.collider, &b.collider, &simplex, debug);
}

template <typename Vec3>
std::optional<CollisionResult<Vec3>> get_collision(const Collider& a, const Collider& b,
    wuzy_gjk_debug* gjk_debug = nullptr, wuzy_epa_debug* epa_debug = nullptr)
{
    wuzy_collision_result res;
    if (wuzy_get_collision(&a.collider, &b.collider, &res, gjk_debug, epa_debug)) {
        return CollisionResult<Vec3> { { res.normal[0], res.normal[1], res.normal[2] }, res.depth };
    } else {
        return std::nullopt;
    }
}

template <typename Vec3>
std::optional<float> gjk_toi(
    const Collider& moving, const Collider& target, const Vec3& delta, int max_iterations = 12)
{
    float t = 0.0f;
    if (wuzy_gjk_toi(&moving.collider, &target.collider, &delta[0], max_iterations, &t)) {
        return t;
    } else {
        return std::nullopt;
    }
}

struct AabbTree {
    enum class UpdateMode {
        Default = WUZY_AABB_TREE_UPDATE_FLAGS_DEFAULT,
        Reinsert = WUZY_AABB_TREE_UPDATE_FLAGS_REINSERT,
        Refit = WUZY_AABB_TREE_UPDATE_FLAGS_REFIT,
    };

    wuzy_aabb_tree* aabb_tree;

    static size_t count_leaves(wuzy_aabb_tree_init_node* node)
    {
        if (node->userdata) {
            return 1;
        }
        return count_leaves(node->left) + count_leaves(node->right);
    }

    AabbTree(size_t max_num_leaves, wuzy_allocator* alloc = nullptr)
    {
        aabb_tree = wuzy_aabb_tree_create(max_num_leaves, alloc);
    }

    AabbTree(wuzy_aabb_tree_init_node* root_node, size_t max_num_leaves = 0,
        wuzy_allocator* alloc = nullptr)
    {
        if (max_num_leaves == 0) {
            max_num_leaves = count_leaves(root_node);
        }
        aabb_tree = wuzy_aabb_tree_create(max_num_leaves, alloc);
        [[maybe_unused]] const auto res = wuzy_aabb_tree_init(aabb_tree, root_node);
        assert(res);
    }

    ~AabbTree() { wuzy_aabb_tree_destroy(aabb_tree); }

    wuzy_aabb_tree_stats get_stats() const
    {
        wuzy_aabb_tree_stats stats;
        wuzy_aabb_tree_get_stats(aabb_tree, &stats);
        return stats;
    }

    std::vector<wuzy_aabb_tree_node> build(
        std::span<wuzy_collider*> colliders, std::span<uint64_t> bitmasks = {})
    {
        assert(bitmasks.empty() || colliders.size() == bitmasks.size());

        std::vector<wuzy_aabb_tree_init_node> init_nodes(colliders.size());
        for (size_t i = 0; i < colliders.size(); ++i) {
            init_nodes[i] = { { 0 }, colliders[i], bitmasks.size() ? bitmasks[i] : 0 };
            wuzy_collider_get_aabb(colliders[i], init_nodes[i].aabb_min, init_nodes[i].aabb_max);
        }
        wuzy_aabb_tree_build(aabb_tree, init_nodes.data(), init_nodes.size());

        std::vector<wuzy_aabb_tree_node> nodes(colliders.size(), wuzy_aabb_tree_node { 0 });
        for (size_t i = 0; i < colliders.size(); ++i) {
            nodes[i] = init_nodes[i].id;
        }
        return nodes;
    }

    void rebuild() { wuzy_aabb_tree_rebuild(aabb_tree); }

    wuzy_aabb_tree_node insert(Collider& collider, uint64_t bitmask = 0)
    {
        float min[3], max[3];
        wuzy_collider_get_aabb(&collider.collider, min, max);
        return wuzy_aabb_tree_insert(aabb_tree, &collider.collider, bitmask, min, max);
    }

    bool update(
        wuzy_aabb_tree_node node, uint64_t bitmask = 0, UpdateMode mode = UpdateMode::Default)
    {
        const auto collider = (wuzy_collider*)wuzy_aabb_tree_get_userdata(aabb_tree, node);
        float min[3], max[3];
        wuzy_collider_get_aabb(collider, min, max);
        return wuzy_aabb_tree_update(
            aabb_tree, node, bitmask, min, max, static_cast<wuzy_aabb_tree_update_mode>(mode));
    }

    Collider* get_collider(wuzy_aabb_tree_node node)
    {
        // Of course this only works if the wuzy_collider returned is actually part of a Collider!
        // This is likely UB, but adding a map that maps wuzy_collider* to Collider*, but the
        // pointers are always the same value is a bit silly.
        auto collider = reinterpret_cast<Collider*>(wuzy_aabb_tree_get_userdata(aabb_tree, node));
        return collider;
    }

    bool remove(wuzy_aabb_tree_node node) { return wuzy_aabb_tree_remove(aabb_tree, node); }

    struct NodeQuery {
        wuzy_aabb_tree_node_query* query;

        ~NodeQuery()
        {
            if (query) {
                wuzy_aabb_tree_node_query_destroy(query);
            }
        }

        template <typename Vec3>
        void begin(const Vec3& v, uint64_t bitmask = 0, wuzy_query_debug* debug = nullptr)
        {
            wuzy_aabb_tree_node_query_point_begin(query, &v[0], bitmask, debug);
        }

        template <typename Vec3>
        void begin(const Vec3& min, const Vec3& max, uint64_t bitmask = 0,
            wuzy_query_debug* debug = nullptr)
        {
            wuzy_aabb_tree_node_query_aabb_begin(query, &min[0], &max[0], bitmask, debug);
        }

        std::optional<wuzy_aabb_tree_node_query_result> next()
        {
            wuzy_aabb_tree_node_query_result node;
            if (wuzy_aabb_tree_node_query_next(query, &node, 1)) {
                return node;
            } else {
                return std::nullopt;
            }
        }

        std::vector<wuzy_aabb_tree_node_query_result> all()
        {
            std::vector<wuzy_aabb_tree_node_query_result> res;
            wuzy_aabb_tree_node_query_result node;
            while (wuzy_aabb_tree_node_query_next(query, &node, 1)) {
                res.push_back(node);
            }
            return res;
        }

        template <typename Vec3>
        std::optional<wuzy_ray_cast_colliders_result> ray_cast(const Vec3& start, const Vec3& dir,
            uint64_t bitmask = 0, wuzy_query_debug* debug = nullptr)
        {
            wuzy_ray_cast_colliders_result res;
            const auto hit = wuzy_aabb_tree_ray_cast_colliders(
                query, &start[0], &dir[0], bitmask, &res, 1, debug);
            if (hit) {
                return res;
            } else {
                return std::nullopt;
            }
        }

        template <typename Vec3>
        std::vector<wuzy_ray_cast_colliders_result> ray_cast(const Vec3& start, const Vec3& dir,
            size_t max_num_results, uint64_t bitmask = 0, wuzy_query_debug* debug = nullptr)
        {
            std::vector<wuzy_ray_cast_colliders_result> results(max_num_results);
            const auto n = wuzy_aabb_tree_ray_cast_colliders(
                query, &start[0], &dir[0], bitmask, results.data(), max_num_results, debug);
            results.resize(n);
            return results;
        }
    };

    NodeQuery create_node_query(wuzy_allocator* alloc = nullptr)
    {
        return { wuzy_aabb_tree_node_query_create(aabb_tree, alloc) };
    }

    std::vector<wuzy_aabb_tree_dump_node> dump_nodes() const
    {
        const auto size = wuzy_aabb_tree_dump(aabb_tree, nullptr, 0);
        std::vector<wuzy_aabb_tree_dump_node> nodes(size);
        const auto num = wuzy_aabb_tree_dump(aabb_tree, nodes.data(), nodes.size());
        nodes.resize(num);
        return nodes;
    }
};
}
