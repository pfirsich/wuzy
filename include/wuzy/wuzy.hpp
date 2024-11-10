#pragma once

#include <optional>
#include <span>
#include <vector>

#include "wuzy.h"

namespace wuzy {
struct Collider {
    wuzy_collider collider;

    Collider() = default;
    Collider(const Collider&) = delete;
    Collider& operator=(const Collider&) = delete;
    Collider(Collider&&) = delete;
    Collider& operator=(Collider&&) = delete;

    void set_transform(const wuzy_mat4& m) { wuzy_collider_set_transform(&collider, &m); }

    wuzy_vec3 support(const wuzy_vec3& direction) const
    {
        return wuzy_collider_support(&collider, direction);
    }

    std::optional<wuzy_ray_cast_result> ray_cast(
        const wuzy_vec3& start, const wuzy_vec3& direction) const
    {
        wuzy_ray_cast_result res;
        if (wuzy_collider_ray_cast(&collider, start, direction, &res)) {
            return res;
        } else {
            return std::nullopt;
        }
    }

    wuzy_aabb get_aabb() const { return wuzy_collider_get_aabb(&collider); }
};

struct TriangleCollider : public Collider {
    wuzy_triangle_collider_userdata userdata;

    TriangleCollider(const wuzy_vec3& v0, const wuzy_vec3& v1, const wuzy_vec3& v2)
    {
        userdata.vertices[0] = v0;
        userdata.vertices[1] = v1;
        userdata.vertices[2] = v2;
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

struct ConvexPolyhedronCollider : public Collider {
    wuzy_convex_polyhedron_collider_userdata userdata;
    std::vector<wuzy_vec3> vertices;
    std::vector<wuzy_face_indices> faces;
    std::vector<wuzy_vec3> normals;

    ConvexPolyhedronCollider(std::span<const wuzy_vec3> v, std::span<const wuzy_face_indices> f)
        : vertices(v.begin(), v.end())
        , faces(f.begin(), f.end())
    {
        normals.resize(faces.size());
        userdata = {
            .vertices = vertices.data(),
            .num_vertices = vertices.size(),
            .face_indices = faces.data(),
            .num_faces = faces.size(),
            .normals = normals.data(),
        };
        wuzy_convex_polyhedron_collider_init(&collider, &userdata);
    }

    ConvexPolyhedronCollider(const ConvexPolyhedronCollider& other)
        : userdata(other.userdata)
        , vertices(other.vertices)
        , faces(other.faces)
        , normals(other.normals)
    {
        collider = other.collider;
        collider.userdata = &userdata;
    }
};

std::optional<wuzy_simplex3d> gjk(
    const Collider& a, const Collider& b, wuzy_gjk_debug* debug = nullptr)
{
    wuzy_simplex3d res;
    if (wuzy_gjk(&a.collider, &b.collider, &res, debug)) {
        return res;
    } else {
        return std::nullopt;
    }
}

bool test_collision(const Collider& a, const Collider& b, wuzy_gjk_debug* debug = nullptr)
{
    return wuzy_test_collision(&a.collider, &b.collider, debug);
}

wuzy_collision_result epa(const Collider& a, const Collider& b, const wuzy_simplex3d& simplex,
    wuzy_epa_debug* debug = nullptr)
{
    return wuzy_epa(&a.collider, &b.collider, &simplex, debug);
}

std::optional<wuzy_collision_result> get_collision(const Collider& a, const Collider& b,
    wuzy_gjk_debug* gjk_debug = nullptr, wuzy_epa_debug* epa_debug = nullptr)
{
    wuzy_collision_result res;
    if (wuzy_get_collision(&a.collider, &b.collider, &res, gjk_debug, epa_debug)) {
        return res;
    } else {
        return std::nullopt;
    }
}

struct AabbTree {
    wuzy_aabb_tree* aabb_tree;

    AabbTree(size_t max_num_leaves, wuzy_allocator* alloc = nullptr)
    {
        aabb_tree = wuzy_aabb_tree_create(max_num_leaves, alloc);
    }

    ~AabbTree() { wuzy_aabb_tree_destroy(aabb_tree); }

    wuzy_aabb_tree_node insert(Collider& collider, uint64_t bitmask = 0)
    {
        return wuzy_aabb_tree_insert(aabb_tree, &collider.collider, bitmask);
    }

    bool update(wuzy_aabb_tree_node node, uint64_t bitmask = 0)
    {
        return wuzy_aabb_tree_update(aabb_tree, node, bitmask);
    }

    Collider* get_collider(wuzy_aabb_tree_node node)
    {
        // Of course this only works if the wuzy_collider returned is actually part of a Collider!
        // This is likely UB, but adding a map that maps wuzy_collider* to Collider*, but the
        // pointers are always the same value is a bit silly.
        auto collider = reinterpret_cast<Collider*>(wuzy_aabb_tree_get_collider(aabb_tree, node));
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

        void begin(const wuzy_vec3& point, uint64_t bitmask = 0)
        {
            wuzy_aabb_tree_node_query_point_begin(query, point, bitmask);
        }

        void begin(const wuzy_aabb& aabb, uint64_t bitmask = 0)
        {
            wuzy_aabb_tree_node_query_aabb_begin(query, &aabb, bitmask);
        }

        std::optional<wuzy_aabb_tree_node> next()
        {
            wuzy_aabb_tree_node node;
            if (wuzy_aabb_tree_node_query_next(query, &node, 1)) {
                return node;
            } else {
                return std::nullopt;
            }
        }

        std::vector<wuzy_aabb_tree_node> all()
        {
            std::vector<wuzy_aabb_tree_node> res;
            wuzy_aabb_tree_node node;
            while (wuzy_aabb_tree_node_query_next(query, &node, 1)) {
                res.push_back(node);
            }
            return res;
        }

        std::optional<wuzy_aabb_tree_ray_cast_result> ray_cast(
            const wuzy_vec3& start, const wuzy_vec3& direction, uint64_t bitmask = 0)
        {
            wuzy_aabb_tree_ray_cast_result res;
            if (wuzy_aabb_tree_node_query_ray_cast(query, start, direction, bitmask, &res)) {
                return res;
            } else {
                return std::nullopt;
            }
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
