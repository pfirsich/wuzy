#include "wuzy/wuzy-hl.h"

#include <cassert>
#include <cfloat>
#include <cmath>
#include <cstring>

#include "shared.hpp"

#include <fmt/core.h>

#define EXPORT extern "C"

using namespace wuzy;

namespace wuzy_hl {
struct Convex {
    uint32_t generation = 1;
    uint32_t next_free;

    wuzy_convex_polyhedron_collider_userdata userdata;
};

struct Mesh {
    uint32_t generation = 1;
    uint32_t next_free;

    wuzy_triangle_collider_userdata* tris;
    size_t num_tris;
    wuzy_aabb_tree* tree;
    wuzy_aabb_tree_node_query* query;
    float aabb_min[3], aabb_max[3];
};

struct Empty { };

struct Collider {
    enum class Type : uint8_t { Sphere, ConvexPolyhedron, Mesh };

    uint32_t generation = 1;
    uint32_t next_free;

    wuzy_collider collider;
    union {
        Empty empty;
        wuzy_sphere_collider_userdata sphere;
        wuzy_hl_mesh_id mesh;
    } collider_userdata;

    wuzy_aabb_tree_node node; // node in global aabb tree
    uint64_t bitmask;
    void* userdata;
    Type type;

    float aabb_min[3], aabb_max[3];
};

template <typename T>
struct Pool {
    T* data;
    size_t capacity;
    uint32_t free_head;

    void init(wuzy_allocator* alloc, size_t c)
    {
        assert(c <= 0x100'0000);
        capacity = c;
        data = allocate<T>(alloc, capacity);
        free_head = 0;

        for (size_t i = 0; i < capacity; ++i) {
            data[i].next_free = (uint32_t)(i + 1);
        }
    }

    void free(wuzy_allocator* alloc) { deallocate(alloc, data, capacity); }

    T* get(uint64_t id)
    {
        const auto gen = id >> 24;
        const auto idx = 0xFF'FFFF & id;
        assert(idx < capacity && data[idx].generation == gen);
        return &data[idx];
    }

    T* insert()
    {
        const auto idx = free_head;
        assert(idx < capacity);
        if (idx >= capacity) {
            return nullptr;
        }
        free_head = data[idx].next_free;
        data[idx].next_free = UINT32_MAX;
        return &data[idx];
    }

    void remove(T* e)
    {
        e->generation++;
        assert(e->generation <= 0xFF'FFFF);
        const auto idx = e - data;
        e->next_free = free_head;
        free_head = (uint32_t)idx;
    }

    uint64_t get_id(const T* e)
    {
        const auto idx = e - data;
        return (e->generation << 24) | (uint32_t)idx;
    }
};

struct State {
    wuzy_allocator alloc;
    Pool<Convex> convex_polyhedra;
    Pool<Mesh> meshes;
    Pool<Collider> colliders;
    wuzy_aabb_tree* tree;
    wuzy_aabb_tree_node_query* query;
    wuzy_ray_cast_tris_result* rc_tri_tmp;
    size_t rc_tri_tmp_size;
};

State* state = nullptr;

EXPORT void wuzy_hl_init(wuzy_hl_create_params params)
{
    auto alloc = params.allocator ? params.allocator : default_allocator();
    state = allocate<State>(alloc);

    state->alloc = *alloc;

    state->tree = wuzy_aabb_tree_create(params.max_num_colliders, alloc);
    state->query = wuzy_aabb_tree_node_query_create(state->tree, alloc);

    state->convex_polyhedra.init(
        alloc, params.max_num_convex_polyhedra ? params.max_num_convex_polyhedra : 16);
    state->meshes.init(alloc, params.max_num_meshes ? params.max_num_meshes : 16);
    state->colliders.init(alloc, params.max_num_colliders);

    state->rc_tri_tmp_size = 64;
    state->rc_tri_tmp = allocate<wuzy_ray_cast_tris_result>(alloc, state->rc_tri_tmp_size);
}

template <typename T, typename Func>
void destroy_all(Pool<T>& pool, Func&& destroy)
{
    for (size_t i = 0; i < pool.capacity; ++i) {
        if (pool.data[i].next_free == UINT32_MAX) { // alive
            destroy({ pool.get_id(&pool.data[i]) });
        }
    }
}

EXPORT void wuzy_hl_shutdown()
{
    deallocate(&state->alloc, state->rc_tri_tmp, state->rc_tri_tmp_size);

    destroy_all(state->colliders, wuzy_hl_collider_destroy);
    state->colliders.free(&state->alloc);
    destroy_all(state->meshes, wuzy_hl_mesh_destroy);
    state->meshes.free(&state->alloc);
    destroy_all(state->convex_polyhedra, wuzy_hl_convex_polyhedron_destroy);
    state->convex_polyhedra.free(&state->alloc);

    wuzy_aabb_tree_node_query_destroy(state->query);
    wuzy_aabb_tree_destroy(state->tree);

    deallocate(&state->alloc, state);
    state = nullptr;
}

EXPORT wuzy_hl_convex_polyhedron_id wuzy_hl_convex_polyhedron_create(
    const float* vertices, size_t num_vertices, const uint32_t* face_indices, size_t num_faces)
{
    auto convex = state->convex_polyhedra.insert();

    convex->userdata.vertices = allocate<float>(&state->alloc, num_vertices * 3);
    convex->userdata.num_vertices = num_vertices;
    convex->userdata.face_indices = allocate<uint32_t>(&state->alloc, num_faces * 3);
    convex->userdata.num_faces = num_faces;
    convex->userdata.normals = allocate<float>(&state->alloc, num_faces * 3);

    std::memcpy(convex->userdata.vertices, vertices, num_vertices * 3 * sizeof(float));
    std::memcpy(convex->userdata.face_indices, face_indices, num_faces * 3 * sizeof(uint32_t));

    wuzy_calculate_normals(convex->userdata.vertices, convex->userdata.num_vertices,
        convex->userdata.face_indices, convex->userdata.num_faces, convex->userdata.normals);

    return { state->convex_polyhedra.get_id(convex) };
}

EXPORT void wuzy_hl_convex_polyhedron_destroy(wuzy_hl_convex_polyhedron_id id)
{
    auto convex = state->convex_polyhedra.get(id.id);
    deallocate(&state->alloc, convex->userdata.vertices, convex->userdata.num_vertices * 3);
    deallocate(&state->alloc, convex->userdata.face_indices, convex->userdata.num_faces * 3);
    deallocate(&state->alloc, convex->userdata.normals, convex->userdata.num_faces * 3);
    state->convex_polyhedra.remove(convex);
}

static void init_aabb(float min[3], float max[3])
{
    for (size_t c = 0; c < 3; ++c) {
        min[c] = FLT_MAX;
        max[c] = -FLT_MAX;
    }
}

static void get_tri_aabb(const wuzy_triangle_collider_userdata& tri, float min[3], float max[3])
{
    for (size_t v = 0; v < 3; ++v) {
        for (size_t c = 0; c < 3; ++c) {
            min[c] = std::fmin(min[c], tri.vertices[v][c]);
            max[c] = std::fmax(max[c], tri.vertices[v][c]);
        }
    }
}

EXPORT wuzy_hl_mesh_id wuzy_hl_mesh_create(
    const float* vertices, size_t num_vertices, const uint32_t* face_indices, size_t num_faces)
{
    auto mesh = state->meshes.insert();

    mesh->tris = allocate<wuzy_triangle_collider_userdata>(&state->alloc, num_faces);
    mesh->num_tris = num_faces;
    mesh->tree = wuzy_aabb_tree_create(num_faces, &state->alloc);
    mesh->query = wuzy_aabb_tree_node_query_create(mesh->tree, &state->alloc);

    init_aabb(mesh->aabb_min, mesh->aabb_max);

    for (size_t i = 0; i < num_faces; ++i) {
        const auto i0 = face_indices[i * 3 + 0];
        const auto i1 = face_indices[i * 3 + 1];
        const auto i2 = face_indices[i * 3 + 2];

        assert(i0 < num_vertices && i1 < num_vertices && i2 < num_vertices);
        std::memcpy(mesh->tris[i].vertices[0], vertices + i0 * 3, sizeof(float) * 3);
        std::memcpy(mesh->tris[i].vertices[1], vertices + i1 * 3, sizeof(float) * 3);
        std::memcpy(mesh->tris[i].vertices[2], vertices + i2 * 3, sizeof(float) * 3);

        wuzy_triangle_collider_userdata_calculate_normal(&mesh->tris[i]);

        get_tri_aabb(mesh->tris[i], mesh->aabb_min, mesh->aabb_max);
    }

    auto nodes = allocate<wuzy_aabb_tree_init_node>(&state->alloc, mesh->num_tris);
    std::memset(nodes, 0, mesh->num_tris * sizeof(wuzy_aabb_tree_init_node));
    for (size_t t = 0; t < mesh->num_tris; ++t) {
        nodes[t].userdata = &mesh->tris[t];
        init_aabb(nodes[t].aabb_min, nodes[t].aabb_max);
        get_tri_aabb(mesh->tris[t], nodes[t].aabb_min, nodes[t].aabb_max);
    }
    wuzy_aabb_tree_build(mesh->tree, nodes, mesh->num_tris);
    deallocate(&state->alloc, nodes, mesh->num_tris);

    return { state->meshes.get_id(mesh) };
}

EXPORT void wuzy_hl_mesh_destroy(wuzy_hl_mesh_id id)
{
    auto mesh = state->meshes.get(id.id);
    wuzy_aabb_tree_node_query_destroy(mesh->query);
    wuzy_aabb_tree_destroy(mesh->tree);
    deallocate(&state->alloc, mesh->tris, mesh->num_tris);
    state->meshes.remove(mesh);
}

static void load_identity(float m[16])
{
    // clang-format off
    static constexpr float identity[16] = {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    };
    // clang-format on
    std::memcpy(m, identity, sizeof(float) * 16);
}

static wuzy_hl_collider_id common_init(Collider* col)
{
    load_identity(col->collider.transform);
    load_identity(col->collider.inverse_transform);
    col->bitmask = 0;
    col->userdata = nullptr;
    col->node = wuzy_aabb_tree_insert(state->tree, col, col->bitmask, col->aabb_min, col->aabb_max);
    return { state->colliders.get_id(col) };
}

EXPORT wuzy_hl_collider_id wuzy_hl_collider_create_sphere(float radius)
{
    auto col = state->colliders.insert();
    col->type = Collider::Type::Sphere;
    col->collider_userdata.sphere = { radius };
    wuzy_sphere_collider_init(&col->collider, &col->collider_userdata.sphere);
    wuzy_collider_get_aabb(&col->collider, col->aabb_min, col->aabb_max);
    return common_init(col);
}

EXPORT wuzy_hl_collider_id wuzy_hl_collider_create_convex_polyhedron(
    wuzy_hl_convex_polyhedron_id convex_id)
{
    auto convex = state->convex_polyhedra.get(convex_id.id);
    auto col = state->colliders.insert();
    col->type = Collider::Type::ConvexPolyhedron;
    wuzy_convex_polyhedron_collider_init(&col->collider, &convex->userdata);
    wuzy_collider_get_aabb(&col->collider, col->aabb_min, col->aabb_max);
    return common_init(col);
}

EXPORT wuzy_hl_collider_id wuzy_hl_collider_create_mesh(wuzy_hl_mesh_id mesh_id)
{
    const auto mesh = state->meshes.get(mesh_id.id);
    auto col = state->colliders.insert();
    col->type = Collider::Type::Mesh;
    // This collider is a bit weird. We will use it for collisions, but replace the userdata
    // with an individual triangle beforehand.
    col->collider.userdata = nullptr;
    col->collider.support_func = wuzy_triangle_collider_support;
    col->collider.ray_cast_func = nullptr;
    col->collider_userdata.mesh = mesh_id;
    std::memcpy(col->aabb_min, mesh->aabb_min, sizeof(float) * 3);
    std::memcpy(col->aabb_max, mesh->aabb_max, sizeof(float) * 3);
    return common_init(col);
}

static bool is_convex(const Collider* col)
{
    return col->type != Collider::Type::Mesh;
}

EXPORT void wuzy_hl_collider_destroy(wuzy_hl_collider_id id)
{
    auto col = state->colliders.get(id.id);
    wuzy_aabb_tree_remove(state->tree, col->node);
    state->colliders.remove(col);
}

EXPORT void wuzy_hl_collider_set_userdata(wuzy_hl_collider_id id, void* userdata)
{
    auto col = state->colliders.get(id.id);
    col->userdata = userdata;
}

EXPORT void* wuzy_hl_collider_get_userdata(wuzy_hl_collider_id id)
{
    return state->colliders.get(id.id)->userdata;
}

EXPORT void wuzy_hl_collider_set_bitmask(wuzy_hl_collider_id id, uint64_t bitmask)
{
    auto col = state->colliders.get(id.id);
    col->bitmask = bitmask;
    wuzy_aabb_tree_update(state->tree, col->node, bitmask, col->aabb_min, col->aabb_max,
        WUZY_AABB_TREE_UPDATE_FLAGS_DEFAULT);
}

EXPORT uint64_t wuzy_hl_collider_get_bitmask(wuzy_hl_collider_id id)
{
    return state->colliders.get(id.id)->bitmask;
}

EXPORT void wuzy_hl_collider_set_transform(wuzy_hl_collider_id id, const float matrix[16])
{
    auto col = state->colliders.get(id.id);
    assert(col->type != Collider::Type::Mesh);
    wuzy_collider_set_transform(&col->collider, matrix);
    wuzy_collider_get_aabb(&col->collider, col->aabb_min, col->aabb_max);
    wuzy_aabb_tree_update(state->tree, col->node, col->bitmask, col->aabb_min, col->aabb_max,
        WUZY_AABB_TREE_UPDATE_FLAGS_DEFAULT);
}

EXPORT void wuzy_hl_collider_get_aabb(wuzy_hl_collider_id id, float min[3], float max[3])
{
    auto col = state->colliders.get(id.id);
    std::memcpy(min, col->aabb_min, sizeof(float) * 3);
    std::memcpy(max, col->aabb_max, sizeof(float) * 3);
}

static size_t collect_query(
    wuzy_hl_collider_id* colliders, size_t max_num_colliders, Collider* self = nullptr)
{
    wuzy_aabb_tree_node_query_result res;
    size_t num_colliders = 0;
    while (num_colliders < max_num_colliders
        && wuzy_aabb_tree_node_query_next(state->query, &res, 1)) {
        const auto col = (Collider*)res.node_userdata;
        if (self == col) {
            continue;
        }
        colliders[num_colliders++] = { state->colliders.get_id(col) };
    }
    return num_colliders;
}

EXPORT size_t wuzy_hl_query_point(const float point[3], uint64_t bitmask,
    wuzy_hl_collider_id* colliders, size_t max_num_colliders)
{
    wuzy_aabb_tree_node_query_point_begin(state->query, point, bitmask, nullptr);
    return collect_query(colliders, max_num_colliders);
}

EXPORT size_t wuzy_hl_query_aabb(const float min[3], const float max[3], uint64_t bitmask,
    wuzy_hl_collider_id* colliders, size_t max_num_colliders)
{
    wuzy_aabb_tree_node_query_aabb_begin(state->query, min, max, bitmask, nullptr);
    return collect_query(colliders, max_num_colliders);
}

EXPORT size_t wuzy_hl_query_candidates(wuzy_hl_collider_id id, uint64_t bitmask,
    wuzy_hl_collider_id* colliders, size_t max_num_colliders)
{
    const auto col = state->colliders.get(id.id);
    wuzy_aabb_tree_node_query_aabb_begin(
        state->query, col->aabb_min, col->aabb_max, bitmask, nullptr);
    return collect_query(colliders, max_num_colliders, col);
}

// Returns true is res is good enough to keep
template <typename T, typename IsBetter>
static size_t keep_best(
    T* results, size_t num_results, size_t max_num_results, const T& res, IsBetter&& is_better)
{
    if (num_results == max_num_results && !is_better(res, results[num_results - 1])) {
        // don't add
        return num_results;
    }
    assert(num_results < max_num_results || is_better(res, results[num_results - 1]));

    if (max_num_results == 1) {
        results[0] = res;
        return 1;
    }

    size_t insert = 0;
    while (insert < num_results && !is_better(res, results[insert])) {
        insert++;
    }

    // shift up from insert
    const auto end = (num_results < max_num_results) ? num_results : (max_num_results - 1);
    for (size_t i = end; i > insert; --i) {
        results[i] = results[i - 1];
    }
    results[insert] = res;

    return num_results < max_num_results ? num_results + 1 : max_num_results;
}

static bool is_collision_better(const wuzy_collision_result& a, const wuzy_collision_result& b)
{
    return a.depth > b.depth;
}

static wuzy_aabb_tree_node_query* get_mesh_collisions_begin(const Collider* a, const Collider* b)
{
    assert(is_convex(a) && !is_convex(b));
    const auto mesh = state->meshes.get(b->collider_userdata.mesh.id);
    wuzy_aabb_tree_node_query_aabb_begin(mesh->query, a->aabb_min, a->aabb_max, 0, nullptr);
    return mesh->query;
}

static bool get_mesh_collisions_next(
    wuzy_aabb_tree_node_query* query, const Collider* a, Collider* b, wuzy_collision_result* cres)
{

    wuzy_aabb_tree_node_query_result qres;
    while (wuzy_aabb_tree_node_query_next(query, &qres, 1)) {
        assert(b->collider.userdata == nullptr);
        b->collider.userdata = qres.node_userdata; // triangle collider userdata!
        const auto col = wuzy_get_collision(&a->collider, &b->collider, cres, nullptr, nullptr);
        b->collider.userdata = nullptr;
        if (col) {
            return true;
        }
    }
    return false;
}

EXPORT size_t wuzy_hl_get_collisions(wuzy_hl_collider_id id_a, wuzy_hl_collider_id id_b,
    wuzy_collision_result* results, size_t max_num_results)
{
    auto a = state->colliders.get(id_a.id);
    auto b = state->colliders.get(id_b.id);
    assert(is_convex(a) || is_convex(b));
    if (is_convex(a) && is_convex(b)) {
        return wuzy_get_collision(&a->collider, &b->collider, results, nullptr, nullptr);
    } else {
        if (!is_convex(a)) {
            std::swap(a, b);
        }

        auto query = get_mesh_collisions_begin(a, b);
        wuzy_collision_result cres;
        size_t num_results = 0;
        while (get_mesh_collisions_next(query, a, b, &cres)) {
            num_results
                = keep_best(results, num_results, max_num_results, cres, is_collision_better);
        }
        return num_results;
    }
}

static bool is_hl_collision_better(const wuzy_hl_collision& a, const wuzy_hl_collision& b)
{
    return a.res.depth > b.res.depth;
}

EXPORT size_t wuzy_hl_get_all_collisions(
    wuzy_hl_collider_id id, wuzy_hl_collision* collisions, size_t max_num_collisions)
{
    const auto col = state->colliders.get(id.id);
    assert(is_convex(col));
    wuzy_aabb_tree_node_query_aabb_begin(
        state->query, col->aabb_min, col->aabb_max, col->bitmask, nullptr);
    size_t num_collisions = 0;
    wuzy_aabb_tree_node_query_result qres;
    while (wuzy_aabb_tree_node_query_next(state->query, &qres, 1)) {
        auto other = (Collider*)qres.node_userdata;
        if (col == other) {
            continue;
        }
        const wuzy_hl_collider_id other_id = { state->colliders.get_id(other) };
        if (other->type == Collider::Type::Mesh) {
            auto query = get_mesh_collisions_begin(col, other);
            wuzy_collision_result cres;
            while (get_mesh_collisions_next(query, col, other, &cres)) {
                num_collisions = keep_best(collisions, num_collisions, max_num_collisions,
                    { other_id, cres }, is_hl_collision_better);
            }
        } else {
            wuzy_collision_result cres;
            if (wuzy_get_collision(&col->collider, &other->collider, &cres, nullptr, nullptr)) {
                num_collisions = keep_best(collisions, num_collisions, max_num_collisions,
                    { other_id, cres }, is_hl_collision_better);
            }
        }
    }
    return num_collisions;
}

template <typename T>
static void grow(T** data, size_t* size, size_t min_new_size)
{
    deallocate(&state->alloc, *data, *size);
    while (*size < min_new_size) {
        *size *= 2;
    }
    *data = allocate<T>(&state->alloc, *size);
}

EXPORT size_t wuzy_hl_collider_ray_cast(wuzy_hl_collider_id id, const float start[3],
    const float dir[3], wuzy_hl_ray_cast_result* results, size_t max_num_results)
{
    assert(max_num_results >= 1);
    auto col = state->colliders.get(id.id);
    if (col->type == Collider::Type::Mesh) {
        const auto mesh = state->meshes.get(col->collider_userdata.mesh.id);

        if (state->rc_tri_tmp_size < max_num_results) {
            grow(&state->rc_tri_tmp, &state->rc_tri_tmp_size, max_num_results);
        }

        const auto n = wuzy_aabb_tree_ray_cast_tris(
            mesh->query, start, dir, 0, state->rc_tri_tmp, max_num_results, nullptr);
        for (size_t i = 0; i < n; ++i) {
            const auto face_idx = state->rc_tri_tmp[i].tri - mesh->tris;
            results[i] = { id, (uint32_t)face_idx, state->rc_tri_tmp[i].hit };
        }
        return n;
    } else {
        results[0].collider = id;
        results[0].face_index = UINT32_MAX;
        return wuzy_collider_ray_cast(&col->collider, start, dir, &results[0].hit);
    }
}

static bool is_ray_cast_better(const wuzy_hl_ray_cast_result& a, const wuzy_hl_ray_cast_result& b)
{
    return a.hit.t < b.hit.t;
}

struct ColliderRayCastUserdata {
    const float* start;
    const float* dir;
    wuzy_hl_ray_cast_result* results;
    size_t num_results;
    size_t max_num_results;

    bool may_add(const wuzy_ray_cast_result& res)
    {
        return num_results < max_num_results || res.t < results[num_results - 1].hit.t;
    }
};

static bool collider_ray_cast_aabb_test(
    void* query_userdata, const float aabb_min[3], const float aabb_max[3])
{
    auto& rc = *(ColliderRayCastUserdata*)query_userdata;
    wuzy_ray_cast_result res;
    const auto hit = wuzy_aabb_ray_cast(aabb_min, aabb_max, rc.start, rc.dir, &res);
    return hit && rc.may_add(res);
}

static bool collider_leaf_test(void* query_userdata, void* node_userdata)
{
    auto& rc = *(ColliderRayCastUserdata*)query_userdata;
    auto col = (Collider*)node_userdata;
    const wuzy_hl_collider_id id = { state->colliders.get_id(col) };
    if (col->type == Collider::Type::Mesh) {
        auto mesh = state->meshes.get(col->collider_userdata.mesh.id);
        assert(rc.max_num_results <= state->rc_tri_tmp_size);
        const auto n = wuzy_aabb_tree_ray_cast_tris(
            mesh->query, rc.start, rc.dir, 0, state->rc_tri_tmp, rc.max_num_results, nullptr);
        for (size_t i = 0; i < n; ++i) {
            const auto face_idx = state->rc_tri_tmp[i].tri - mesh->tris;
            rc.num_results = keep_best(rc.results, rc.num_results, rc.max_num_results,
                { id, (uint32_t)face_idx, state->rc_tri_tmp[i].hit }, is_ray_cast_better);
        }
        return n > 0;
    } else {
        wuzy_ray_cast_result res;
        const auto hit = wuzy_collider_ray_cast(&col->collider, rc.start, rc.dir, &res);
        if (hit) {
            rc.num_results = keep_best(rc.results, rc.num_results, rc.max_num_results,
                { id, UINT32_MAX, res }, is_ray_cast_better);
            return true;
        }
        return false;
    }
}

EXPORT size_t wuzy_hl_ray_cast(const float start[3], const float dir[3], uint64_t bitmask,
    wuzy_hl_ray_cast_result* results, size_t max_num_results)
{
    assert(max_num_results >= 1);

    if (state->rc_tri_tmp_size < max_num_results) {
        grow(&state->rc_tri_tmp, &state->rc_tri_tmp_size, max_num_results);
    }

    ColliderRayCastUserdata crcud { start, dir, results, 0, max_num_results };
    wuzy_aabb_tree_node_query_begin(
        state->query, bitmask, &crcud, collider_ray_cast_aabb_test, collider_leaf_test, nullptr);
    wuzy_aabb_tree_node_query_next(state->query, nullptr, 0);

    return crcud.num_results;
}

EXPORT wuzy_aabb_tree* wuzy_hl_get_aabb_tree()
{
    return state->tree;
}

EXPORT wuzy_collider* wuzy_hl_get_collider(wuzy_hl_collider_id id)
{
    auto col = state->colliders.get(id.id);
    return &col->collider;
}

EXPORT wuzy_aabb_tree* wuzy_hl_mesh_get_aabb_tree(wuzy_hl_mesh_id id)
{
    auto mesh = state->meshes.get(id.id);
    return mesh->tree;
}
}