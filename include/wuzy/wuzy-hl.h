#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "wuzy.h"

// All functions abort (assert) on invalid handles, capacity exhaustion, or null preconditions.
// Release builds still abort via a fatal path; returning values are always valid postconditions.
// This library is purely single-threaded and does no locking.

// All ids are 24 bits generation and 24 bits index, so they fit into 48 bits and will be
// exactly representable by a double.

typedef struct {
    uint64_t id; // 0 is invalid
} wuzy_hl_collider_id;

typedef struct {
    uint64_t id; // 0 is invalid
} wuzy_hl_convex_polyhedron_id;

typedef struct {
    uint64_t id; // 0 is invalid
} wuzy_hl_mesh_id;

typedef struct {
    wuzy_allocator* allocator; // default will use malloc
    size_t max_num_colliders; // no default
    size_t max_num_convex_polyhedra; // default: 16
    size_t max_num_meshes; // default: 16
} wuzy_hl_create_params;

void wuzy_hl_init(wuzy_hl_create_params params);
void wuzy_hl_shutdown(void);

// 3 floats per vertex, 3 indices per face. faces are triangles.
wuzy_hl_convex_polyhedron_id wuzy_hl_convex_polyhedron_create(
    const float* vertices, size_t num_vertices, const uint32_t* face_indices, size_t num_faces);
// Destroying a convex polyhedron referenced by a collider is UB
void wuzy_hl_convex_polyhedron_destroy(wuzy_hl_convex_polyhedron_id convex);

// This builds an optimized bounding volume hierarchy.
wuzy_hl_mesh_id wuzy_hl_mesh_create(
    const float* vertices, size_t num_vertices, const uint32_t* face_indices, size_t num_faces);
// Destroying a mesh referenced by a collider is UB
void wuzy_hl_mesh_destroy(wuzy_hl_mesh_id mesh);

wuzy_hl_collider_id wuzy_hl_collider_create_sphere(float radius);
wuzy_hl_collider_id wuzy_hl_collider_create_capsule(const float half_up[3], float radius);
wuzy_hl_collider_id wuzy_hl_collider_create_convex_polyhedron(wuzy_hl_convex_polyhedron_id convex);

// Mesh colliders do not take into account the transform
wuzy_hl_collider_id wuzy_hl_collider_create_mesh(wuzy_hl_mesh_id mesh);

void wuzy_hl_collider_destroy(wuzy_hl_collider_id collider);

void wuzy_hl_collider_set_userdata(wuzy_hl_collider_id collider, void* userdata);
void* wuzy_hl_collider_get_userdata(wuzy_hl_collider_id collider);

void wuzy_hl_collider_set_bitmask(wuzy_hl_collider_id collider, uint64_t bitmask);
uint64_t wuzy_hl_collider_get_bitmask(wuzy_hl_collider_id collider);

// This function will not work for mesh colliders!
// The matrix is column-major.
void wuzy_hl_collider_set_transform(wuzy_hl_collider_id collider, const float matrix[16]);

void wuzy_hl_collider_get_aabb(wuzy_hl_collider_id collider, float min[3], float max[3]);

size_t wuzy_hl_query_point(const float point[3], uint64_t bitmask, wuzy_hl_collider_id* colliders,
    size_t max_num_colliders);

size_t wuzy_hl_query_aabb(const float min[3], const float max[3], uint64_t bitmask,
    wuzy_hl_collider_id* colliders, size_t max_num_colliders);

// This is a convenience function that essentially starts an aabb query with the aabb of a collider.
size_t wuzy_hl_query_candidates(wuzy_hl_collider_id collider, uint64_t bitmask,
    wuzy_hl_collider_id* colliders, size_t max_num_colliders);

// For convex colliders this will return 0 or 1 collisions. If one collider is a mesh collider,
// the returned collisions can be arbitrarily many.
// The returned collisions are sorted descending by depth.
// Either a or b must be a convex collider.
size_t wuzy_hl_get_collisions(wuzy_hl_collider_id a, wuzy_hl_collider_id b,
    wuzy_collision_result* results, size_t max_num_results);

typedef struct {
    wuzy_hl_collider_id collider;
    wuzy_collision_result res;
} wuzy_hl_collision;

// This is a wrapper around wuzy_hl_query_candidates_begin and wuzy_hl_collider_get_collisions
// and returns collisions with collider and all other colliders.
// collider must be convex.
size_t wuzy_hl_get_all_collisions(
    wuzy_hl_collider_id collider, wuzy_hl_collision* collisions, size_t max_num_collisions);

typedef struct {
    float delta[3]; // desired move (world space)
    float skin; // the distance to (attempt to) maintain between other colliders
    float min_delta; // slide is terminated if movement is below this threshold
    uint64_t bitmask; // bitmask to select other colliders
} wuzy_hl_move_and_slide_params;

typedef struct {
    float moved_delta[3];
    float remaining_delta[3];
    bool hit;
    float last_hit_normal[3];
    wuzy_hl_collider_id last_hit_collider;
    uint32_t last_hit_face_index;
} wuzy_hl_move_and_slide_result;

// Kinematic sweep & slide for a convex collider (`moving`).
// Moves `moving` by up to `delta` in world space.
// This updates the collider, so use `moved_delta`/`remaining_delta` to sync external state.
// Pass a zero-initialized `out`.
void wuzy_hl_move_and_slide(wuzy_hl_collider_id moving, wuzy_hl_move_and_slide_params params,
    wuzy_hl_move_and_slide_result* out);

typedef struct {
    wuzy_hl_collider_id collider;
    uint32_t face_index; // set iff mesh collider
    wuzy_ray_cast_result hit;
} wuzy_hl_ray_cast_result;

// dir need not be normalized; hit is at start + t*dir with t >= 0, t is not limited.
// Only hits front faces. The results are sorted ascending by t.
// For convex colliders this will return 0 or 1 hits. If the collider is a mesh collider, the number
// of results can be in [0, max_num_results].
size_t wuzy_hl_collider_ray_cast(wuzy_hl_collider_id collider, const float start[3],
    const float dir[3], wuzy_hl_ray_cast_result* results, size_t max_num_results);

size_t wuzy_hl_ray_cast(const float start[3], const float dir[3], uint64_t bitmask,
    wuzy_hl_ray_cast_result* results, size_t max_num_results);

// These give you access to some of the underlying low-level API objects, in case you need them.
// Mutating objects returned by the following functions may break invariants of the high-level API,
// so your usage of them might break at any time.
wuzy_aabb_tree* wuzy_hl_get_aabb_tree();
wuzy_collider* wuzy_hl_get_collider(wuzy_hl_collider_id collider);
wuzy_aabb_tree* wuzy_hl_mesh_get_aabb_tree(wuzy_hl_mesh_id mesh);

#ifdef __cplusplus
}
#endif
