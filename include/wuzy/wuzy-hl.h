#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "wuzy.h"

// This high-level layer is much more liberal with allocation and will

typedef struct wuzy_collision_system wuzy_collision_system;

typedef struct {
    uint32_t id;
} wuzy_collider_handle;

typedef struct {
    wuzy_allocator* allocator; // default will use malloc
    size_t max_num_colliders; // no default
    size_t max_num_triangle_colliders; // default:
    size_t max_num_sphere_colliders; // default:
    size_t max_num_convex_polyhedron_colliders; // default:
    size_t max_num_vertices; // default:
    size_t max_num_faces; // default:
    size_t max_num_queries; // default: 8
} wuzy_collision_system_create_params;

wuzy_collision_system* wuzy_collision_system_create(wuzy_collision_system_create_params params);

wuzy_collider_handle wuzy_collision_system_collider_create_triangle(
    wuzy_collision_system* system, wuzy_vec3 v0, wuzy_vec3 v1, wuzy_vec3 v2);

wuzy_collider_handle wuzy_collision_system_collider_create_sphere(
    wuzy_collision_system* system, float radius);

wuzy_collider_handle wuzy_collision_system_collider_create_convex_polyhedron(
    wuzy_collision_system* system, wuzy_vec3* vertices, size_t num_vertices,
    wuzy_face_indices* face_indices, size_t num_faces);

wuzy_collider_handle wuzy_collision_system_collider_create_mesh(wuzy_collision_system* system,
    wuzy_vec3* vertices, size_t num_vertices, wuzy_face_indices* face_indices, size_t num_faces);

bool wuzy_collision_system_collider_set_userdata(
    wuzy_collision_system* system, wuzy_collider_handle collider, void* userdata);
void* wuzy_collision_system_collider_get_userdata(
    wuzy_collision_system* system, wuzy_collider_handle collider);

bool wuzy_collision_system_collider_set_bitmask(
    wuzy_collision_system* system, wuzy_collider_handle collider, uint64_t bitmask);
uint64_t wuzy_collision_system_collider_get_bitmask(
    wuzy_collision_system* system, wuzy_collider_handle collider);

bool wuzy_collision_system_collider_set_transform(
    wuzy_collision_system* system, wuzy_collider_handle collider, const wuzy_mat4* transform);

wuzy_aabb wuzy_collision_system_collider_get_aabb(
    wuzy_collision_system* system, wuzy_collider_handle collider);

wuzy_aabb wuzy_collision_system_collider_ray_cast(wuzy_collision_system* system,
    wuzy_collider_handle collider, wuzy_vec3 start, wuzy_vec3 direction,
    wuzy_ray_cast_result* result);

bool wuzy_collision_system_collider_remove(
    wuzy_collision_system* system, wuzy_collider_handle collider);

bool wuzy_collision_system_get_collision(wuzy_collision_system* system, wuzy_collider_handle a,
    wuzy_collider_handle b, wuzy_collision_result* result);

typedef struct wuzy_collision_system_query wuzy_collision_system_query;

// Queries provide an iterator interface, where you `_begin` a query and then call `_next` on it
// repeatedly until it returns 0. The begin functions return a query from a pre-allocated pool,
// which requires that queries can be marked as free to use again. You should do this using the
// `_end` function, but when `_next` returned zero, the query is automatically marked as ended and
// you don't need to call `_end`.
// If you attempt to begin a query and no finished query is available in the pool, the function will
// return null.

wuzy_collision_system_query* wuzy_collision_system_query_point_begin(
    wuzy_collision_system* system, wuzy_vec3 point, uint64_t bitmask);

wuzy_collision_system_query* wuzy_collision_system_query_aabb_begin(
    wuzy_collision_system* system, const wuzy_aabb* aabb, uint64_t bitmask);

// This is a convenience function that essentially starts an aabb query with the aabb of a collider.
wuzy_collision_system_query* wuzy_collision_system_query_candidates_begin(
    wuzy_collision_system* system, wuzy_collider_handle collider);

size_t wuzy_collision_system_query_next(
    wuzy_collision_system_query* query, wuzy_collider_handle* colliders, size_t max_colliders);

void wuzy_collision_system_query_end(wuzy_collision_system_query* query);

// Since ray casts only return a single result, this function does begin, next and end in a single
// call.
bool wuzy_collision_system_query_ray_cast(wuzy_collision_system* system, wuzy_vec3 start,
    wuzy_vec3 direction, uint64_t bitmask, wuzy_collider_handle* collider,
    wuzy_ray_cast_result* result);

// These give you access to some of the underlying low-level API objects, for whatever you might
// need to do with them.
wuzy_aabb_tree* wuzy_collision_system_get_broadphase(wuzy_collision_system* system);
wuzy_collider* wuzy_collision_system_get_collider(
    wuzy_collision_system* system, wuzy_collider_handle collider);
wuzy_aabb_tree* wuzy_collision_system_get_aabb_tree(
    wuzy_collision_system* system, wuzy_collider_handle collider);
wuzy_aabb_tree_node_query* wuzy_collision_system_get_query(wuzy_collision_system_query* query);

#ifdef __cplusplus
}
#endif
