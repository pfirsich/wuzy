#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* General
## Allocations
No function here allocates, unless explicitly stated or unless it takes a wuzy_allocator argument.
This is by design, so you can manage objects on your own and ideally you only have to
allocate at initialization time.
You may always pass nullptr as a wuzy_allocator, in which case malloc will be used.

## Thread Safety
If a function takes a non-const pointer, it might mutate the referenced object. There is no locking
or synchronization happening in this library, so you have to take care to not mutate objects from
multiple threads simultaneously. This is almost entirely only relevant for aabb_tree, which you
should lock externally, if you need parallel access.
 */

// Basic Types

// float[3] should be considered a vector with components x, y, z
// float[16] should be considered 4x4 matrices in column-major order

// https://nullprogram.com/blog/2023/12/17/
typedef void* (*wuzy_allocator_allocate)(size_t size, void* ctx);
typedef void* (*wuzy_allocator_reallocate)(void* ptr, size_t old_size, size_t new_size, void* ctx);
typedef void (*wuzy_allocator_deallocate)(void* ptr, size_t size, void* ctx);

typedef struct {
    wuzy_allocator_allocate allocate;
    wuzy_allocator_reallocate reallocate;
    wuzy_allocator_deallocate deallocate;
    void* ctx;
} wuzy_allocator;

void wuzy_invert_trs(const float mat[16], float inv[16]);

// Collision Detection

typedef struct {
    float normal[3];
    float hit_position[3];
    float t;
} wuzy_ray_cast_result;

typedef struct {
    // Make sure to initialize these with identity matrices!
    float transform[16];
    float inverse_transform[16];
    // This userdata can be used for additional collider shape information for support_func and
    // ray_cast_func. For a few pre-provided shapes, see below.
    // This userdata is not owned by collider, so make sure it lives as long as you want to use
    // wuzy_collider.
    void* userdata;
    // Support_func and ray_cast_func take vectors in local space and return vectors in local space.
    // So do the functions wuzy_*_collider_support_func/ray_cast.
    void (*support_func)(const void* userdata, const float dir[3], float sup[3]);
    // Will return true if the ray hit and fill in result, otherwise just returns false
    bool (*ray_cast_func)(const void* userdata, const float start[3], const float dir[3],
        wuzy_ray_cast_result* result);
} wuzy_collider;

void wuzy_collider_set_transform(wuzy_collider* collider, const float transform[16]);
// These take into account the collider transform, so they take vectors in world space
void wuzy_collider_support(const wuzy_collider* collider, const float dir[3], float sup[3]);
bool wuzy_collider_ray_cast(const wuzy_collider* collider, const float start[3], const float dir[3],
    wuzy_ray_cast_result* result);
void wuzy_collider_get_aabb(const wuzy_collider* collider, float min[3], float max[3]);

// Helper function for triangle and convex polyhedron colliders.
// There should be three floats for every vertex and three indices for every face.
// There should be just as many normals as there are faces and three floats per normal.
bool wuzy_calculate_normals(const float* vertices, size_t num_vertices,
    const uint32_t* face_indices, size_t num_faces, float* normals);

typedef struct {
    float vertices[3][3];
    // normal must be: cross(v[1] - v[0], v[2] - v[0]) (not normalized!)
    float normal[3];
} wuzy_triangle_collider_userdata;

// This calculates the normal from vertices, initializes transform and inverse_transform with
// identity matrices and sets collider->support_func, collider->ray_cast_func and
// collider->userdata.
void wuzy_triangle_collider_init(
    wuzy_collider* collider, wuzy_triangle_collider_userdata* userdata);
void wuzy_triangle_collider_support(const void* userdata, const float dir[3], float sup[3]);
bool wuzy_triangle_collider_ray_cast(
    const void* userdata, const float start[3], const float dir[3], wuzy_ray_cast_result* result);

typedef struct {
    float radius;
} wuzy_sphere_collider_userdata;

// This initializes the collider fields like the wuzy_triangle_collider_init.
void wuzy_sphere_collider_init(wuzy_collider* collider, wuzy_sphere_collider_userdata* userdata);
void wuzy_sphere_collider_support(const void* userdata, const float dir[3], float sup[3]);
bool wuzy_sphere_collider_ray_cast(
    const void* userdata, const float start[3], const float dir[3], wuzy_ray_cast_result* result);

typedef struct {
    float* vertices;
    size_t num_vertices;
    uint32_t* face_indices;
    size_t num_faces;
    float* normals; // one normal per face
} wuzy_convex_polyhedron_collider_userdata;

// This calculates the normals (make sure userdata->normals points to suitably sized memory) and
// inits the fields in collider like wuzy_triangle_collider_init.
void wuzy_convex_polyhedron_collider_init(
    wuzy_collider* collider, wuzy_convex_polyhedron_collider_userdata* userdata);
void wuzy_convex_polyhedron_collider_support(
    const void* userdata, const float dir[3], float sup[3]);
bool wuzy_convex_polyhedron_collider_ray_cast(
    const void* userdata, const float start[3], const float dir[3], wuzy_ray_cast_result* result);

// TODO: Capsule!

typedef struct {
    float vertices[4][3];
    size_t num_vertices;
} wuzy_simplex3d;

typedef struct wuzy_gjk_debug wuzy_gjk_debug;

// debug may be null of course. To get debug info, just pass non-null pointing to a null-ed
// wuzy_gjk_debug.
bool wuzy_gjk(
    const wuzy_collider* a, const wuzy_collider* b, wuzy_simplex3d* result, wuzy_gjk_debug* debug);

typedef struct wuzy_epa_debug wuzy_epa_debug;

// TODO: Lower-Level return for EPA
typedef struct {
    float normal[3];
    float depth;
    // TODO: contact points
} wuzy_collision_result;

wuzy_collision_result wuzy_epa(const wuzy_collider* a, const wuzy_collider* b,
    const wuzy_simplex3d* simplex, wuzy_epa_debug* debug);

// High Level Collision Detection

bool wuzy_test_collision(const wuzy_collider* a, const wuzy_collider* b, wuzy_gjk_debug* debug);
bool wuzy_get_collision(const wuzy_collider* a, const wuzy_collider* b,
    wuzy_collision_result* result, wuzy_gjk_debug* gjk_debug, wuzy_epa_debug* epa_debug);

// AABB Tree

typedef struct {
    // These ids are u64s, but they will not exceed 48 bits (16 bit generation, 32 bit index), so
    // they can fit into double precision floats, particularly to allow easy integration with Lua.
    // 0 represents an invalid node.
    uint64_t id;
} wuzy_aabb_tree_node;

typedef struct wuzy_aabb_tree wuzy_aabb_tree;

// Will return null if allocation fails. alloc may be null, in which case it will use malloc.
// The colliders being inserted into the tree are leaf-nodes, but the tree will have internal
// (non-leaf) nodes too, so that the total number of nodes allocated is larger.
// Currently the largest max_num_colliders that can be passed to this function is 32757.
wuzy_aabb_tree* wuzy_aabb_tree_create(size_t max_num_colliders, wuzy_allocator* alloc);

void wuzy_aabb_tree_destroy(wuzy_aabb_tree* tree);

/* The bitmask can be used to assign groups to colliders.
   Query and ray cast functions will only return nodes that share bits with the passed bit mask,
   i.e. `(a & b) > 0`.
   If 0 is passed for the bitmask to the query of ray cast functions, there will be no filtering
   done based on the bitmask, i.e. it is as if 0xffff'ffff'ffff'ffff was passed.
 */

typedef struct wuzy_aabb_tree_init_node wuzy_aabb_tree_init_node;

struct wuzy_aabb_tree_init_node {
    wuzy_aabb_tree_node id; // Will be set by wuzy_aabb_tree_init
    // for internal nodes, left and right must be non-null and collider must be null
    // for leaf nodes, left and right must be null and collider must be non-null
    wuzy_collider* collider;
    uint64_t bitmask; // bitmask is ignored for internal nodes
    wuzy_aabb_tree_init_node* left;
    wuzy_aabb_tree_init_node* right;
};

// Building these trees is not free, so you might want to dump then at asset compile time
// (wuzy_aabb_tree_dump) and then load it back at load time using this function.
// This function asserts that the tree is empty.
// You may also build even more optimized trees offline and load them in here.
bool wuzy_aabb_tree_init(wuzy_aabb_tree* tree, wuzy_aabb_tree_init_node* root_node);

// Instead of inserting a bunch of colliders and then rebuilding to improve the tree, you can use
// this function to build a new tree from many colliders.
// This function asserts that the tree is empty.
// `bitmasks` may be nullptr in which case all bitmasks will be -1.
void wuzy_aabb_tree_build(wuzy_aabb_tree* tree, wuzy_collider* const* colliders,
    const uint64_t* bitmasks, size_t num_colliders, wuzy_aabb_tree_node* nodes);

// This is very expensive (O(n^3)!), but should yield a near-optimal tree.
// You likely want to do this offline and then use wuzy_aabb_tree_dump_tree and wuzy_aabb_tree_init.
// Note that simply inserting repeatedly is actually not that bad and yields fairly reasonable
// trees.
// Also this function does a bunch of allocations (via std::vector), other than any other function.
void wuzy_aabb_tree_rebuild(wuzy_aabb_tree* tree);

// If 0 is passed for the bitmask, it is set to 0xffff'ffff'ffff'ffff.
// Will return invalid node (id = 0) if the node could not be inserted (maximum number of nodes
// reached).
wuzy_aabb_tree_node wuzy_aabb_tree_insert(
    wuzy_aabb_tree* tree, wuzy_collider* collider, uint64_t bitmask);

wuzy_collider* wuzy_aabb_tree_get_collider(wuzy_aabb_tree* tree, wuzy_aabb_tree_node node);

typedef enum {
    // The default is REINSERT.
    WUZY_AABB_TREE_UPDATE_FLAGS_DEFAULT = 0,
    // REINSERT will remove the node and then find a new, good place for it.
    WUZY_AABB_TREE_UPDATE_FLAGS_REINSERT,
    // REFIT will simply update the nodes AABB/bitmask and its parents. This is much quicker than
    // reinsert, but yields a much worse tree.
    // This can be used to implement something like enlarged AABBs.
    WUZY_AABB_TREE_UPDATE_FLAGS_REFIT,
} wuzy_aabb_tree_update_mode;

// This will compute the aabb of collider associated with this node again and update the tree
// accordingly. Will return true if the given node exists (in which case it will be update) and
// false if not. If 0 is passed for the bitmask, it is not changed.
bool wuzy_aabb_tree_update(wuzy_aabb_tree* tree, wuzy_aabb_tree_node node, uint64_t bitmask,
    wuzy_aabb_tree_update_mode mode);

// Will return whether the given node existed before removal.
bool wuzy_aabb_tree_remove(wuzy_aabb_tree* tree, wuzy_aabb_tree_node node);

typedef struct wuzy_aabb_tree_dump_node wuzy_aabb_tree_dump_node;

struct wuzy_aabb_tree_dump_node {
    wuzy_aabb_tree_node id;
    const wuzy_collider* collider; // will be null for internal nodes
    float aabb_min[3];
    float aabb_max[3];
    uint64_t bitmask;
    wuzy_aabb_tree_dump_node* parent;
    wuzy_aabb_tree_dump_node* left;
    wuzy_aabb_tree_dump_node* right;
};

// Returns number of nodes
size_t wuzy_aabb_tree_dump(
    const wuzy_aabb_tree* tree, wuzy_aabb_tree_dump_node* nodes, size_t max_num_nodes);

typedef struct {
    size_t num_nodes;
    size_t num_colliders;
    size_t max_num_nodes;
} wuzy_aabb_tree_stats;

void wuzy_aabb_tree_get_stats(const wuzy_aabb_tree* tree, wuzy_aabb_tree_stats* stats);

typedef struct wuzy_aabb_tree_node_query wuzy_aabb_tree_node_query;

// A query requires some storage that is likely too large for the stack and to avoid
// unexpected (and repeated) dynamic allocations, you can create a query object beforehand. It can
// be reused, but you can only use it for a single query at a time. If the tree itself is not
// modified, you can also perform multiple queries in parallel, as long as you don't use the same
// query object simultaneously from multiple threads.
wuzy_aabb_tree_node_query* wuzy_aabb_tree_node_query_create(
    const wuzy_aabb_tree* tree, wuzy_allocator* alloc);

void wuzy_aabb_tree_node_query_destroy(wuzy_aabb_tree_node_query* query);

// These are supposed to implement iterators. You start a query with _begin and then call _next
// repeatedly (even with max_nodes = 1) until it returns less than max_nodes.
// It's perfectly fine to _begin a query that's not finished yet.
// The value returned is the number of results filled into the out parameter.

// There was a generic begin function which took a aabb_test and a collider_test callback which
// allowed implementing a point, aabb and ray cast query yourself, but it introduces overhead in the
// common operations and I don't even think there is much of a use for it, so it's gone now.

typedef struct wuzy_query_debug wuzy_query_debug;

void wuzy_aabb_tree_node_query_point_begin(wuzy_aabb_tree_node_query* query, const float point[3],
    uint64_t bitmask, wuzy_query_debug* debug);

void wuzy_aabb_tree_node_query_aabb_begin(wuzy_aabb_tree_node_query* query, const float aabb_min[3],
    const float aabb_max[3], uint64_t bitmask, wuzy_query_debug* debug);

size_t wuzy_aabb_tree_node_query_next(
    wuzy_aabb_tree_node_query* query, wuzy_aabb_tree_node* nodes, size_t max_nodes);

// Since a ray cast only has a single result, this function doesn't require a separate _begin and
// _next.

bool wuzy_aabb_tree_node_query_ray_cast(wuzy_aabb_tree_node_query* query, const float start[3],
    const float dir[3], uint64_t bitmask, wuzy_aabb_tree_node* hit_node,
    wuzy_ray_cast_result* result, wuzy_query_debug* debug);

/*
typedef struct {
    wuzy_aabb_tree_node a;
    wuzy_aabb_tree_node b;
} wuzy_aabb_tree_node_pair;

typedef struct wuzy_aabb_tree_pair_query wuzy_aabb_tree_pair_query;

wuzy_aabb_tree_pair_query* wuzy_aabb_tree_pair_query_create(
    const wuzy_aabb_tree* tree, wuzy_allocator* alloc);

void wuzy_aabb_tree_pair_query_destroy(wuzy_aabb_tree_pair_query* query);

void wuzy_aabb_tree_pair_query_begin(wuzy_aabb_tree_pair_query* query);

size_t wuzy_aabb_tree_pair_query_next(
    wuzy_aabb_tree_pair_query* query, wuzy_aabb_tree_node_pair* pairs, size_t max_pairs);

size_t wuzy_aabb_tree_pair_query_all(wuzy_aabb_tree_pair_query* query,
    void (*callback)(void*, wuzy_aabb_tree_node, wuzy_aabb_tree_node), void* userdata);
*/

// Debug Types
// All pointers in these types will be filled with dynamically allocated memory. I could size them
// sufficiently and have them be completely on the stack, but some of these structures would get so
// large that you should likely start allocating them on the heap. Also the size is dependent on the
// iteration count, which I consider too much of an implementation detail. The real reason I did
// this though is because they were fixed size arrays before, but they had to be so large, that it
// was extremely annoying to deal with in gdb and considering they are for debugging, it defeated
// the purpose. Make sure to free them with the provided functions!

typedef struct {
    float direction[3];
    float a_support[3];
    float b_support[3];
    float support[3];
    wuzy_simplex3d simplex;
    bool contains_origin;
} wuzy_gjk_debug_iteration;

struct wuzy_gjk_debug {
    wuzy_allocator* alloc; // default allocator (malloc) will be used if null
    wuzy_gjk_debug_iteration* iterations;
    size_t num_iterations;
    size_t max_num_iterations; // If you set this to non-zero it will limit the number of iterations
};

void wuzy_gjk_debug_free(wuzy_gjk_debug* debug);

typedef struct {
    size_t from;
    size_t to;
} wuzy_epa_debug_edge;

typedef struct {
    size_t v0;
    size_t v1;
    size_t v2;
    float normal[3];
    float dist; // distance to origin
} wuzy_epa_debug_face;

typedef struct {
    float* polytope_vertices; // 3 floats per vertex
    size_t num_polytope_vertices;
    wuzy_epa_debug_face* polytope_faces;
    size_t num_polytope_faces;
    bool* face_removed; // size is num_polytope_faces
    size_t min_dist_face_index;
    float min_face_dist;
    float support_point[3];
    float support_dist;
    wuzy_epa_debug_edge* edges_to_patch;
    size_t num_edges_to_patch;
} wuzy_epa_debug_iteration;

struct wuzy_epa_debug {
    wuzy_allocator* alloc; // default allocator (malloc) will be used if null
    wuzy_epa_debug_iteration* iterations;
    size_t num_iterations;
    size_t max_num_iterations; // If you set this to non-zero it will limit the number of iterations
};

void wuzy_epa_debug_free(wuzy_epa_debug* debug);

struct wuzy_query_debug {
    size_t nodes_checked;
    size_t bitmask_checks_passed;
    size_t aabb_checks_passed;
    size_t leaves_checked;
    size_t full_checks_passed;
};

#ifdef __cplusplus
}
#endif
