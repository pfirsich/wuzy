#include "wuzy/wuzy.h"

#include <array>
#include <cassert>
#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <span>
#include <tuple>

#define EXPORT extern "C"

namespace {
wuzy_allocator* default_allocator()
{
    static wuzy_allocator alloc {
        .allocate = [](size_t size, void*) -> void* { return std::malloc(size); },
        .reallocate = [](void* ptr, size_t, size_t new_size, void*) -> void* {
            return std::realloc(ptr, new_size);
        },
        .deallocate = [](void* ptr, size_t, void*) { std::free(ptr); },
        .ctx = nullptr,
    };
    return &alloc;
}

template <typename T>
T* allocate(wuzy_allocator* alloc, size_t count = 1)
{
    alloc = alloc ? alloc : default_allocator();
    // NOLINTNEXTLINE(bugprone-sizeof-expression)
    auto ptr = reinterpret_cast<T*>(alloc->allocate(sizeof(T) * count, alloc->ctx));
    for (size_t i = 0; i < count; ++i) {
        new (ptr + i) T {};
    }
    return ptr;
}

template <typename T>
T* reallocate(wuzy_allocator* alloc, T* ptr, size_t old_count, size_t new_count)
{
    static_assert(std::is_trivially_copyable_v<T>);
    alloc = alloc ? alloc : default_allocator();
    ptr = reinterpret_cast<T*>(
        alloc->reallocate(ptr, sizeof(T) * old_count, sizeof(T) * new_count, alloc->ctx));
    return ptr;
}

template <typename T>
void deallocate(wuzy_allocator* alloc, T* ptr, size_t count = 1)
{
    alloc = alloc ? alloc : default_allocator();
    for (size_t i = 0; i < count; ++i) {
        (ptr + i)->~T();
    }
    // NOLINTNEXTLINE(bugprone-sizeof-expression)
    return alloc->deallocate(ptr, sizeof(T) * count, alloc->ctx);
}

// wuzy_vec3/wuzy_vec4

wuzy_vec3 add(const wuzy_vec3& a, const wuzy_vec3& b)
{
    return wuzy_vec3 { a.x + b.x, a.y + b.y, a.z + b.z };
}

wuzy_vec3 sub(const wuzy_vec3& a, const wuzy_vec3& b)
{
    return wuzy_vec3 { a.x - b.x, a.y - b.y, a.z - b.z };
}

float dot(const wuzy_vec3& a, const wuzy_vec3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

float dot(const wuzy_vec4& a, const wuzy_vec4& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

wuzy_vec3 cross(const wuzy_vec3& a, const wuzy_vec3& b)
{
    return wuzy_vec3 {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

float length(const wuzy_vec3& v)
{
    return std::sqrt(dot(v, v));
}

float length(const wuzy_vec4& v)
{
    return std::sqrt(dot(v, v));
}

wuzy_vec3 mul(const wuzy_vec3& v, float s)
{
    return wuzy_vec3 { v.x * s, v.y * s, v.z * s };
}

wuzy_vec4 mul(const wuzy_vec4& v, float s)
{
    return wuzy_vec4 { v.x * s, v.y * s, v.z * s, v.w * s };
}

wuzy_vec3 normalize(const wuzy_vec3& v)
{
    const auto len = length(v);
    return wuzy_vec3 { v.x / len, v.y / len, v.z / len };
}

[[maybe_unused]] bool is_finite(const wuzy_vec3& v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

// wuzy_mat4

wuzy_mat4 transpose(const wuzy_mat4& m)
{
    const auto& c = m.cols;
    return {
        wuzy_vec4 { c[0].x, c[1].x, c[2].x, c[3].x },
        wuzy_vec4 { c[0].y, c[1].y, c[2].y, c[3].y },
        wuzy_vec4 { c[0].z, c[1].z, c[2].z, c[3].z },
        wuzy_vec4 { c[0].w, c[1].w, c[2].w, c[3].w },
    };
}

wuzy_mat4 mul(const wuzy_mat4& a, const wuzy_mat4& b)
{
    const auto tr = transpose(a);
    const auto& r = tr.cols; // a rows
    const auto c = b.cols; // b columns
    return wuzy_mat4 {
        wuzy_vec4 { dot(r[0], c[0]), dot(r[1], c[0]), dot(r[2], c[0]), dot(r[3], c[0]) },
        wuzy_vec4 { dot(r[0], c[1]), dot(r[1], c[1]), dot(r[2], c[1]), dot(r[3], c[1]) },
        wuzy_vec4 { dot(r[0], c[2]), dot(r[1], c[2]), dot(r[2], c[2]), dot(r[3], c[2]) },
        wuzy_vec4 { dot(r[0], c[3]), dot(r[1], c[3]), dot(r[2], c[3]), dot(r[3], c[3]) },
    };
}

wuzy_vec3 mul(const wuzy_mat4& m, const wuzy_vec3& v, float w)
{
    const auto tr = transpose(m);
    const auto& rows = tr.cols;
    const auto vec = wuzy_vec4 { v.x, v.y, v.z, w };
    return wuzy_vec3 { dot(rows[0], vec), dot(rows[1], vec), dot(rows[2], vec) };
}

wuzy_mat4 identity()
{
    return {
        wuzy_vec4 { 1.0f, 0.0f, 0.0f, 0.0f },
        wuzy_vec4 { 0.0f, 1.0f, 0.0f, 0.0f },
        wuzy_vec4 { 0.0f, 0.0f, 1.0f, 0.0f },
        wuzy_vec4 { 0.0f, 0.0f, 0.0f, 1.0f },
    };
}

// wuzy_aabb

bool overlap(const wuzy_aabb& a, const wuzy_aabb& b)
{
    return a.min.x <= b.max.x && a.min.y <= b.max.y && a.min.z <= b.max.z && a.max.x >= b.min.x
        && a.max.y >= b.min.y && a.max.z >= b.min.z;
}

bool contains(const wuzy_aabb& aabb, const wuzy_vec3& p)
{
    return p.x >= aabb.min.x && p.y >= aabb.min.y && p.z >= aabb.min.z && p.x <= aabb.max.x
        && p.y <= aabb.max.y && p.z <= aabb.max.z;
}

wuzy_aabb combine(const wuzy_aabb& a, const wuzy_aabb& b)
{
    return wuzy_aabb {
        wuzy_vec3 {
            std::min(a.min.x, b.min.x),
            std::min(a.min.y, b.min.y),
            std::min(a.min.z, b.min.z),
        },
        wuzy_vec3 {
            std::max(a.max.x, b.max.x),
            std::max(a.max.y, b.max.y),
            std::max(a.max.z, b.max.z),
        },
    };
}

[[maybe_unused]] float volume(const wuzy_aabb& aabb)
{
    const auto s = sub(aabb.max, aabb.min);
    return s.x * s.y * s.z;
}

bool ray_cast(const wuzy_aabb& aabb, const wuzy_vec3& start, const wuzy_vec3& direction,
    wuzy_ray_cast_result* res)
{
    // Real-Time Collision Detection, 5.3.3
    // This could be made much faster if we extend the ray data:
    // https://knork.org/fast-AABB-test.html
    static const wuzy_vec3 axes[3] {
        wuzy_vec3 { 1.0f, 0.0f, 0.0f },
        wuzy_vec3 { 0.0f, 1.0f, 0.0f },
        wuzy_vec3 { 0.0f, 0.0f, 1.0f },
    };
    const float ood[3] = { 1.0f / direction.x, 1.0f / direction.y, 1.0f / direction.z };
    const float* min = &aabb.min.x;
    const float* max = &aabb.max.x;
    const float* dir = &direction.x;
    const float* pos = &start.x;

    float tmin = 0.0f;
    float tmax = FLT_MAX;
    for (size_t axis = 0; axis < 3; ++axis) {
        if (std::abs(dir[axis]) < FLT_EPSILON) {
            // Ray is parallel to slab. No hit if origin not within slab.
            if (pos[axis] < min[axis] || pos[axis] > max[axis]) {
                return false;
            }
        } else {
            // Compute intersection t value of ray with near and far plane of slab
            float normal_sign = 1.0f;
            float t1 = (min[axis] - pos[axis]) * ood[axis];
            float t2 = (max[axis] - pos[axis]) * ood[axis];
            // Make t1 be intersection with near plane t2 with far plane
            if (t1 > t2) {
                std::swap(t1, t2);
                normal_sign = -1.0f;
            }
            if (t1 > tmin) {
                tmin = t1;
                res->normal = mul(axes[axis], normal_sign);
            }
            if (t2 < tmax) {
                tmax = t2;
            }
            // Exit with no collision as soon as slab intersection becomes empty
            if (tmin > tmax) {
                return false;
            }
        }
    }
    res->t = tmin;
    res->hit_position = add(start, mul(direction, res->t));
    return true;
}

template <typename T>
class VecAdapter {
public:
    VecAdapter(std::span<T> data) : data_(data) { }

    T& operator[](size_t idx)
    {
        assert(idx < size_);
        return data_[idx];
    }

    const T& operator[](size_t idx) const
    {
        assert(idx < size_);
        return data_[idx];
    }

    void push_back(T v)
    {
        assert(size_ < data_.size());
        data_[size_] = std::move(v);
        size_++;
    }

    void pop_and_swap(size_t idx)
    {
        assert(idx < size_);
        data_[idx] = data_[size_ - 1];
        size_--;
    }

    void clear() { size_ = 0; }

    size_t find(const T& v) const
    {
        for (size_t i = 0; i < size_; ++i) {
            if (data_[i] == v) {
                return i;
            }
        }
        return static_cast<size_t>(-1);
    }

    // These are required to allow conversion to std::span
    T* begin() { return data_.data(); }
    const T* begin() const { return data_.data(); }
    T* end() { return data_.data() + size_; }
    const T* end() const { return data_.data() + size_; }
    T* data() { return data_.data(); }
    const T* data() const { return data_.data(); }
    size_t size() const { return size_; }

private:
    std::span<T> data_;
    size_t size_ = 0;
};
}

EXPORT wuzy_mat4 wuzy_invert_trs(const wuzy_mat4* m)
{
    // inv(TRS) = inv(S) * inv(R) * inv(T) = inv(S) * transpose(R) * inv(T)
    const auto sx = length(m->cols[0]);
    const auto sy = length(m->cols[1]);
    const auto sz = length(m->cols[2]);

    const auto r = wuzy_mat4 {
        mul(m->cols[0], 1.0f / sx),
        mul(m->cols[1], 1.0f / sy),
        mul(m->cols[2], 1.0f / sz),
        wuzy_vec4 { 0.0f, 0.0f, 0.0f, 1.0f },
    };

    auto s = identity();
    s.cols[0].x = 1.0f / sx;
    s.cols[1].y = 1.0f / sy;
    s.cols[2].z = 1.0f / sz;

    auto t = identity();
    t.cols[3] = mul(m->cols[3], -1.0f);

    return mul(mul(s, transpose(r)), t);
}

EXPORT void wuzy_collider_set_transform(wuzy_collider* collider, const wuzy_mat4* transform)
{
    collider->transform = *transform;
    collider->inverse_transform = wuzy_invert_trs(transform);
}

EXPORT wuzy_vec3 wuzy_collider_support(const wuzy_collider* collider, wuzy_vec3 direction)
{
    const auto local_dir = mul(collider->inverse_transform, direction, 0.0f);
    const auto local_sup = collider->support_func(collider->userdata, local_dir);
    return mul(collider->transform, local_sup, 1.0f);
}

EXPORT bool wuzy_collider_ray_cast(const wuzy_collider* collider, wuzy_vec3 start,
    wuzy_vec3 direction, wuzy_ray_cast_result* result)
{
    const auto start_local = mul(collider->inverse_transform, start, 1.0f);
    const auto dir_local = mul(collider->inverse_transform, direction, 0.0f);
    const auto res = collider->ray_cast_func(collider->userdata, start_local, dir_local, result);
    result->normal = mul(collider->transform, result->normal, 0.0f);
    result->hit_position = mul(collider->transform, result->hit_position, 1.0f);
    return res;
}

EXPORT wuzy_aabb wuzy_collider_get_aabb(const wuzy_collider* collider)
{
    // http://allenchou.net/2014/02/game-physics-updating-aabbs-for-polyhedrons/
    wuzy_aabb aabb;
    // TODO: Check whether this is optimized and if not, do it myself.
    // By passing base vectors, we are essentially using columns of the inverse transform for the
    // local direction.
    aabb.min.x = wuzy_collider_support(collider, wuzy_vec3 { -1.0f, 0.0f, 0.0f }).x;
    aabb.min.y = wuzy_collider_support(collider, wuzy_vec3 { 0.0f, -1.0f, 0.0f }).y;
    aabb.min.z = wuzy_collider_support(collider, wuzy_vec3 { 0.0f, 0.0f, -1.0f }).z;
    aabb.max.x = wuzy_collider_support(collider, wuzy_vec3 { 1.0f, 0.0f, 0.0f }).x;
    aabb.max.y = wuzy_collider_support(collider, wuzy_vec3 { 0.0f, 1.0f, 0.0f }).y;
    aabb.max.z = wuzy_collider_support(collider, wuzy_vec3 { 0.0f, 0.0f, 1.0f }).z;
    return aabb;
}

EXPORT bool wuzy_calculate_normals(const wuzy_vec3* vertices, size_t num_vertices,
    const wuzy_face_indices* face_indices, size_t num_faces, wuzy_vec3* normals)
{
    for (size_t f = 0; f < num_faces; ++f) {
        const auto& i = face_indices[f];
        if (i.i0 >= num_vertices || i.i1 >= num_vertices || i.i2 >= num_vertices) {
            return false;
        }
        const auto& v0 = vertices[i.i0];
        const auto& v1 = vertices[i.i1];
        const auto& v2 = vertices[i.i2];
        normals[f] = normalize(cross(sub(v1, v0), sub(v2, v0)));
    }
    return true;
}

EXPORT void wuzy_triangle_collider_init(
    wuzy_collider* collider, wuzy_triangle_collider_userdata* userdata)
{
    assert(is_finite(userdata->vertices[0]));
    assert(is_finite(userdata->vertices[1]));
    assert(is_finite(userdata->vertices[2]));
    userdata->normal = cross(sub(userdata->vertices[1], userdata->vertices[0]),
        sub(userdata->vertices[2], userdata->vertices[0]));
    collider->transform = identity();
    collider->inverse_transform = identity();
    collider->userdata = userdata;
    collider->support_func = wuzy_triangle_collider_support;
    collider->ray_cast_func = wuzy_triangle_collider_ray_cast;
}

EXPORT wuzy_vec3 wuzy_triangle_collider_support(const void* userdata, wuzy_vec3 direction)
{
    const auto& tri = *reinterpret_cast<const wuzy_triangle_collider_userdata*>(userdata);
    wuzy_vec3 max_vtx;
    float max_dot = -FLT_MAX;
    for (size_t i = 0; i < 3; ++i) {
        const auto d = dot(tri.vertices[i], direction);
        if (d > max_dot) {
            max_dot = d;
            max_vtx = tri.vertices[i];
        }
    }
    return max_vtx;
}

EXPORT bool wuzy_triangle_collider_ray_cast(
    const void* userdata, wuzy_vec3 start, wuzy_vec3 direction, wuzy_ray_cast_result* result)
{
    // Real-Time Collision Detection, 5.3.6
    // This is essentially Möller-Trumbore, but with using the scalar triple product identify a few
    // times and pre-calculating the normal.
    const auto& tri = *reinterpret_cast<const wuzy_triangle_collider_userdata*>(userdata);

    // Check if ray points in direction of triangle normal (or is coplanar)
    const auto d = -dot(tri.normal, direction);
    if (d <= FLT_EPSILON) {
        return false;
    }

    // Check which half space the ray origin is in
    const auto s = sub(start, tri.vertices[0]);
    const auto t = dot(tri.normal, s);
    if (t < 0.0f) {
        return false;
    }

    // Compute barycentric coordinates of intersection
    const auto e = cross(direction, s);
    const auto v = -dot(sub(tri.vertices[2], tri.vertices[0]), e);
    if (v < 0.0f || v > d) {
        return false;
    }
    const auto w = dot(sub(tri.vertices[1], tri.vertices[0]), e);
    if (w < 0.0f || v + w > d) {
        return false;
    }

    result->t = t / d;
    result->normal = normalize(tri.normal); // tri.normal is not normalized!
    result->hit_position = add(start, mul(direction, result->t));
    return true;
}

EXPORT void wuzy_sphere_collider_init(
    wuzy_collider* collider, wuzy_sphere_collider_userdata* userdata)
{
    collider->transform = identity();
    collider->inverse_transform = identity();
    collider->userdata = userdata;
    collider->support_func = wuzy_sphere_collider_support;
    collider->ray_cast_func = wuzy_sphere_collider_ray_cast;
}

EXPORT wuzy_vec3 wuzy_sphere_collider_support(const void* userdata, wuzy_vec3 direction)
{
    const auto& sphere = *reinterpret_cast<const wuzy_sphere_collider_userdata*>(userdata);
    return mul(normalize(direction), sphere.radius);
}

EXPORT bool wuzy_sphere_collider_ray_cast(
    const void* userdata, wuzy_vec3 start, wuzy_vec3 direction, wuzy_ray_cast_result* result)
{
    const auto& sphere = *reinterpret_cast<const wuzy_sphere_collider_userdata*>(userdata);

    // Real-Time Collision Detection, 5.3.2
    const auto b = dot(start, direction);
    const auto c = dot(start, start) - sphere.radius * sphere.radius;
    // ray starts outside sphere and points away from it or ray starts inside the sphere
    if ((c > 0.0f && b > 0.0f) || c < 0.0f) {
        return false;
    }

    const auto discr = b * b - c;
    if (discr < 0.0f) {
        return false;
    }

    result->t = std::max(0.0f, -b - std::sqrt(discr));
    result->hit_position = add(start, mul(direction, result->t));
    result->normal = normalize(result->hit_position);
    return true;
}

EXPORT void wuzy_convex_polyhedron_collider_init(
    wuzy_collider* collider, wuzy_convex_polyhedron_collider_userdata* userdata)
{
    wuzy_calculate_normals(userdata->vertices, userdata->num_vertices, userdata->face_indices,
        userdata->num_faces, userdata->normals);
    collider->transform = identity();
    collider->inverse_transform = identity();
    collider->userdata = userdata;
    collider->support_func = wuzy_convex_polyhedron_collider_support;
    collider->ray_cast_func = wuzy_convex_polyhedron_collider_ray_cast;
}

EXPORT wuzy_vec3 wuzy_convex_polyhedron_collider_support(const void* userdata, wuzy_vec3 direction)
{
    // This is one of the most expensive steps of the GJK algorithm. There are ways to improve the
    // complexity of this function to O(log N) by hill climbing for extreme vertices, where vertices
    // are stored in a data structure which allows easy querying for neighbouring vertices (would be
    // trivial in 2D). This requires a good bit more work and it's tricky to avoid infinite loops,
    // so I won't do it until I need it.

    // The convexity of the polyhedron provides that the supporting point is always one of the
    // vertices, so it is enough to simply check all of them.
    const auto& poly = *reinterpret_cast<const wuzy_convex_polyhedron_collider_userdata*>(userdata);
    assert(poly.num_vertices > 0);
    wuzy_vec3 max_vtx;
    float max_dot = -FLT_MAX;
    for (size_t i = 0; i < poly.num_vertices; ++i) {
        const auto d = dot(poly.vertices[i], direction);
        if (d > max_dot) {
            max_dot = d;
            max_vtx = poly.vertices[i];
        }
    }
    return max_vtx;
}

EXPORT bool wuzy_convex_polyhedron_collider_ray_cast(
    const void* userdata, wuzy_vec3 start, wuzy_vec3 direction, wuzy_ray_cast_result* result)
{
    const auto& poly = *reinterpret_cast<const wuzy_convex_polyhedron_collider_userdata*>(userdata);
    // Real-Time Collision Detection, 5.3.8
    float tfirst = 0.0f;
    float tlast = std::numeric_limits<float>::max();
    wuzy_vec3 normal;
    // Check ray against each half-space defined by each face
    for (size_t f = 0; f < poly.num_faces; ++f) {
        const auto& n = poly.normals[f];
        const auto& v0 = poly.vertices[poly.face_indices[f].i0];
        const auto d = dot(v0, n);

        const auto denom = dot(n, direction);
        const auto dist = d - dot(n, start);
        if (denom == 0.0f) {
            // Ray is parallel to the face
            if (dist > 0.0f) {
                return false;
            }
        } else {
            const auto t = dist / denom;
            if (denom < 0.0f) {
                // Entering half-space
                if (t > tfirst) {
                    tfirst = t;
                    normal = n;
                }
            } else {
                // Exiting half-space
                if (t < tlast) {
                    tlast = t;
                }
            }
            if (tfirst > tlast) {
                return false;
            }
        }
    }
    result->t = tfirst;
    result->normal = normal;
    result->hit_position = add(start, mul(direction, result->t));
    return true;
}

namespace {
// Support function of the minkowski difference `a - b`.
wuzy_vec3 support(const wuzy_collider* a, const wuzy_collider* b, const wuzy_vec3& direction)
{
    const auto a_sup = wuzy_collider_support(a, direction);
    const auto b_sup = wuzy_collider_support(b, mul(direction, -1.0f));
    return sub(a_sup, b_sup);
}

bool same_half_space(const wuzy_vec3& a, const wuzy_vec3& b)
{
    return dot(a, b) > 0.0f;
}

struct NextSimplexResult {
    wuzy_simplex3d simplex;
    wuzy_vec3 direction;
    bool contains_origin;
};

NextSimplexResult line(const wuzy_simplex3d& simplex, const wuzy_vec3& /*direction*/)
{
    /*
     *            .           .
     *           .           .
     *          .           .
     *         b           .
     *   <3>  x           .
     *       . \         .
     *      .   \       .
     *     .     \     .
     *    .       \   .
     *   .         \ a
     *  .    <1>    x    <2>
     *             .
     *            .
     *           .
     */

    // The simplex is a line. The previously added point is b.
    // Since we have chosen the direction to be towards the origin o, a should be too and we
    // expect the origin to be in the half-space at b in the direction of b -> a (<1> or <2>,
    // not <3>).
    assert(simplex.num_vertices == 2);
    const auto a = simplex.vertices[0];
    const auto b = simplex.vertices[1];
    const auto ab = sub(b, a);
    const auto ao = mul(a, -1.0f);
    // We need to check whether the origin is between a and b (<1>) or "after" a (in the
    // half-space at a in the direction of b -> a, <2>).
    // If ao and ab are in the same half-space, the origin is between a and b (<1>).
    if (same_half_space(ab, ao)) {
        // <1>
        // ab.cross(ao) will give us a vector perpendicular to ab and ao. The second cross
        // product will give us a vector that is coplanar with ab and ao and perpendicular
        // to ab. This is roughly the vector from the line ab towards the origin.
        auto dir = cross(cross(ab, ao), ab);
        if (dot(dir, dir) < FLT_EPSILON) {
            // This will happen if a, b and the origin are (roughly) collinear!
            // Pick any direction orthogonal to ab.
            // https://box2d.org/posts/2014/02/computing-a-basis/
            dir = normalize(ab);
            assert(is_finite(dir));
            dir = std::abs(dir.x) >= 0.57735f ? wuzy_vec3 { dir.y, -dir.x, 0.0f }
                                              : wuzy_vec3 { 0.0f, dir.z, -dir.y };
        }
        assert(length(dir) > FLT_EPSILON);
        return { simplex, dir, false };
    } else {
        // <2>
        // If ao and ab are not in the same half-space, the origin is "after"/"beyond" a (<2>).
        // This means that the origin is not in the direction of b and it will not help us
        // enclose the origin, so we throw it away.
        // We keep only a (because it more towards the origin) and search towards the origin
        // again.
        return { wuzy_simplex3d { { a }, 1 }, ao, false };
    }
}

NextSimplexResult triangle(const wuzy_simplex3d& simplex, const wuzy_vec3& direction)
{
    /*
     *                   .
     *.      <6>       .
     *  .             .
     *    .        .
     *      .    .
     *        . c
     *         x                           .
     *         |\                        .
     *         |  \        <1>         .
     *         |    \                .
     *         |      \            .
     *         |        \        .
     *         |          \    .
     *         |            \ a
     *   <7>   | <2,3>       x      <5>
     *         |            / .
     *         |          /     .
     *         |        /         .
     *         |      /             .
     *         |    /                 .
     *         |  /                     .
     *         |/                         .
     *         x
     *        . b        <4>
     *      .     .
     *    .         .
     *  .     <8>     .
     *.                 .
     * There are two regions inside the triangle (<2> and <3>), because one is "above" and one
     *"below" the triangle.
     */

    assert(simplex.num_vertices == 3);
    const auto a = simplex.vertices[0];
    const auto b = simplex.vertices[1];
    const auto c = simplex.vertices[2];

    const auto ab = sub(b, a);
    const auto ac = sub(c, a);
    const auto ao = mul(a, -1.0f);
    const auto abc = cross(ab, ac); // normal of the triangle

    // Regions <6>, <7> and <8> are ruled out, because they do not point towards `a`.

    // First we try to rule out region <1> and <4>, because according to Casey, they rule out
    // the largest number of options, but I don't quite understand it.

    // `abc.cross(ac)` is the normal of the plane that contains a and c (coplanar), so we are
    // checking if the origin is in the half-space defined by that plane (<1> and <5>).
    if (same_half_space(cross(abc, ac), ao)) {
        // Now we still need to distinguish <1> and <5> by checking whether the origin is
        // towards C (<1>) or away from C (<5>).
        if (same_half_space(ac, ao)) {
            // <1>
            // The direction is a vector orthogonal to the edge `ac`, but pointing towards
            // the origin.
            const auto dir = cross(cross(ac, ao), ac);
            assert(is_finite(dir) && length(dir) > FLT_EPSILON);
            return { wuzy_simplex3d { { a, c }, 2 }, dir, false };
        } else {
            // <5>
            return line(wuzy_simplex3d { { a, b }, 2 }, direction);
        }
    }

    // `ab.cross(abc)` is the normal of the plane that contains a and b, so we are checking
    // for region <4>.
    if (same_half_space(cross(ab, abc), ao)) {
        // <4>
        return line(wuzy_simplex3d { { a, b }, 2 }, direction);
    }

    // <2> or <3> are left
    assert(is_finite(abc) && length(abc) > FLT_EPSILON);
    if (same_half_space(abc, ao)) {
        // "above" the triangle
        return { simplex, abc, false };
    } else {
        // "below" the triangle
        // "rewind" the triangle to point "down" instead
        return { wuzy_simplex3d { { a, c, b }, 3 }, mul(abc, -1.0f), false };
    }
}

NextSimplexResult tetrahedron(const wuzy_simplex3d& simplex, const wuzy_vec3& direction)
{
    /*
     * This is pretty bad, but I can't do it much better.
     *
     *                 a
     *                x
     *               /|\
     *              / | \
     *             /  |  \
     *            /   |    \
     *          /     |     \
     *         /      |      \ c
     *        /       | ......x
     *     d /    ....|...   /
     *      x.......  |     /
     *       \        |    /
     *         \      |   /
     *           \    |  /
     *             \  | /
     *               \|b
     *                x
     */

    assert(simplex.num_vertices == 4);
    const auto a = simplex.vertices[0];
    const auto b = simplex.vertices[1];
    const auto c = simplex.vertices[2];
    const auto d = simplex.vertices[3];

    const auto ab = sub(b, a);
    const auto ac = sub(c, a);
    const auto ad = sub(d, a);
    const auto ao = mul(a, -1.0f);

    const auto abc = cross(ab, ac);
    const auto acd = cross(ac, ad);
    const auto adb = cross(ad, ab);

    // Since we chose `a` last, the origin should be in the half-space defined by the plane bcd
    // in the direction of a. We just need to check if the origin is in the half-space defined
    // by the sides of the tetrahedron (pointing outside). If the origin is neither in the space
    // defined by abc, acd and adb, we know it must be inside the tetrahedron, because it can't
    // be in the space defined by bcd (see first statement).

    if (same_half_space(abc, ao)) {
        return triangle(wuzy_simplex3d { { a, b, c }, 3 }, direction);
    }
    if (same_half_space(acd, ao)) {
        return triangle(wuzy_simplex3d { { a, c, d }, 3 }, direction);
    }
    if (same_half_space(adb, ao)) {
        return triangle(wuzy_simplex3d { { a, d, b }, 3 }, direction);
    }

    return { simplex, direction, true };
}

NextSimplexResult next_simplex(const wuzy_simplex3d& simplex, const wuzy_vec3& direction)
{
    switch (simplex.num_vertices) {
    case 2:
        return line(simplex, direction);
    case 3:
        return triangle(simplex, direction);
    case 4:
        return tetrahedron(simplex, direction);
    default:
        assert(false && "Invalid number of points in simplex");
        std::abort();
    }
}

void push_front(wuzy_simplex3d& simplex, const wuzy_vec3& v)
{
    simplex.vertices[3] = simplex.vertices[2];
    simplex.vertices[2] = simplex.vertices[1];
    simplex.vertices[1] = simplex.vertices[0];
    simplex.vertices[0] = v;
    assert(simplex.num_vertices < 4);
    simplex.num_vertices++;
}
}

EXPORT bool wuzy_gjk(
    const wuzy_collider* c1, const wuzy_collider* c2, wuzy_simplex3d* result, wuzy_gjk_debug* debug)
{
    // As the initial direction we choose the x-axis. We want to bias the search towards the
    // origin/point of collision, so it would make sense to use the relative vector between the
    // centers of the shapes.
    // Casey says it doesn't really matter what we start with, because it converges really
    // quickly.
    auto direction = wuzy_vec3 { 1.0f, 0.0f, 0.0f };

    const auto a0 = support(c1, c2, direction);

    // TODO: Handle a0 == origin

    auto& simplex = *result;
    simplex.vertices[0] = a0;
    simplex.num_vertices = 1;

    if (debug) {
        debug->iterations = allocate<wuzy_gjk_debug_iteration>(debug->alloc, 1);
        debug->num_iterations = 1;
        debug->iterations[debug->num_iterations - 1] = {
            .direction = direction,
            .a_support = wuzy_collider_support(c1, direction),
            .b_support = wuzy_collider_support(c2, mul(direction, -1.0f)),
            .support = a0,
            .simplex = simplex,
            .contains_origin = false,
        };
    }

    // Choose dir towards the origin: a0 -> O = O - a0 = -a0.
    direction = mul(a0, -1.0f);

    if (debug) {
        debug->iterations = nullptr;
        debug->num_iterations = 0;
    }

    size_t num_iterations = 0;
    const auto debug_max_iterations
        = debug && debug->max_num_iterations ? debug->max_num_iterations : 64;
    while (num_iterations++ < debug_max_iterations) {
        // direction doesn't have to be normalized, but it can't be a null-vector
        assert(length(direction) > FLT_EPSILON);
        assert(is_finite(direction));
        const auto a = support(c1, c2, direction);
        assert(is_finite(a));

        if (debug) {
            // Yes, we are allocating a storm in here
            debug->iterations = reallocate<wuzy_gjk_debug_iteration>(
                debug->alloc, debug->iterations, debug->num_iterations, debug->num_iterations + 1);
            debug->num_iterations++;
            debug->iterations[debug->num_iterations - 1] = {
                .direction = direction,
                .a_support = wuzy_collider_support(c1, direction),
                .b_support = wuzy_collider_support(c2, mul(direction, -1.0f)),
                .support = a,
                .simplex = {},
                .contains_origin = false,
            };
        }

        if (dot(a, direction) < 0.0f) {
            // No Intersection:
            // We went towards the origin (direction points towards the origin) and the next
            // supporting point we found was away from the origin instead. That means we did not
            // "cross" the origin and there is no way to include the origin.
            return false;
        }

        push_front(simplex, a);

        auto res = next_simplex(simplex, direction);
        for (size_t i = 0; i < res.simplex.num_vertices; ++i) {
            assert(is_finite(res.simplex.vertices[i]));
        }

        if (debug) {
            debug->iterations[debug->num_iterations - 1].simplex = res.simplex;
            debug->iterations[debug->num_iterations - 1].contains_origin = res.contains_origin;
        }

        if (res.contains_origin) {
            return true;
        }
        simplex = res.simplex;
        direction = res.direction;
    }

    // We only reach this if we exceed the maximum number of iterations
    return false;
}

EXPORT void wuzy_gjk_debug_free(wuzy_gjk_debug* debug)
{
    deallocate(debug->alloc, debug->iterations, debug->num_iterations);
    debug->iterations = nullptr;
}

EXPORT bool wuzy_test_collision(
    const wuzy_collider* a, const wuzy_collider* b, wuzy_gjk_debug* debug)
{
    const auto a_aabb = wuzy_collider_get_aabb(a);
    const auto b_aabb = wuzy_collider_get_aabb(b);
    if (!overlap(a_aabb, b_aabb)) {
        return false;
    }
    wuzy_simplex3d res = {};
    return wuzy_gjk(a, b, &res, debug);
}

namespace {
struct EpaTriangle {
    size_t v0;
    size_t v1;
    size_t v2;
    wuzy_vec3 normal = { 0.0f, 0.0f, 0.0f };
    float dist = 0.0f; // Distance to origin
};

void update_normal(std::span<const wuzy_vec3> vertices, EpaTriangle& face, bool flip = true)
{
    assert(face.v0 < vertices.size() && face.v1 < vertices.size() && face.v2 < vertices.size());
    const auto& v0 = vertices[face.v0];
    const auto& v1 = vertices[face.v1];
    const auto& v2 = vertices[face.v2];

    face.normal = normalize(cross(sub(v1, v0), sub(v2, v0)));
    assert(is_finite(face.normal));
    // The shortest path from the origin to the face is orthogonal to the plane (face), i.e.
    // along the normal.
    // Expressed in the base of the plane (normal + 2 tangent vectors) every point on the
    // plane has the same coefficient for its normal component.
    // Therefore we can determine the distance of the plane to the origin by
    // |dot(normal, p)|, where p is any point on the plane. We choose v0.
    // We also want to orient the normal correctly and since the polytope contains the
    // origin and is convex, every normal has to point away from the origin.
    // We can ensure this by making sure that the normals point in the same direction as any
    // point on the plane (e.g. a vertex).
    // So to make the normal point "outside" we have to make sure that normal.dot(-a) < 0.0f
    // or normal.dot(a) > 0.0.
    face.dist = dot(face.normal, v0);
    if (face.dist < 0.0f) {
        // Even if we know the face is correctly oriented and we do not want to flip it
        // (flip=false), we have to negatee face.dist or get_closest_face will do the wrong thing
        // (or raise an assertion).
        if (flip) {
            // Fix winding order of the triangle so that v0 -> v1 -> v2 is CCW
            std::swap(face.v1, face.v2);
            face.normal = mul(face.normal, -1.0f);
        }
        face.dist = -face.dist;
    }
}

// Returns the index of the face closest to the origin and its distance.
std::pair<size_t, float> get_closest_face(std::span<const EpaTriangle> faces)
{
    float min_dist = std::numeric_limits<float>::max();
    size_t min_face_idx = 0;
    assert(faces.size() > 0);
    for (size_t i = 0; i < faces.size(); ++i) {
        assert(faces[i].normal.x != 0.0f || faces[i].normal.y != 0.0f || faces[i].normal.z != 0.0f);
        assert(faces[i].dist >= 0.0f);
        if (faces[i].dist < min_dist) {
            min_dist = faces[i].dist;
            min_face_idx = i;
        }
    }
    return { min_face_idx, min_dist };
}

using Edge = std::pair<size_t, size_t>;

void add_unique_edge(VecAdapter<Edge>& edges_to_patch, size_t first, size_t second)
{
    // If the edge is part of multiple triangles, we DO NOT want to patch it, because it
    // would create internal geometry, which will mess up everything.
    // If the edge is part of multiple triangles, it will be one other adjacent triangle at
    // most and the edge's direction will be reversed (if both triangles are CCW, which we
    // made sure of in update_normal).
    const auto reverse_edge_idx = edges_to_patch.find(Edge(second, first));
    assert(edges_to_patch.find(Edge(first, second)) > edges_to_patch.size());
    if (reverse_edge_idx < edges_to_patch.size()) {
        edges_to_patch.pop_and_swap(reverse_edge_idx);
    } else {
        edges_to_patch.push_back(Edge(first, second));
    }
};
}

EXPORT wuzy_collision_result wuzy_epa(const wuzy_collider* c1, const wuzy_collider* c2,
    const wuzy_simplex3d* simplex, wuzy_epa_debug* debug)
{
    // Our variant of GJK always computes a tetrahedron before returning true.
    // Other variants might not do that and would require extension of the simplex to at least 4
    // vertices.
    assert(simplex->num_vertices == 4);

    static constexpr auto max_num_iterations = 32;

    // 4 vertices from initial simplex + one in every iteration
    std::array<wuzy_vec3, 4 + max_num_iterations> polytope_vertices_data;
    VecAdapter<wuzy_vec3> polytope_vertices(polytope_vertices_data);
    for (size_t i = 0; i < 4; ++i) {
        assert(is_finite(simplex->vertices[i]));
        polytope_vertices.push_back(simplex->vertices[i]);
    }

    // 4 faces from initial simplex + 2 faces per iteration
    std::array<EpaTriangle, 4 + max_num_iterations * 2> polytope_faces_data;
    VecAdapter<EpaTriangle> polytope_faces(polytope_faces_data);
    // If you look at `triangle` (part of GJK) you will see that the origin is towards v0 (a),
    // looking from the face defined by v1, v2, v3 (b, c, d).
    // From that you can derive a correct winding order (see below).

    // If the origin is very close to a face of the initial simplex (< FLT_EPSILON),
    // it's possible that GJK will determine that the origin is inside the simplex, but
    // `update_normal` above will disagree, just because of floating point errors and try
    // to flip a normal, creating internal geometry/a simplex with holes/a face
    // with the wrong winding order. From then on everything else will go wrong.
    // This is a very rare case and requires significant noodling around in a test level, but it
    // is possible - bummer.
    // This is why below we pass `flip=false` to `update_normal` to keep the faces
    // as they are, because we know they face the correct way and consequently keep the correct
    // normals so the polytope is expanded in the right directions.
    polytope_faces.push_back({ 0, 1, 2 });
    polytope_faces.push_back({ 0, 2, 3 });
    polytope_faces.push_back({ 0, 3, 1 });
    polytope_faces.push_back({ 1, 3, 2 });

    for (auto& face : polytope_faces) {
        update_normal(polytope_vertices, face, false);
    }

    // I don't know if there is a good way to calculate a good upper bound for this, but this seems
    // to work for now.
    std::array<Edge, 16> edges_to_patch_data;
    VecAdapter<Edge> edges_to_patch(edges_to_patch_data);

    size_t min_dist_face_idx = 0;
    float min_face_dist = 0.0f;

    if (debug) {
        debug->iterations = nullptr;
        debug->num_iterations = 0;
    }

    size_t num_iterations = 0;
    const auto debug_max_iterations
        = debug && debug->max_num_iterations ? debug->max_num_iterations : max_num_iterations;
    while (num_iterations++ < debug_max_iterations) {
        std::tie(min_dist_face_idx, min_face_dist) = get_closest_face(polytope_faces);

        // Expand the polytope "outside" from the face closest to the origin.
        const auto min_dist_face_normal = polytope_faces[min_dist_face_idx].normal;
        const auto sup_point = support(c1, c2, min_dist_face_normal);
        const auto sup_dist = dot(min_dist_face_normal, sup_point);
        assert(is_finite(sup_point));

        if (debug) {
            // Yes, we are allocating a storm in here
            debug->iterations = reallocate<wuzy_epa_debug_iteration>(
                debug->alloc, debug->iterations, debug->num_iterations, debug->num_iterations + 1);

            debug->num_iterations++;
            auto& it = debug->iterations[debug->num_iterations - 1];

            it.num_polytope_vertices = polytope_vertices.size();
            it.polytope_vertices = allocate<wuzy_vec3>(debug->alloc, it.num_polytope_vertices);
            std::memcpy(it.polytope_vertices, polytope_vertices.data(),
                polytope_vertices.size() * sizeof(wuzy_vec3));

            it.num_polytope_faces = polytope_faces.size();
            it.polytope_faces = allocate<wuzy_epa_debug_face>(debug->alloc, it.num_polytope_faces);
            std::memcpy(it.polytope_faces, polytope_faces.data(),
                polytope_faces.size() * sizeof(EpaTriangle));

            it.face_removed = allocate<bool>(debug->alloc, it.num_polytope_faces);
            std::memset(it.face_removed, 0, sizeof(bool) * it.num_polytope_faces);

            it.min_dist_face_index = min_dist_face_idx;
            it.min_face_dist = min_face_dist;
            it.support_point = sup_point;
            it.support_dist = sup_dist;
            it.edges_to_patch = nullptr;
            it.num_edges_to_patch = 0;
        }

        // If we cannot expand the polytope (the new point is on the closest edge), the closest
        // face is actually the closest face possible and we have reached the edge of the
        // minkowski difference.
        if (std::abs(sup_dist - min_face_dist) < 1e-4f) {
            break;
        }

        // Remove all faces with a normal towards the new point
        edges_to_patch.clear();
        size_t orig_face_idx = 0;
        for (size_t i = 0; i < polytope_faces.size();) {
            // So many implementations check for dot(normal, sup_point) > 0 here, which is
            // clearly wrong.
            // Counter-example: Plane at (2,0) with normal (1,0) and new supporting point
            // at (1,0) (or (1,5) or something for a more realistic scenario). In this case
            // dot(normal, sup_point) would be > 0, but the point is clearly on the wrong side of
            // the plane.
            const auto face_points_towards_sup = same_half_space(
                polytope_faces[i].normal, sub(sup_point, polytope_vertices[polytope_faces[i].v0]));

            if (debug) {
                debug->iterations[debug->num_iterations - 1].face_removed[orig_face_idx++]
                    = face_points_towards_sup;
            }

            if (face_points_towards_sup) {
                add_unique_edge(edges_to_patch, polytope_faces[i].v0, polytope_faces[i].v1);
                add_unique_edge(edges_to_patch, polytope_faces[i].v1, polytope_faces[i].v2);
                add_unique_edge(edges_to_patch, polytope_faces[i].v2, polytope_faces[i].v0);
                polytope_faces.pop_and_swap(i);
            } else {
                ++i;
            }
        }

        if (debug) {
            auto& it = debug->iterations[debug->num_iterations - 1];
            it.num_edges_to_patch = edges_to_patch.size();
            it.edges_to_patch = allocate<wuzy_epa_debug_edge>(debug->alloc, it.num_edges_to_patch);
            std::memcpy(
                it.edges_to_patch, edges_to_patch.data(), edges_to_patch.size() * sizeof(Edge));
        }

        // Patch up the freshly cut edges (tasty!) with the new point
        polytope_vertices.push_back(sup_point);
        for (const auto& [first, second] : edges_to_patch) {
            polytope_faces.push_back({ first, second, polytope_vertices.size() - 1 });
            update_normal(polytope_vertices, polytope_faces[polytope_faces.size() - 1]);
        }
    }

    // TODO: Project the origin into the closest face to approximate contact points

    return {
        polytope_faces[min_dist_face_idx].normal,
        // Add some epsilon to make sure we resolve the collision
        min_face_dist + 1e-4f,
    };
}

EXPORT void wuzy_epa_debug_free(wuzy_epa_debug* debug)
{
    for (size_t i = 0; i < debug->num_iterations; ++i) {
        const auto& it = debug->iterations[i];
        deallocate(debug->alloc, it.polytope_vertices, it.num_polytope_vertices);
        deallocate(debug->alloc, it.polytope_faces, it.num_polytope_faces);
        deallocate(debug->alloc, it.face_removed, it.num_polytope_faces);
        deallocate(debug->alloc, it.edges_to_patch, it.num_edges_to_patch);
    }
    deallocate(debug->alloc, debug->iterations, debug->num_iterations);
    debug->iterations = nullptr;
}

EXPORT bool wuzy_get_collision(const wuzy_collider* a, const wuzy_collider* b,
    wuzy_collision_result* result, wuzy_gjk_debug* gjk_debug, wuzy_epa_debug* epa_debug)
{
    const auto a_aabb = wuzy_collider_get_aabb(a);
    const auto b_aabb = wuzy_collider_get_aabb(b);
    if (!overlap(a_aabb, b_aabb)) {
        return false;
    }
    wuzy_simplex3d simplex = {};
    if (!wuzy_gjk(a, b, &simplex, gjk_debug)) {
        return false;
    }
    *result = wuzy_epa(a, b, &simplex, epa_debug);
    return true;
}

namespace {
uint32_t id_get_idx(uint32_t id)
{
    return id & 0xFFFF;
}

uint32_t id_get_gen(uint32_t id)
{
    return id >> 16;
}

uint32_t id_combine(uint32_t idx, uint32_t gen)
{
    return (gen << 16) | idx;
}

struct Node {
    uint16_t generation = 1;

    wuzy_collider* collider = nullptr;
    wuzy_aabb aabb;
    uint64_t bitmask = 0;

    Node* parent = nullptr;
    Node* left = nullptr;
    Node* right = nullptr;

    bool is_leaf() const { return !left && !right; }

    void update_from_children()
    {
        assert(left && right);
        aabb = combine(left->aabb, right->aabb);
        bitmask = left->bitmask | right->bitmask;
    }

    void replace_child(Node* old_child, Node* new_child)
    {
        assert(left == old_child || right == old_child);
        if (left == old_child) {
            left = new_child;
        } else {
            right = new_child;
        }
    }
};

struct AabbTree {
    wuzy_allocator* alloc;
    size_t max_num_colliders;
    size_t max_num_nodes;
    size_t num_nodes = 0;
    size_t num_colliders = 0;
    Node* nodes;
    size_t* free_list;
    size_t free_list_size = 0;
    Node* root = nullptr;

    Node* get_new_node()
    {
        if (free_list_size) {
            const auto idx = free_list[free_list_size - 1];
            free_list_size--;
            auto node = nodes + idx;
            node->parent = nullptr;
            node->left = nullptr;
            node->right = nullptr;
            return nodes + idx;
        }
        if (num_nodes < max_num_nodes) {
            num_nodes++;
            return nodes + num_nodes - 1;
        }
        return nullptr;
    }

    void free_node(Node* node)
    {
        node->parent = nullptr;
        node->left = nullptr;
        node->right = nullptr;
        node->generation++;
        if (node->collider) {
            num_colliders--;
        }
        assert(free_list_size < max_num_nodes);
        free_list[free_list_size] = static_cast<size_t>(node - nodes);
        free_list_size++;
    }

    bool insert(Node* node, Node* parent)
    {
        // http://allenchou.net/2014/02/game-physics-broadphase-dynamic-aabb-tree/
        // https://box2d.org/files/ErinCatto_DynamicBVH_Full.pdf
        if (!root) {
            root = node;
            return true;
        }
        assert(parent);

        if (parent->is_leaf()) {
            // Insert a new non-leaf node that will be the parent of `node` and `parent`
            const auto new_parent = get_new_node();
            if (!new_parent) {
                return false;
            }

            new_parent->left = node;
            node->parent = new_parent;

            new_parent->right = parent;
            auto old_parent = parent->parent;
            if (!old_parent) {
                assert(parent == root);
                root = new_parent;
            } else {
                old_parent->replace_child(parent, new_parent);
                new_parent->parent = old_parent;
            }
            parent->parent = new_parent;

            new_parent->update_from_children();
            return true;
        } else {
            // We try to find a good place for the new node and we do this by minimizing the
            // increase in total volume of our nodes. The probability of a ray hitting an AABB is
            // roughly proportional to its surface area and the probability for an AABB to collide
            // (overlap) with another is roughly proportional to its volume. I chose to prioritize
            // collision detection performance here, so I use the volume.
            // The less volume our nodes have, the more empty space we can rule out in each
            // iteration.
            // Therefore we define the total cost of a tree as the volume of all it's interior
            // nodes. The cost of the leaf nodes is the same in every configuration, so we do not
            // have to include it, but this is irrelevant here.
            assert(parent->left && parent->right);
            const auto left_vol_diff
                = volume(combine(parent->left->aabb, node->aabb)) - volume(parent->left->aabb);
            const auto right_vol_diff
                = volume(combine(parent->right->aabb, node->aabb)) - volume(parent->right->aabb);
            if (left_vol_diff <= right_vol_diff) {
                insert(node, parent->left);
            } else {
                insert(node, parent->right);
            }
            parent->update_from_children();
            return true;
        }
    }

    void remove(Node* node)
    {
        if (!node->parent) {
            assert(node == root);
            root = nullptr;
        } else {
            //   p      # parent
            // n   s    # node, sibling
            assert(node->parent->left == node || node->parent->right == node);
            auto sibling = node->parent->left == node ? node->parent->right : node->parent->left;
            assert(sibling);
            if (node->parent == root) {
                free_node(node->parent);
                root = sibling;
                sibling->parent = nullptr;
            } else {
                //      g       # grand parent
                //  ?       p
                //        n   s
                auto grand_parent = node->parent->parent;
                assert(grand_parent);
                free_node(node->parent);
                sibling->parent = grand_parent;
                grand_parent->replace_child(node->parent, sibling);
            }
        }
    }

    uint32_t get_id(Node* node) const
    {
        const auto idx = static_cast<uint32_t>(node - nodes);
        return id_combine(idx, nodes[idx].generation);
    }

    Node* get_node(uint32_t id) const
    {
        const auto idx = id_get_idx(id);
        const auto gen = id_get_gen(id);
        assert(idx < num_nodes);
        auto node = nodes + idx;
        return node->generation == gen ? node : nullptr;
    }
};
}

EXPORT wuzy_aabb_tree* wuzy_aabb_tree_create(size_t max_num_colliders, wuzy_allocator* alloc)
{
    // Since the binary tree is a "full binary tree" (every internal node has two children), the
    // most unfortunate arrangement of nodes is a "linear spine", where every e.g. left child is an
    // internal node and every right child is a leaf (or the other way around).
    // In that case we need N internal nodes for N leaf nodes, which is 2N in total.
    const auto max_num_nodes = 2 * max_num_colliders;
    if (max_num_nodes > 0xffff) {
        return nullptr;
    }
    alloc = alloc ? alloc : default_allocator();
    auto tree = allocate<AabbTree>(alloc);
    if (!tree) {
        return nullptr;
    }
    tree->alloc = alloc;
    tree->max_num_colliders = max_num_colliders;
    tree->max_num_nodes = max_num_nodes;
    tree->nodes = allocate<Node>(alloc, max_num_nodes);
    if (!tree->nodes) {
        deallocate(tree->alloc, tree);
        return nullptr;
    }
    tree->free_list = allocate<size_t>(alloc, max_num_nodes);
    if (!tree->free_list) {
        deallocate(tree->alloc, tree->nodes, tree->max_num_nodes);
        deallocate(tree->alloc, tree);
    }
    return reinterpret_cast<wuzy_aabb_tree*>(tree);
}

EXPORT void wuzy_aabb_tree_destroy(wuzy_aabb_tree* wtree)
{
    auto tree = reinterpret_cast<AabbTree*>(wtree);
    deallocate(tree->alloc, tree->nodes, tree->max_num_nodes);
    deallocate(tree->alloc, tree->free_list, tree->max_num_nodes);
    deallocate(tree->alloc, tree);
}

namespace {
Node* insert_node(AabbTree* tree, Node* parent, wuzy_aabb_tree_init_node* init_node)
{
    auto node = tree->get_new_node();
    if (!node) {
        return node;
    }
    init_node->id.id = tree->get_id(node);
    node->collider = init_node->collider;
    if (node->collider) { // leaf node
        assert(!init_node->left && !init_node->right);
        node->bitmask = init_node->bitmask ? init_node->bitmask : static_cast<uint64_t>(-1);
        node->aabb = wuzy_collider_get_aabb(init_node->collider);
        node->parent = parent;
        tree->num_colliders++;
    } else {
        assert(init_node->left && init_node->right);
        node->left = insert_node(tree, node, init_node->left);
        node->right = insert_node(tree, node, init_node->right);
        node->update_from_children();
    }
    return node;
}
}

EXPORT bool wuzy_aabb_tree_init(wuzy_aabb_tree* wtree, wuzy_aabb_tree_init_node* root_node)
{
    auto tree = reinterpret_cast<AabbTree*>(wtree);
    assert(tree->num_nodes == 0);
    tree->root = insert_node(tree, nullptr, root_node);
    return tree->root;
}

EXPORT wuzy_aabb_tree_node wuzy_aabb_tree_insert(
    wuzy_aabb_tree* wtree, wuzy_collider* collider, uint64_t bitmask)
{
    auto tree = reinterpret_cast<AabbTree*>(wtree);
    auto node = tree->get_new_node();
    if (!node) {
        return { 0 };
    }
    node->collider = collider;
    node->aabb = wuzy_collider_get_aabb(collider);
    node->bitmask = bitmask ? bitmask : static_cast<uint64_t>(-1);
    if (!tree->insert(node, tree->root)) {
        tree->free_node(node);
        return { 0 };
    }
    tree->num_colliders++;
    return { tree->get_id(node) };
}

EXPORT wuzy_collider* wuzy_aabb_tree_get_collider(wuzy_aabb_tree* wtree, wuzy_aabb_tree_node wnode)
{
    auto tree = reinterpret_cast<AabbTree*>(wtree);
    auto node = tree->get_node(wnode.id);
    return node ? node->collider : nullptr;
}

EXPORT bool wuzy_aabb_tree_update(
    wuzy_aabb_tree* wtree, wuzy_aabb_tree_node wnode, uint64_t bitmask)
{
    auto tree = reinterpret_cast<AabbTree*>(wtree);
    auto node = tree->get_node(wnode.id);
    if (!node) {
        return false;
    }
    tree->remove(node);
    node->aabb = wuzy_collider_get_aabb(node->collider);
    node->bitmask = bitmask ? bitmask : node->bitmask;
    tree->insert(node, tree->root); // We ignore the return, because it should have worked
    return true;
}

EXPORT bool wuzy_aabb_tree_remove(wuzy_aabb_tree* wtree, wuzy_aabb_tree_node wnode)
{
    auto tree = reinterpret_cast<AabbTree*>(wtree);
    auto node = tree->get_node(wnode.id);
    if (!node) {
        return false;
    }
    tree->remove(node);
    tree->free_node(node);
    return true;
}

namespace {
wuzy_aabb_tree_dump_node* add_node(const AabbTree* tree, Node* node,
    wuzy_aabb_tree_dump_node* parent, wuzy_aabb_tree_dump_node* nodes, size_t* num_nodes,
    size_t max_num_nodes)
{
    if (!node) {
        return nullptr;
    }
    const auto idx = (*num_nodes)++;
    nodes[idx].id = wuzy_aabb_tree_node { tree->get_id(node) };
    nodes[idx].collider = node->collider;
    nodes[idx].aabb = node->aabb;
    nodes[idx].parent = parent;
    nodes[idx].left = add_node(tree, node->left, nodes + idx, nodes, num_nodes, max_num_nodes);
    nodes[idx].right = add_node(tree, node->right, nodes + idx, nodes, num_nodes, max_num_nodes);
    return nodes + idx;
}
}

EXPORT size_t wuzy_aabb_tree_dump(
    const wuzy_aabb_tree* wtree, wuzy_aabb_tree_dump_node* nodes, size_t max_num_nodes)
{
    auto tree = reinterpret_cast<const AabbTree*>(wtree);
    if (!nodes || max_num_nodes < tree->num_nodes) {
        return tree->num_nodes;
    }
    size_t num_nodes = 0;
    add_node(tree, tree->root, nullptr, nodes, &num_nodes, max_num_nodes);
    return num_nodes;
}

void wuzy_aabb_tree_get_stats(const wuzy_aabb_tree* wtree, wuzy_aabb_tree_stats* stats)
{
    auto tree = reinterpret_cast<const AabbTree*>(wtree);
    stats->num_colliders = tree->num_colliders;
    stats->num_nodes = tree->num_nodes;
    stats->max_num_nodes = tree->max_num_nodes;
}

namespace {
struct NodeQuery {
    enum class Type { Point, Aabb };

    wuzy_allocator* alloc;
    const AabbTree* tree;
    Node** node_stack;
    size_t node_stack_size;
    size_t node_stack_capacity;
    uint64_t bitmask;
    Type type;
    union {
        wuzy_vec3 point;
        wuzy_aabb aabb;
    } params;
    wuzy_query_debug* debug;
};
}

EXPORT wuzy_aabb_tree_node_query* wuzy_aabb_tree_node_query_create(
    const wuzy_aabb_tree* wtree, wuzy_allocator* alloc)
{
    alloc = alloc ? alloc : default_allocator();
    auto tree = reinterpret_cast<const AabbTree*>(wtree);
    auto query = allocate<NodeQuery>(alloc);
    if (!query) {
        return nullptr;
    }
    query->alloc = alloc;
    query->tree = tree;
    // If the tree is a linear spine (see wuzy_aabb_tree_create) the depth of the tree is N, where N
    // is the number of leaves. So we need to allocate a node stack of size max_num_colliders.
    query->node_stack_capacity = tree->max_num_colliders;
    query->node_stack = allocate<Node*>(alloc, query->node_stack_capacity);
    if (!query->node_stack) {
        deallocate(query->alloc, query);
        return nullptr;
    }
    return reinterpret_cast<wuzy_aabb_tree_node_query*>(query);
}

EXPORT void wuzy_aabb_tree_node_query_destroy(wuzy_aabb_tree_node_query* wquery)
{
    auto query = reinterpret_cast<NodeQuery*>(wquery);
    deallocate(query->alloc, query->node_stack, query->node_stack_capacity);
    deallocate(query->alloc, query);
}

EXPORT void wuzy_aabb_tree_node_query_point_begin(
    wuzy_aabb_tree_node_query* wquery, wuzy_vec3 point, uint64_t bitmask, wuzy_query_debug* debug)
{
    auto query = reinterpret_cast<NodeQuery*>(wquery);
    query->bitmask = bitmask ? bitmask : static_cast<uint64_t>(-1);
    query->type = NodeQuery::Type::Point;
    query->params.point = point;
    query->debug = debug;
    query->node_stack_size = 0;
    if (query->tree->root) {
        query->node_stack[query->node_stack_size++] = query->tree->root;
    }
}

EXPORT void wuzy_aabb_tree_node_query_aabb_begin(wuzy_aabb_tree_node_query* wquery,
    const wuzy_aabb* aabb, uint64_t bitmask, wuzy_query_debug* debug)
{
    auto query = reinterpret_cast<NodeQuery*>(wquery);
    query->bitmask = bitmask ? bitmask : static_cast<uint64_t>(-1);
    query->type = NodeQuery::Type::Aabb;
    query->params.aabb = *aabb;
    query->debug = debug;
    query->node_stack_size = 0;
    if (query->tree->root) {
        query->node_stack[query->node_stack_size++] = query->tree->root;
    }
}

namespace {
size_t query_point_next(NodeQuery* query, wuzy_aabb_tree_node* nodes, size_t max_nodes)
{
    size_t num_nodes = 0;
    while (query->node_stack_size && num_nodes < max_nodes) {
        auto node = query->node_stack[--query->node_stack_size];
        if (query->debug) {
            query->debug->nodes_checked++;
        }

        if ((node->bitmask & query->bitmask) == 0) {
            continue;
        }
        if (query->debug) {
            query->debug->bitmask_checks_passed++;
        }

        if (contains(node->aabb, query->params.point)) {
            if (query->debug) {
                query->debug->aabb_checks_passed++;
            }
            if (node->is_leaf()) {
                if (query->debug) {
                    query->debug->leaves_checked++;
                    query->debug->full_checks_passed++;
                }
                nodes[num_nodes++] = wuzy_aabb_tree_node { query->tree->get_id(node) };
            } else {
                assert(query->node_stack_size + 2 <= query->node_stack_capacity);
                query->node_stack[query->node_stack_size++] = node->left;
                query->node_stack[query->node_stack_size++] = node->right;
            }
        }
    }
    return num_nodes;
}

size_t query_aabb_next(NodeQuery* query, wuzy_aabb_tree_node* nodes, size_t max_nodes)
{
    size_t num_nodes = 0;
    while (query->node_stack_size && num_nodes < max_nodes) {
        auto node = query->node_stack[--query->node_stack_size];
        if (query->debug) {
            query->debug->nodes_checked++;
        }

        if ((node->bitmask & query->bitmask) == 0) {
            continue;
        }
        if (query->debug) {
            query->debug->bitmask_checks_passed++;
        }

        if (overlap(node->aabb, query->params.aabb)) {
            if (query->debug) {
                query->debug->aabb_checks_passed++;
            }
            if (node->is_leaf()) {
                if (query->debug) {
                    query->debug->leaves_checked++;
                    query->debug->full_checks_passed++;
                }
                nodes[num_nodes++] = wuzy_aabb_tree_node { query->tree->get_id(node) };
            } else {
                assert(query->node_stack_size + 2 <= query->node_stack_capacity);
                query->node_stack[query->node_stack_size++] = node->left;
                query->node_stack[query->node_stack_size++] = node->right;
            }
        }
    }
    return num_nodes;
}
}

EXPORT size_t wuzy_aabb_tree_node_query_next(
    wuzy_aabb_tree_node_query* wquery, wuzy_aabb_tree_node* nodes, size_t max_nodes)
{
    auto query = reinterpret_cast<NodeQuery*>(wquery);
    if (query->type == NodeQuery::Type::Point) {
        return query_point_next(query, nodes, max_nodes);
    } else if (query->type == NodeQuery::Type::Aabb) {
        return query_aabb_next(query, nodes, max_nodes);
    } else {
        assert(false && "Unreachable");
        return 0;
    }
}

EXPORT bool wuzy_aabb_tree_node_query_ray_cast(wuzy_aabb_tree_node_query* wquery, wuzy_vec3 start,
    wuzy_vec3 direction, uint64_t bitmask, wuzy_aabb_tree_ray_cast_result* result,
    wuzy_query_debug* debug)
{
    bitmask = bitmask ? bitmask : static_cast<uint64_t>(-1);
    auto query = reinterpret_cast<NodeQuery*>(wquery);
    query->node_stack_size = 0;
    if (query->tree->root) {
        query->node_stack[query->node_stack_size++] = query->tree->root;
    }

    bool hit = false;
    wuzy_ray_cast_result temp_res;
    while (query->node_stack_size) {
        auto node = query->node_stack[--query->node_stack_size];
        if (debug) {
            debug->nodes_checked++;
        }

        if ((node->bitmask & bitmask) == 0) {
            continue;
        }
        if (debug) {
            debug->bitmask_checks_passed++;
        }

        const auto aabb_hit = ray_cast(node->aabb, start, direction, &temp_res);
        if (aabb_hit && (!hit || temp_res.t < result->result.t)) {
            if (debug) {
                debug->aabb_checks_passed++;
            }
            if (node->is_leaf()) {
                if (debug) {
                    debug->leaves_checked++;
                }
                const auto collider_hit
                    = wuzy_collider_ray_cast(node->collider, start, direction, &temp_res);
                if (collider_hit && (!hit || temp_res.t < result->result.t)) {
                    if (debug) {
                        debug->full_checks_passed++;
                    }
                    result->node = wuzy_aabb_tree_node { query->tree->get_id(node) };
                    result->result = temp_res;
                    hit = true;
                }
            } else {
                assert(query->node_stack_size + 2 <= query->node_stack_capacity);
                query->node_stack[query->node_stack_size++] = node->left;
                query->node_stack[query->node_stack_size++] = node->right;
            }
        }
    }
    return hit;
}
