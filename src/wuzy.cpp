#include "wuzy/wuzy.h"

#include <array>
#include <cassert>
#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <span>
#include <tuple>
#include <vector>

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

static constexpr int x = 0;
static constexpr int y = 1;
static constexpr int z = 2;
static constexpr int w = 3;

struct vec3;

struct vec3_view {
    const float* data;

    explicit vec3_view(const float v[3]) : data(v) { }
    vec3_view(const vec3& v);

    float operator[](size_t idx) const { return *(data + idx); }
};

struct vec3 {
    float x, y, z;

    float* data() { return &x; }
    const float* data() const { return &x; }
    float operator[](size_t idx) const { return *(&x + idx); }
    float& operator[](size_t idx) { return *(&x + idx); }
    vec3& operator=(const vec3_view& v)
    {
        x = v[0];
        y = v[1];
        z = v[2];
        return *this;
    }
};

vec3_view::vec3_view(const vec3& v) : data(&v.x) { }

vec3_view v3(const float v[3])
{
    return vec3_view(v);
}

struct vec4 {
    float x, y, z, w;
};

struct vec4_view {
    const float* data;

    explicit vec4_view(const float v[4]) : data(v) { }
    vec4_view(const vec4& v) : data(&v.x) { }

    float operator[](size_t idx) const { return *(data + idx); }
};

vec3 make_vec3(const float v[3])
{
    return { v[x], v[y], v[z] };
}

void copy(float dst[3], vec3_view src)
{
    std::memcpy(dst, src.data, sizeof(float) * 3);
}

vec3 add(vec3_view a, vec3_view b)
{
    return vec3 { a[x] + b[x], a[y] + b[y], a[z] + b[z] };
}

vec3 sub(vec3_view a, vec3_view b)
{
    return vec3 { a[x] - b[x], a[y] - b[y], a[z] - b[z] };
}

float dot(vec3_view a, vec3_view b)
{
    return a[x] * b[x] + a[y] * b[y] + a[z] * b[z];
}

float dot(vec4_view a, vec4_view b)
{
    return a[x] * b[x] + a[y] * b[y] + a[z] * b[z] + a[w] * b[w];
}

vec3 cross(vec3_view a, vec3_view b)
{
    return vec3 {
        a[y] * b[z] - a[z] * b[y],
        a[z] * b[x] - a[x] * b[z],
        a[x] * b[y] - a[y] * b[x],
    };
}

float length(vec3_view v)
{
    return std::sqrt(dot(v, v));
}

float length(vec4_view v)
{
    return std::sqrt(dot(v, v));
}

vec3 mul(vec3_view v, float s)
{
    return vec3 { v[x] * s, v[y] * s, v[z] * s };
}

vec4 mul(vec4_view v, float s)
{
    return vec4 { v[x] * s, v[y] * s, v[z] * s, v[w] * s };
}

vec3 normalize(vec3_view v)
{
    const auto len = length(v);
    return vec3 { v[x] / len, v[y] / len, v[z] / len };
}

[[maybe_unused]] bool is_finite(vec3_view v)
{
    return std::isfinite(v[x]) && std::isfinite(v[y]) && std::isfinite(v[z]);
}

vec3 vmin(vec3_view a, vec3_view b)
{
    return { std::min(a[x], b[x]), std::min(a[y], b[y]), std::min(a[z], b[z]) };
}

vec3 vmax(vec3_view a, vec3_view b)
{
    return { std::max(a[x], b[x]), std::max(a[y], b[y]), std::max(a[z], b[z]) };
}

// Matrix operations (column-major 4x4)

struct mat4 {
    vec4 cols[4];

    vec4& operator[](size_t idx) { return cols[idx]; }
    const vec4& operator[](size_t idx) const { return cols[idx]; }
};

struct mat4_view {
    const float* data;

    explicit mat4_view(const float v[16]) : data(v) { }
    mat4_view(const mat4& v) : data(&v.cols[0].x) { }

    vec4_view operator[](size_t idx) const { return vec4_view(data + idx * 4); }
};

mat4_view m4(const float m[16])
{
    return mat4_view(m);
}

mat4 transpose(mat4_view m)
{
    return {
        vec4 { m[0][x], m[1][x], m[2][x], m[3][x] },
        vec4 { m[0][y], m[1][y], m[2][y], m[3][y] },
        vec4 { m[0][z], m[1][z], m[2][z], m[3][z] },
        vec4 { m[0][w], m[1][w], m[2][w], m[3][w] },
    };
}

void copy(float dst[16], mat4_view src)
{
    std::memcpy(dst, src.data, sizeof(float) * 16);
}

mat4 mul(mat4_view a, mat4_view b)
{
    const auto tr = transpose(a);
    const auto& r = tr.cols; // a rows
    return mat4 {
        vec4 { dot(r[0], b[0]), dot(r[1], b[0]), dot(r[2], b[0]), dot(r[3], b[0]) },
        vec4 { dot(r[0], b[1]), dot(r[1], b[1]), dot(r[2], b[1]), dot(r[3], b[1]) },
        vec4 { dot(r[0], b[2]), dot(r[1], b[2]), dot(r[2], b[2]), dot(r[3], b[2]) },
        vec4 { dot(r[0], b[3]), dot(r[1], b[3]), dot(r[2], b[3]), dot(r[3], b[3]) },
    };
}

vec3 mul(mat4_view m, vec3_view v, float w)
{
    const auto tr = transpose(m);
    const auto& rows = tr.cols;
    const auto vec = vec4 { v[x], v[y], v[z], w };
    return vec3 { dot(rows[0], vec), dot(rows[1], vec), dot(rows[2], vec) };
}

mat4 identity()
{
    return {
        vec4 { 1.0f, 0.0f, 0.0f, 0.0f },
        vec4 { 0.0f, 1.0f, 0.0f, 0.0f },
        vec4 { 0.0f, 0.0f, 1.0f, 0.0f },
        vec4 { 0.0f, 0.0f, 0.0f, 1.0f },
    };
}

// AABB

struct Aabb {
    vec3 min, max;
};

bool overlap(const Aabb& a, const Aabb& b)
{
    return a.min.x <= b.max.x && a.min.y <= b.max.y && a.min.z <= b.max.z && a.max.x >= b.min.x
        && a.max.y >= b.min.y && a.max.z >= b.min.z;
}

bool contains(const Aabb& aabb, const vec3_view& p)
{
    return p[x] >= aabb.min.x && p[y] >= aabb.min.y && p[z] >= aabb.min.z && p[x] <= aabb.max.x
        && p[y] <= aabb.max.y && p[z] <= aabb.max.z;
}

Aabb combine(const Aabb& a, const Aabb& b)
{
    return Aabb { vmin(a.min, b.min), vmax(a.max, b.max) };
}

[[maybe_unused]] float volume(const Aabb& aabb)
{
    const auto s = sub(aabb.max, aabb.min);
    return s.x * s.y * s.z;
}

// TODO: Check how this is used and change interface
bool ray_cast(const Aabb& aabb, vec3_view start, vec3_view dir, wuzy_ray_cast_result* res)
{
    // Real-Time Collision Detection, 5.3.3
    // This could be made much faster if we extend the ray data:
    // https://knork.org/fast-AABB-test.html
    static const vec3 axes[3] = {
        { 1.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f },
    };
    const vec3 ood = { 1.0f / dir[x], 1.0f / dir[y], 1.0f / dir[z] };

    float tmin = 0.0f;
    float tmax = FLT_MAX;
    for (size_t axis = 0; axis < 3; ++axis) {
        if (std::abs(dir[axis]) < FLT_EPSILON) {
            // Ray is parallel to slab. No hit if origin not within slab.
            if (start[axis] < aabb.min[axis] || start[axis] > aabb.max[axis]) {
                return false;
            }
        } else {
            // Compute intersection t value of ray with near and far plane of slab
            float normal_sign = 1.0f;
            float t1 = (aabb.min[axis] - start[axis]) * ood[axis];
            float t2 = (aabb.max[axis] - start[axis]) * ood[axis];
            // Make t1 be intersection with near plane t2 with far plane
            if (t1 > t2) {
                std::swap(t1, t2);
                normal_sign = -1.0f;
            }
            if (t1 > tmin) {
                tmin = t1;
                copy(res->normal, mul(axes[axis], normal_sign));
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
    copy(res->hit_position, add(start, mul(dir, res->t)));
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

EXPORT void wuzy_invert_trs(const float mat[16], float inv[16])
{
    // inv(TRS) = inv(S) * inv(R) * inv(T) = inv(S) * transpose(R) * inv(T)
    const auto m = m4(mat);
    const auto sx = length(m[0]);
    const auto sy = length(m[1]);
    const auto sz = length(m[2]);

    // To get the rotation matrix, we take the 3x3 upper left block and
    // divide out the scale matrix.
    const mat4 r = {
        mul(m[0], 1.0f / sx),
        mul(m[1], 1.0f / sy),
        mul(m[2], 1.0f / sz),
        vec4 { 0.0f, 0.0f, 0.0f, 1.0f },
    };

    // Because the rotation matrix is orthogonal, its inverse is its transpose.
    const auto inv_r = transpose(r);

    // We just build a scale matrix with the inverse scale factors
    auto inv_s = identity();
    inv_s.cols[0].x = 1.0f / sx;
    inv_s.cols[1].y = 1.0f / sy;
    inv_s.cols[2].z = 1.0f / sz;

    auto inv_t = identity();
    inv_t.cols[3] = mul(m[3], -1.0f);

    copy(inv, mul(mul(inv_s, inv_r), inv_t));
}

EXPORT void wuzy_collider_set_transform(wuzy_collider* collider, const float transform[16])
{
    copy(collider->transform, m4(transform));
    wuzy_invert_trs(transform, collider->inverse_transform);
}

static vec3 collider_support(const wuzy_collider* collider, vec3_view dir)
{
    const auto local_dir = mul(m4(collider->inverse_transform), dir, 0.0f);
    vec3 local_sup;
    collider->support_func(collider->userdata, local_dir.data(), local_sup.data());
    // Transform support point back to world space
    return mul(m4(collider->transform), local_sup, 1.0f);
}

EXPORT void wuzy_collider_support(const wuzy_collider* collider, const float dir[3], float sup[3])
{
    copy(sup, collider_support(collider, v3(dir)));
}

EXPORT bool wuzy_collider_ray_cast(const wuzy_collider* collider, const float start[3],
    const float dir[3], wuzy_ray_cast_result* result)
{
    const auto inv_trafo = m4(collider->inverse_transform);
    const auto start_local = mul(inv_trafo, v3(start), 1.0f);
    const auto dir_local = mul(inv_trafo, v3(dir), 0.0f);

    // Ray cast in local space
    wuzy_ray_cast_result local_result;
    if (!collider->ray_cast_func(
            collider->userdata, start_local.data(), dir_local.data(), &local_result)) {
        return false;
    }

    // Transform results back to world space
    const auto trafo = m4(collider->transform);
    copy(result->hit_position, mul(trafo, v3(local_result.hit_position), 1.0f));
    copy(result->normal, normalize(mul(trafo, v3(local_result.normal), 0.0f)));
    result->t = local_result.t;
    return true;
}

static Aabb collider_get_aabb(const wuzy_collider* collider)
{
    // http://allenchou.net/2014/02/game-physics-updating-aabbs-for-polyhedrons/
    Aabb aabb;
    // TODO: Check whether this is optimized and if not, do it myself.
    // By passing base vectors, we are essentially using columns of the inverse transform for the
    // local direction.
    aabb.min.x = collider_support(collider, vec3 { -1.0f, 0.0f, 0.0f }).x;
    aabb.min.y = collider_support(collider, vec3 { 0.0f, -1.0f, 0.0f }).y;
    aabb.min.z = collider_support(collider, vec3 { 0.0f, 0.0f, -1.0f }).z;
    aabb.max.x = collider_support(collider, vec3 { 1.0f, 0.0f, 0.0f }).x;
    aabb.max.y = collider_support(collider, vec3 { 0.0f, 1.0f, 0.0f }).y;
    aabb.max.z = collider_support(collider, vec3 { 0.0f, 0.0f, 1.0f }).z;
    return aabb;
}

EXPORT void wuzy_collider_get_aabb(const wuzy_collider* collider, float min[3], float max[3])
{
    const auto aabb = collider_get_aabb(collider);
    copy(min, aabb.min);
    copy(max, aabb.max);
}

static void calculate_normal(
    float normal[3], const float v0[3], const float v1[3], const float v2[3])
{
    copy(normal, normalize(cross(sub(v3(v1), v3(v0)), sub(v3(v2), v3(v0)))));
}

EXPORT bool wuzy_calculate_normals(const float* vertices, size_t num_vertices,
    const size_t* face_indices, size_t num_faces, float* normals)
{
    for (size_t i = 0; i < num_faces; i++) {
        size_t i0 = face_indices[i * 3];
        size_t i1 = face_indices[i * 3 + 1];
        size_t i2 = face_indices[i * 3 + 2];

        if (i0 >= num_vertices || i1 >= num_vertices || i2 >= num_vertices) {
            return false;
        }

        calculate_normal(normals + i * 3, vertices + i0 * 3, vertices + i1 * 3, vertices + i2 * 3);
    }

    return true;
}

EXPORT void wuzy_triangle_collider_init(
    wuzy_collider* collider, wuzy_triangle_collider_userdata* userdata)
{
    assert(is_finite(v3(userdata->vertices[0])));
    assert(is_finite(v3(userdata->vertices[1])));
    assert(is_finite(v3(userdata->vertices[2])));

    copy(collider->transform, identity());
    copy(collider->inverse_transform, identity());
    collider->userdata = userdata;
    collider->support_func = wuzy_triangle_collider_support;
    collider->ray_cast_func = wuzy_triangle_collider_ray_cast;
    calculate_normal(
        userdata->normal, userdata->vertices[0], userdata->vertices[1], userdata->vertices[2]);
}

EXPORT void wuzy_triangle_collider_support(const void* userdata, const float dir[3], float sup[3])
{
    const auto& tri = *static_cast<const wuzy_triangle_collider_userdata*>(userdata);
    float max_dot = -FLT_MAX;
    for (int i = 0; i < 3; i++) {
        float d = dot(v3(tri.vertices[i]), v3(dir));
        if (d > max_dot) {
            max_dot = d;
            copy(sup, v3(tri.vertices[i]));
        }
    }
}

EXPORT bool wuzy_triangle_collider_ray_cast(
    const void* userdata, const float start[3], const float dir[3], wuzy_ray_cast_result* result)
{
    // Real-Time Collision Detection, 5.3.6
    // This is essentially Möller-Trumbore, but with using the scalar triple product identify a few
    // times and pre-calculating the normal.
    const auto& tri = *static_cast<const wuzy_triangle_collider_userdata*>(userdata);

    // Check if ray points in direction of triangle normal (or is coplanar)
    const auto d = -dot(v3(tri.normal), v3(dir));
    if (d <= FLT_EPSILON) {
        return false;
    }

    // Check which half space the ray origin is in
    const auto s = sub(v3(start), v3(tri.vertices[0]));
    const auto t = dot(v3(tri.normal), s);
    if (t < 0.0f) {
        return false;
    }

    // Compute barycentric coordinates of intersection
    const auto e = cross(v3(dir), s);
    const auto v = -dot(sub(v3(tri.vertices[2]), v3(tri.vertices[0])), e);
    if (v < 0.0f || v > d) {
        return false;
    }

    const auto w = dot(sub(v3(tri.vertices[1]), v3(tri.vertices[0])), e);
    if (w < 0.0f || v + w > d) {
        return false;
    }

    result->t = t / d;
    copy(result->normal, normalize(v3(tri.normal))); // tri.normal is not normalized!
    copy(result->hit_position, add(v3(start), mul(v3(dir), result->t)));
    return true;
}

// Sphere collider
EXPORT void wuzy_sphere_collider_init(
    wuzy_collider* collider, wuzy_sphere_collider_userdata* userdata)
{
    copy(collider->transform, identity());
    copy(collider->inverse_transform, identity());
    collider->userdata = userdata;
    collider->support_func = wuzy_sphere_collider_support;
    collider->ray_cast_func = wuzy_sphere_collider_ray_cast;
}

EXPORT void wuzy_sphere_collider_support(const void* userdata, const float dir[3], float sup[3])
{
    const auto& sphere = *static_cast<const wuzy_sphere_collider_userdata*>(userdata);
    copy(sup, mul(normalize(v3(dir)), sphere.radius));
}

EXPORT bool wuzy_sphere_collider_ray_cast(
    const void* userdata, const float start[3], const float dir[3], wuzy_ray_cast_result* result)
{
    const auto& sphere = *reinterpret_cast<const wuzy_sphere_collider_userdata*>(userdata);

    // Real-Time Collision Detection, 5.3.2
    const auto b = dot(v3(start), v3(dir));
    const auto c = dot(v3(start), v3(start)) - sphere.radius * sphere.radius;
    // ray starts outside sphere and points away from it or ray starts inside the sphere
    if ((c > 0.0f && b > 0.0f) || c < 0.0f) {
        return false;
    }

    const auto discr = b * b - c;
    if (discr < 0.0f) {
        return false;
    }

    result->t = std::max(0.0f, -b - std::sqrt(discr));
    copy(result->hit_position, add(v3(start), mul(v3(dir), result->t)));
    copy(result->normal, normalize(v3(result->hit_position)));
    return true;
}

// Convex polyhedron collider
EXPORT void wuzy_convex_polyhedron_collider_init(
    wuzy_collider* collider, wuzy_convex_polyhedron_collider_userdata* userdata)
{
    wuzy_calculate_normals(userdata->vertices, userdata->num_vertices, userdata->face_indices,
        userdata->num_faces, userdata->normals);
    copy(collider->transform, identity());
    copy(collider->inverse_transform, identity());
    collider->userdata = userdata;
    collider->support_func = wuzy_convex_polyhedron_collider_support;
    collider->ray_cast_func = wuzy_convex_polyhedron_collider_ray_cast;
}

EXPORT void wuzy_convex_polyhedron_collider_support(
    const void* userdata, const float dir[3], float sup[3])
{
    const auto& poly = *static_cast<const wuzy_convex_polyhedron_collider_userdata*>(userdata);
    assert(poly.num_vertices > 0);
    float max_dot = -FLT_MAX;
    for (size_t i = 0; i < poly.num_vertices; i++) {
        const auto v = v3(poly.vertices + i * 3);
        float d = dot(v, v3(dir));
        if (d > max_dot) {
            max_dot = d;
            copy(sup, v);
        }
    }
}

EXPORT bool wuzy_convex_polyhedron_collider_ray_cast(
    const void* userdata, const float start[3], const float dir[3], wuzy_ray_cast_result* result)
{
    const auto& poly = *reinterpret_cast<const wuzy_convex_polyhedron_collider_userdata*>(userdata);

    // Real-Time Collision Detection, 5.3.8
    float tfirst = 0.0f;
    float tlast = std::numeric_limits<float>::max();
    // Check ray against each half-space defined by each face
    for (size_t f = 0; f < poly.num_faces; ++f) {
        const auto n = v3(poly.normals + f * 3);
        const auto i0 = poly.face_indices[f * 3 + 0];
        const auto v0 = v3(poly.vertices + i0 * 3);
        const auto d = dot(v0, n);

        const auto denom = dot(n, v3(dir));
        const auto dist = d - dot(n, v3(start));
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
                    copy(result->normal, n);
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
    copy(result->hit_position, add(v3(start), mul(v3(dir), result->t)));
    return true;
}

namespace {
// Support function of the minkowski difference `a - b`.
vec3 support(const wuzy_collider* a, const wuzy_collider* b, vec3_view dir)
{
    const auto a_sup = collider_support(a, dir);
    const auto b_sup = collider_support(b, mul(dir, -1.0f));
    return sub(a_sup, b_sup);
}

bool same_half_space(vec3_view a, vec3_view b)
{
    return dot(a, b) > 0.0f;
}

wuzy_simplex3d make_simplex(vec3_view v0)
{
    wuzy_simplex3d simplex;
    copy(simplex.vertices[0], v0);
    simplex.num_vertices = 1;
    return simplex;
}

wuzy_simplex3d make_simplex(vec3_view v0, vec3_view v1)
{
    wuzy_simplex3d simplex;
    copy(simplex.vertices[0], v0);
    copy(simplex.vertices[1], v1);
    simplex.num_vertices = 2;
    return simplex;
}

wuzy_simplex3d make_simplex(vec3_view v0, vec3_view v1, vec3_view v2)
{
    wuzy_simplex3d simplex;
    copy(simplex.vertices[0], v0);
    copy(simplex.vertices[1], v1);
    copy(simplex.vertices[2], v2);
    simplex.num_vertices = 3;
    return simplex;
}

struct NextSimplexResult {
    wuzy_simplex3d simplex;
    vec3 direction;
    bool contains_origin;

    NextSimplexResult(const wuzy_simplex3d& s, vec3_view dir, bool orig = false)
        : simplex(s)
        , contains_origin(orig)
    {
        copy(&direction.x, dir);
    }

    NextSimplexResult(vec3_view v0, vec3_view dir)
        : simplex(make_simplex(v0))
        , contains_origin(false)
    {
        copy(&direction.x, dir);
    }

    NextSimplexResult(vec3_view v0, vec3_view v1, vec3_view dir)
        : simplex(make_simplex(v0, v1))
        , contains_origin(false)
    {
        copy(&direction.x, dir);
    }

    NextSimplexResult(vec3_view v0, vec3_view v1, vec3_view v2, vec3_view dir)
        : simplex(make_simplex(v0, v1, v2))
        , contains_origin(false)
    {
        copy(&direction.x, dir);
    }
};

NextSimplexResult line(const wuzy_simplex3d& simplex, vec3_view /*direction*/)
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
    const auto a = v3(simplex.vertices[0]);
    const auto b = v3(simplex.vertices[1]);
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
            dir = std::abs(dir.x) >= 0.57735f ? vec3 { dir.y, -dir.x, 0.0f }
                                              : vec3 { 0.0f, dir.z, -dir.y };
        }
        assert(length(dir) > FLT_EPSILON);
        return { simplex, dir };
    } else {
        // <2>
        // If ao and ab are not in the same half-space, the origin is "after"/"beyond" a (<2>).
        // This means that the origin is not in the direction of b and it will not help us
        // enclose the origin, so we throw it away.
        // We keep only a (because it more towards the origin) and search towards the origin
        // again.
        return { a, ao };
    }
}

NextSimplexResult triangle(const wuzy_simplex3d& simplex, vec3_view direction)
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
    const auto a = v3(simplex.vertices[0]);
    const auto b = v3(simplex.vertices[1]);
    const auto c = v3(simplex.vertices[2]);

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
            return { a, c, dir };
        } else {
            // <5>
            return line(make_simplex(a, b), direction);
        }
    }

    // `ab.cross(abc)` is the normal of the plane that contains a and b, so we are checking
    // for region <4>.
    if (same_half_space(cross(ab, abc), ao)) {
        // <4>
        return line(make_simplex(a, b), direction);
    }

    // <2> or <3> are left
    assert(is_finite(abc) && length(abc) > FLT_EPSILON);
    if (same_half_space(abc, ao)) {
        // "above" the triangle
        return { simplex, abc, false };
    } else {
        // "below" the triangle
        // "rewind" the triangle to point "down" instead
        return { make_simplex(a, c, b), mul(abc, -1.0f) };
    }
}

NextSimplexResult tetrahedron(const wuzy_simplex3d& simplex, vec3_view direction)
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
    const auto a = v3(simplex.vertices[0]);
    const auto b = v3(simplex.vertices[1]);
    const auto c = v3(simplex.vertices[2]);
    const auto d = v3(simplex.vertices[3]);

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
        return triangle(make_simplex(a, b, c), direction);
    }
    if (same_half_space(acd, ao)) {
        return triangle(make_simplex(a, c, d), direction);
    }
    if (same_half_space(adb, ao)) {
        return triangle(make_simplex(a, d, b), direction);
    }

    return { simplex, direction, true };
}

NextSimplexResult next_simplex(const wuzy_simplex3d& simplex, vec3_view direction)
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

void push_front(wuzy_simplex3d& simplex, vec3_view v)
{
    copy(simplex.vertices[3], v3(simplex.vertices[2]));
    copy(simplex.vertices[2], v3(simplex.vertices[1]));
    copy(simplex.vertices[1], v3(simplex.vertices[0]));
    copy(simplex.vertices[0], v);
    assert(simplex.num_vertices < 4);
    simplex.num_vertices++;
}

void add_debug_iteration(wuzy_gjk_debug* debug, const wuzy_collider* c1, const wuzy_collider* c2,
    const vec3& direction, const vec3& support)
{
    if (!debug) {
        return;
    }

    // Yes, we are allocating a storm in here
    if (!debug->iterations) {
        debug->iterations = allocate<wuzy_gjk_debug_iteration>(debug->alloc, 1);
        debug->num_iterations = 1;
    } else {
        debug->iterations = reallocate<wuzy_gjk_debug_iteration>(
            debug->alloc, debug->iterations, debug->num_iterations, debug->num_iterations + 1);
        debug->num_iterations++;
    }

    const auto a_sup = collider_support(c1, direction);
    const auto b_sup = collider_support(c2, mul(direction, -1.0f));
    debug->iterations[debug->num_iterations - 1] = {
        .direction = { direction.x, direction.y, direction.z },
        .a_support = { a_sup.x, a_sup.y, a_sup.z },
        .b_support = { b_sup.x, b_sup.y, b_sup.z },
        .support = { support.x, support.y, support.z },
        .simplex = {},
        .contains_origin = false,
    };
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
    vec3 direction = { 1.0f, 0.0f, 0.0f };

    const auto a0 = support(c1, c2, direction);

    // TODO: Handle a0 == origin

    auto& simplex = *result;
    copy(simplex.vertices[0], a0);
    simplex.num_vertices = 1;

    add_debug_iteration(debug, c1, c2, direction, a0);
    if (debug) {
        debug->iterations[debug->num_iterations - 1].simplex = simplex;
    }

    // Choose dir towards the origin: a0 -> O = O - a0 = -a0.
    direction = mul(a0, -1.0f);

    size_t num_iterations = 0;
    const auto debug_max_iterations
        = debug && debug->max_num_iterations ? debug->max_num_iterations : 64;
    while (num_iterations++ < debug_max_iterations) {
        // direction doesn't have to be normalized, but it can't be a null-vector
        assert(length(direction) > FLT_EPSILON);
        assert(is_finite(direction));
        const auto a = support(c1, c2, direction);
        assert(is_finite(a));

        add_debug_iteration(debug, c1, c1, direction, a);

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
            assert(is_finite(v3(res.simplex.vertices[i])));
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
    const auto a_aabb = collider_get_aabb(a);
    const auto b_aabb = collider_get_aabb(b);
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
    vec3 normal = { 0.0f, 0.0f, 0.0f };
    float dist = 0.0f; // Distance to origin
};

void update_normal(std::span<const vec3> vertices, EpaTriangle& face, bool flip = true)
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
    std::array<vec3, 4 + max_num_iterations> polytope_vertices_data;
    VecAdapter<vec3> polytope_vertices(polytope_vertices_data);
    for (size_t i = 0; i < 4; ++i) {
        assert(is_finite(v3(simplex->vertices[i])));
        polytope_vertices.push_back(make_vec3(simplex->vertices[i]));
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
            it.polytope_vertices = allocate<float>(debug->alloc, it.num_polytope_vertices * 3);
            std::memcpy(it.polytope_vertices, polytope_vertices.data(),
                polytope_vertices.size() * sizeof(vec3));

            it.num_polytope_faces = polytope_faces.size();
            it.polytope_faces = allocate<wuzy_epa_debug_face>(debug->alloc, it.num_polytope_faces);
            std::memcpy(it.polytope_faces, polytope_faces.data(),
                polytope_faces.size() * sizeof(EpaTriangle));

            it.face_removed = allocate<bool>(debug->alloc, it.num_polytope_faces);
            std::memset(it.face_removed, 0, sizeof(bool) * it.num_polytope_faces);

            it.min_dist_face_index = min_dist_face_idx;
            it.min_face_dist = min_face_dist;
            copy(it.support_point, sup_point);
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

    wuzy_collision_result res;
    copy(res.normal, polytope_faces[min_dist_face_idx].normal);
    // Add some epsilon to make sure we resolve the collision
    res.depth = min_face_dist + 1e-4f;
    return res;
}

EXPORT void wuzy_epa_debug_free(wuzy_epa_debug* debug)
{
    for (size_t i = 0; i < debug->num_iterations; ++i) {
        const auto& it = debug->iterations[i];
        deallocate(debug->alloc, it.polytope_vertices, it.num_polytope_vertices * 3);
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
    const auto a_aabb = collider_get_aabb(a);
    const auto b_aabb = collider_get_aabb(b);
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
// To change the Id layout, only the following typedefs and three functions should be changed.
using IdType = decltype(wuzy_aabb_tree_node::id);
using GenerationType = uint16_t;
using IndexType = uint32_t;

IndexType id_get_idx(IdType id)
{
    return static_cast<IndexType>(id & 0xFFFFFFFF);
}

GenerationType id_get_gen(IdType id)
{
    return static_cast<GenerationType>(id >> 32);
}

IdType id_combine(uint32_t idx, uint32_t gen)
{
    return static_cast<IdType>(gen) << 32 | idx;
}

struct Node {
    GenerationType generation = 1;

    wuzy_collider* collider = nullptr;
    Aabb aabb;
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

    void refit()
    {
        for (auto p = parent; p; p = p->parent) {
            p->update_from_children();
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

    IdType get_id(Node* node) const
    {
        const auto idx = static_cast<uint32_t>(node - nodes);
        return id_combine(idx, nodes[idx].generation);
    }

    Node* get_node(IdType id) const
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
        node->aabb = collider_get_aabb(init_node->collider);
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

EXPORT void wuzy_aabb_tree_build(wuzy_aabb_tree* wtree, wuzy_collider* const* colliders,
    const uint64_t* bitmasks, size_t num_colliders, wuzy_aabb_tree_node* nodes)
{
    // Just add a bunch of collider nodes, just enough so that rebuild will work
    auto tree = reinterpret_cast<AabbTree*>(wtree);
    assert(tree->num_nodes == 0);
    assert(num_colliders <= tree->max_num_colliders);
    for (size_t i = 0; i < num_colliders; ++i) {
        auto node = tree->get_new_node();
        assert(node);
        node->collider = colliders[i];
        node->aabb = collider_get_aabb(node->collider);
        node->bitmask = (bitmasks && bitmasks[i]) ? bitmasks[i] : static_cast<uint64_t>(-1);
        nodes[i].id = tree->get_id(node);
        tree->num_colliders++;
    }

    wuzy_aabb_tree_rebuild(wtree);
}

EXPORT void wuzy_aabb_tree_rebuild(wuzy_aabb_tree* wtree)
{
    // This rebuilds the tree bottom-up.
    // I tried top-down as well, but using many different strategies (lowest total volume, lowest
    // total surface area, median split) I could not get a tree that was better than the one you get
    // by simply inserting in a loop.
    // lowst total volume and lowest total surface area (SAH) have the problem that with large
    // polygons (roughly the size of the overall AABB), you cannot split without getting a larger
    // cost (because you get overlap). In that case you often get just a single node split off and
    // you end up with (roughly) a linear spine. Also triangles are often axis aligned (walls,
    // floors, ceilings) so that their AABB has volume 0, which messes up everything and gives bad
    // results as well.
    // When you use a median split you get a tree that looks very nice on paper (perfectly
    // balanced), but the two branches at each node are not visited with equal probability, so you
    // in fact end up with trees that require more checks than the default (repeated insertion)
    // tree.
    // This bottom-up approach is the only one I found that actually produces better trees (about
    // 30% fewer checks in my tests).

    // TODO: Some optimization ideas for the future: We could use tree->nodes instead of a nodes
    // vector then we know we don't have gaps and we don't have a pointer indirection, but we cannot
    // move elements around and we need to have a check like `if(!node->parent)` to determine if a
    // node is in the current working set inside the loop that calculates the cost. It's hard to
    // tell whether this will improve things.

    auto tree = reinterpret_cast<AabbTree*>(wtree);
    std::vector<Node*> nodes;
    nodes.reserve(tree->num_nodes);
    for (size_t i = 0; i < tree->num_nodes; ++i) {
        if (tree->nodes[i].is_leaf()) {
            nodes.push_back(&tree->nodes[i]);
        } else {
            // free all internal nodes
            tree->free_node(&tree->nodes[i]);
        }
    }

    while (nodes.size() > 1) {
        // We look at all the nodes and find the two that are the best to merge and then put that
        // parent into the nodes to merge and remove the children. We repeat until a single node is
        // left.

        // It would be nice to not repeat these calculations for every pair all the time, but
        // storing these is not that simple.
        // I tried using a min-heap for this, but you need to do a lot of book keeping to remove the
        // pairs (or to skip them, I tried both). Just popping an element can be expensive if the
        // heap is large.
        // In the end using a heap here is much slower (~2x) for a tree with 500 tris.
        // Of course usually levels are much larger and the heap should improve performance if N is
        // large, but since you need to store a heap of *pairs* of nodes, you have another problem.
        // With just 10k tris you need 2G of memory just for the heap. And with 100k tris
        // (reasonable and not even that large), you need 200G, which is impossible on most
        // machines. So this naive implementation is actually the best I can do right now.
        size_t best_i = 0;
        size_t best_j = 1;
        float best_cost = FLT_MAX;
        for (size_t i = 0; i < nodes.size(); ++i) {
            for (size_t j = i + 1; j < nodes.size(); ++j) {
                // TODO: Reconsider `area` here, it seems to yield better trees sometimes.
                const auto cost = volume(combine(nodes[i]->aabb, nodes[j]->aabb));
                if (cost < best_cost) {
                    best_cost = cost;
                    best_i = i;
                    best_j = j;
                }
            }
        }

        auto parent = tree->get_new_node();
        parent->left = nodes[best_i];
        nodes[best_i]->parent = parent;
        parent->right = nodes[best_j];
        nodes[best_j]->parent = parent;
        parent->update_from_children();

        assert(best_j > best_i);
        // j might be last, so we start with it
        std::swap(nodes[best_j], nodes[nodes.size() - 1]);
        nodes.pop_back();
        std::swap(nodes[best_i], nodes[nodes.size() - 1]);
        nodes.pop_back();
        nodes.push_back(parent);
    }
    tree->root = nodes[0];
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
    node->aabb = collider_get_aabb(collider);
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

EXPORT bool wuzy_aabb_tree_update(wuzy_aabb_tree* wtree, wuzy_aabb_tree_node wnode,
    uint64_t bitmask, wuzy_aabb_tree_update_mode mode)
{
    auto tree = reinterpret_cast<AabbTree*>(wtree);
    auto node = tree->get_node(wnode.id);
    if (!node) {
        return false;
    }
    node->aabb = collider_get_aabb(node->collider);
    node->bitmask = bitmask ? bitmask : node->bitmask;
    switch (mode) {
    case WUZY_AABB_TREE_UPDATE_FLAGS_DEFAULT:
    case WUZY_AABB_TREE_UPDATE_FLAGS_REINSERT:
        tree->remove(node);
        tree->insert(node, tree->root); // We ignore the return, because it should have worked
        return true;
    case WUZY_AABB_TREE_UPDATE_FLAGS_REFIT:
        node->refit();
        return true;
    default:
        return false;
    }
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
    copy(nodes[idx].aabb_min, node->aabb.min);
    copy(nodes[idx].aabb_max, node->aabb.max);
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
        vec3 point;
        Aabb aabb;
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

EXPORT void wuzy_aabb_tree_node_query_point_begin(wuzy_aabb_tree_node_query* wquery,
    const float point[3], uint64_t bitmask, wuzy_query_debug* debug)
{
    auto query = reinterpret_cast<NodeQuery*>(wquery);
    query->bitmask = bitmask ? bitmask : static_cast<uint64_t>(-1);
    query->type = NodeQuery::Type::Point;
    query->params.point = { point[x], point[y], point[z] };
    query->debug = debug;
    query->node_stack_size = 0;
    if (query->tree->root) {
        query->node_stack[query->node_stack_size++] = query->tree->root;
    }
}

EXPORT void wuzy_aabb_tree_node_query_aabb_begin(wuzy_aabb_tree_node_query* wquery,
    const float aabb_min[3], const float aabb_max[3], uint64_t bitmask, wuzy_query_debug* debug)
{
    auto query = reinterpret_cast<NodeQuery*>(wquery);
    query->bitmask = bitmask ? bitmask : static_cast<uint64_t>(-1);
    query->type = NodeQuery::Type::Aabb;
    query->params.aabb.min = { aabb_min[x], aabb_min[y], aabb_min[z] };
    query->params.aabb.max = { aabb_max[x], aabb_max[y], aabb_max[z] };
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

EXPORT bool wuzy_aabb_tree_node_query_ray_cast(wuzy_aabb_tree_node_query* wquery,
    const float start[3], const float dir[3], uint64_t bitmask, wuzy_aabb_tree_node* hit_node,
    wuzy_ray_cast_result* result, wuzy_query_debug* debug)
{
    bitmask = bitmask ? bitmask : static_cast<uint64_t>(-1);
    auto query = reinterpret_cast<NodeQuery*>(wquery);
    query->node_stack_size = 0;
    if (query->tree->root) {
        query->node_stack[query->node_stack_size++] = query->tree->root;
    }

    bool hit = false;
    hit_node->id = 0;
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

        const auto aabb_hit = ray_cast(node->aabb, v3(start), v3(dir), &temp_res);
        if (aabb_hit && (!hit || temp_res.t < result->t)) {
            if (debug) {
                debug->aabb_checks_passed++;
            }
            if (node->is_leaf()) {
                if (debug) {
                    debug->leaves_checked++;
                }
                const auto collider_hit
                    = wuzy_collider_ray_cast(node->collider, start, dir, &temp_res);
                if (collider_hit && (!hit || temp_res.t < result->t)) {
                    if (debug) {
                        debug->full_checks_passed++;
                    }
                    hit_node->id = query->tree->get_id(node);
                    *result = temp_res;
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