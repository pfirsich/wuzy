#include "wuzy/wuzy.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include <iostream> // DEBUG

using namespace wuzy::detail;

namespace wuzy {
std::string toString(const Vec3& v)
{
    char buf[64];
    std::sprintf(buf, "{%f, %f, %f}", v.x, v.y, v.z);
    return buf;
}

float Vec3::dot(const Vec3& other) const
{
    return x * other.x + y * other.y + z * other.z;
}

Vec3 Vec3::cross(const Vec3& other) const
{
    return Vec3 {
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x,
    };
}

float Vec3::length() const
{
    return std::sqrt(dot(*this));
}

Vec3 Vec3::normalized() const
{
    return *this / length();
}

float Vec3::operator[](size_t i) const
{
    assert(i < 3);
    switch (i) {
    default:
    case 0:
        return x;
    case 1:
        return y;
    case 2:
        return z;
    }
}

Vec3 Vec3::operator-() const
{
    return Vec3 { -x, -y, -z };
}

Vec3 Vec3::operator-(const Vec3& other) const
{
    return Vec3 { x - other.x, y - other.y, z - other.z };
}

Vec3 Vec3::operator+(const Vec3& other) const
{
    return Vec3 { x + other.x, y + other.y, z + other.z };
}

Vec3 Vec3::operator*(float s) const
{
    return Vec3 { s * x, s * y, s * z };
}

Vec3 Vec3::operator/(float s) const
{
    return Vec3 { x / s, y / s, z / s };
}

bool Vec3::operator==(const Vec3& other) const
{
    return x == other.x && y == other.y && z == other.z;
}

bool Vec3::operator!=(const Vec3& other) const
{
    return !(*this == other);
}

Vec3 xAxis { 1.0f, 0.0f, 0.0f };
Vec3 yAxis { 0.0f, 1.0f, 0.0f };
Vec3 zAxis { 0.0f, 0.0f, 1.0f };

Vec4::Vec4(float x, float y, float z, float w)
    : x(x)
    , y(y)
    , z(z)
    , w(w)
{
}

Vec4::Vec4(const Vec3& v, float w)
    : x(v.x)
    , y(v.y)
    , z(v.z)
    , w(w)
{
}

float Vec4::dot(const Vec4& other) const
{
    return x * other.x + y * other.y + z * other.z + w * other.w;
}

float Vec4::length() const
{
    return std::sqrt(dot(*this));
}

Vec4 Vec4::operator/(float s) const
{
    return Vec4 { x / s, y / s, z / s, w / s };
}

Vec4::operator Vec3() const
{
    return Vec3 { x, y, z };
}

Mat4 Mat4::transpose() const
{
    return {
        Vec4 { cols[0].x, cols[1].x, cols[2].x, cols[3].x },
        Vec4 { cols[0].y, cols[1].y, cols[2].y, cols[3].y },
        Vec4 { cols[0].z, cols[1].z, cols[2].z, cols[3].z },
        Vec4 { cols[0].w, cols[1].w, cols[2].w, cols[3].w },
    };
}

Mat4 Mat4::operator*(const Mat4& m) const
{
    const auto r = transpose().cols;
    return Mat4 {
        Vec4 { r[0].dot(m.cols[0]), r[1].dot(m.cols[0]), r[2].dot(m.cols[0]), r[3].dot(m.cols[0]) },
        Vec4 { r[0].dot(m.cols[1]), r[1].dot(m.cols[1]), r[2].dot(m.cols[1]), r[3].dot(m.cols[1]) },
        Vec4 { r[0].dot(m.cols[2]), r[1].dot(m.cols[2]), r[2].dot(m.cols[2]), r[3].dot(m.cols[2]) },
        Vec4 { r[0].dot(m.cols[3]), r[1].dot(m.cols[3]), r[2].dot(m.cols[3]), r[3].dot(m.cols[3]) },
    };
}

Vec4 Mat4::operator*(const Vec4& v) const
{
    const auto rows = transpose().cols;
    return Vec4 { rows[0].dot(v), rows[1].dot(v), rows[2].dot(v), rows[3].dot(v) };
}

Mat4 Mat4::translate(const Vec3& v)
{
    return {
        Vec4 { 1.0f, 0.0f, 0.0f, 0.0f },
        Vec4 { 0.0f, 1.0f, 0.0f, 0.0f },
        Vec4 { 0.0f, 0.0f, 1.0f, 0.0f },
        Vec4 { v.x, v.y, v.z, 1.0f },
    };
}

Mat4 Mat4::scale(const Vec3& v)
{
    return {
        Vec4 { v.x, 0.0f, 0.0f, 0.0f },
        Vec4 { 0.0f, v.y, 0.0f, 0.0f },
        Vec4 { 0.0f, 0.0f, v.z, 0.0f },
        Vec4 { 0.0f, 0.0f, 0.0f, 1.0f },
    };
}

Vec3 Aabb::center() const
{
    return (min + max) * 0.5f;
}

Vec3 Aabb::extent() const
{
    return size() * 0.5f;
}

Vec3 Aabb::size() const
{
    return max - min;
}

float Aabb::area() const
{
    const auto s = size();
    return 2.0f * (s.x * s.y + s.x * s.z + s.x * s.z);
}

float Aabb::volume() const
{
    const auto s = size();
    return s.x * s.y * s.z;
}

bool Aabb::contains(const Vec3& p) const
{
    return p.x >= min.x && p.y >= min.y && p.z >= min.z && p.x <= max.x && p.y <= max.y
        && p.z <= max.z;
}

bool Aabb::overlaps(const Aabb& b) const
{
    return min.x <= b.max.x && min.y <= b.max.y && min.z <= b.max.z && max.x >= b.min.x
        && max.y >= b.min.y && max.z >= b.min.z;
}

std::optional<RayCastResult> Aabb::rayCast(const Vec3& position, const Vec3& direction) const
{
    // Real-Time Collision Detection, 5.3.3
    // This could be made much faster if we extend the ray data:
    // https://knork.org/fast-AABB-test.html
    static const Vec3 axes[3] { xAxis, yAxis, zAxis };
    const Vec3 ood { 1.0f / direction.x, 1.0f / direction.y, 1.0f / direction.z };

    float tmin = 0.0f;
    float tmax = std::numeric_limits<float>::max();
    Vec3 normal;
    for (size_t axis = 0; axis < 3; ++axis) {
        if (std::abs(direction[axis]) < std::numeric_limits<float>::epsilon()) {
            // Ray is parallel to slab. No hit if origin not within slab.
            if (position[axis] < min[axis] || position[axis] > max[axis]) {
                return std::nullopt;
            }
        } else {
            // Compute intersection t value of ray with near and far plane of slab
            float normalSign = 1.0f;
            float t1 = (min[axis] - position[axis]) * ood[axis];
            float t2 = (max[axis] - position[axis]) * ood[axis];
            // Make t1 be intersection with near plane t2 with far plane
            if (t1 > t2) {
                std::swap(t1, t2);
                normalSign = -1.0f;
            }
            if (t1 > tmin) {
                tmin = t1;
                normal = axes[axis] * normalSign;
            }
            if (t2 < tmax) {
                tmax = t2;
            }
            // Exit with no collision as soon as slab intersection becomes empty
            if (tmin > tmax) {
                return std::nullopt;
            }
        }
    }
    return RayCastResult { tmin, normal };
}

Aabb Aabb::combine(const Aabb& other) const
{
    return Aabb {
        Vec3 {
            std::min(min.x, other.min.x),
            std::min(min.y, other.min.y),
            std::min(min.z, other.min.z),
        },
        Vec3 {
            std::max(max.x, other.max.x),
            std::max(max.y, other.max.y),
            std::max(max.z, other.max.z),
        },
    };
}

namespace {
    Mat4 invertTrs(const Mat4& m)
    {
        // inv(TRS) = inv(S) * inv(R) * inv(T) = inv(S) * transpose(R) * inv(T)
        const auto sx = m.cols[0].length();
        const auto sy = m.cols[1].length();
        const auto sz = m.cols[2].length();

        const auto t = Vec3 { m.cols[3].x, m.cols[3].y, m.cols[3].z };

        const auto r = Mat4 {
            m.cols[0] / sx,
            m.cols[1] / sy,
            m.cols[2] / sz,
            Vec4 { 0.0f, 0.0f, 0.0f, 1.0f },
        };

        return Mat4::scale(Vec3 { 1.0f / sx, 1.0f / sy, 1.0f / sz }) * r.transpose()
            * Mat4::translate(-t);
    }
}

Triangle::Triangle(const Vec3& v0, const Vec3& v1, const Vec3& v2)
    : vertices_ { v0, v1, v2 }
    , normal_((v1 - v0).cross(v2 - v0))
{
}

Vec3 Triangle::support(const Vec3& direction) const
{
    Vec3 maxPoint;
    float maxDot = -std::numeric_limits<float>::max();
    for (const auto& vertex : vertices_) {
        const auto dot = vertex.dot(direction);
        if (dot > maxDot) {
            maxDot = dot;
            maxPoint = vertex;
        }
    }
    return maxPoint;
}

std::optional<RayCastResult> Triangle::rayCast(const Vec3& pos, const Vec3& dir) const
{
    // Check if ray points towards triangle (or coplanar)
    const auto d = -normal_.dot(dir);
    if (d <= 0.0f) {
        return std::nullopt;
    }

    // Check in which half space the ray origin is
    const auto tr = pos - vertices_[0];
    const auto t = normal_.dot(tr);
    if (t < 0.0f) {
        return std::nullopt;
    }

    // compute barycentric coordinates of intersection
    const auto e = dir.cross(tr);
    const auto v = -(vertices_[2] - vertices_[0]).dot(e);
    if (v < 0.0f || v > d) {
        return std::nullopt;
    }
    const auto w = (vertices_[1] - vertices_[0]).dot(e);
    if (w < 0.0f || v + w > d) {
        return std::nullopt;
    }

    return RayCastResult { t / d, normal_.normalized() };
}

ConvexPolyhedron::ConvexPolyhedron(
    std::vector<Vec3> vertices, std::vector<std::tuple<size_t, size_t, size_t>> faces)
    : vertices_(std::move(vertices))
    , faces_(std::move(faces))
{
    assert(vertices_.size() > 0);
    assert(faces_.size() > 0);

    normals_.reserve(faces_.size());
    for (const auto& [i0, i1, i2] : faces_) {
        const auto& v0 = vertices_[i0];
        const auto& v1 = vertices_[i1];
        const auto& v2 = vertices_[i2];
        normals_.push_back((v1 - v0).cross(v2 - v0).normalized());
    }
}

Vec3 ConvexPolyhedron::support(const Vec3& direction) const
{
    // This is one of the most expensive steps of the GJK algorithm. There are ways to improve the
    // complexity of this function to O(log N) by hill climbing for extreme vertices, where vertices
    // are stored in a data structure which allows easy querying for neighbouring vertices (would be
    // trivial in 2D). This requires a good bit more work and it's tricky to avoid infinite loops,
    // so I won't do it until I need it.

    // The convexity of the polyhedron provides that the supporting point is always one of the
    // vertices, so it is enough to simply check all of them.
    assert(vertices_.size() > 0);
    Vec3 maxPoint;
    float maxDot = -std::numeric_limits<float>::max();
    for (const auto& vertex : vertices_) {
        const auto dot = vertex.dot(direction);
        if (dot > maxDot) {
            maxDot = dot;
            maxPoint = vertex;
        }
    }
    return maxPoint;
}

std::optional<RayCastResult> ConvexPolyhedron::rayCast(
    const Vec3& position, const Vec3& direction) const
{
    // Real-Time Collision Detection, 5.3.8
    float tfirst = 0.0f;
    float tlast = std::numeric_limits<float>::max();
    Vec3 normal;
    // Check ray against each half-space defined by each face
    for (size_t i = 0; i < faces_.size(); ++i) {
        const auto& n = normals_[i];
        const auto& v0 = vertices_[std::get<0>(faces_[i])];
        const auto d = v0.dot(n);

        const auto denom = n.dot(direction);
        const auto dist = d - n.dot(position);
        if (denom == 0.0f) {
            // Ray is parallel to the face
            if (dist > 0.0f) {
                return std::nullopt;
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
                return std::nullopt;
            }
        }
    }
    return RayCastResult { tfirst, normal };
}

Sphere::Sphere(float radius)
    : radius_(radius)
{
}

Vec3 Sphere::support(const Vec3& direction) const
{
    return direction.normalized() * radius_;
}

std::optional<RayCastResult> Sphere::rayCast(const Vec3& position, const Vec3& direction) const
{
    // Real-Time Collision Detection, 5.3.2
    const auto b = position.dot(direction);
    const auto c = position.dot(position) - radius_ * radius_;
    // ray starts outside sphere and points away from it or ray starts inside the sphere
    if ((c > 0.0f && b > 0.0f) || c < 0.0f) {
        return std::nullopt;
    }

    const auto discr = b * b - c;
    if (discr < 0.0f) {
        return std::nullopt;
    }

    const auto t = std::max(0.0f, -b - std::sqrt(discr));
    const auto normal = (position + direction * t).normalized();
    return RayCastResult { t, normal };
}

void Collider::addShape(std::unique_ptr<ConvexShape> shape, Mat4 transform)
{
    const auto inverseTransform = invertTrs(transform);
    shapes_.push_back(ColliderShape {
        std::move(shape),
        std::move(transform),
        inverseTransform,
        transform_ * transform,
        // inv(T * t) = inv(t) * inv(T)
        inverseTransform * inverseTransform_,
    });
}

void Collider::setTransform(const Mat4& transform)
{
    transform_ = transform;
    inverseTransform_ = invertTrs(transform);
    for (auto& shape : shapes_) {
        shape.fullTransform = transform_ * shape.transform;
        shape.inverseFullTransform = shape.inverseTransform * inverseTransform_;
    }
    aabbDirty_ = true;
}

void Collider::setTransform(const float* transformMatrix)
{
    Mat4 matrix;
    std::memcpy(&matrix.cols[0].x, transformMatrix, sizeof(Mat4));
    setTransform(matrix);
}

Vec3 Collider::support(const Vec3& direction) const
{
    Vec3 maxPoint;
    float maxDot = -std::numeric_limits<float>::max();
    for (const auto& shape : shapes_) {
        const auto dir = shape.inverseFullTransform * Vec4(direction, 0.0f);
        const auto sup = shape.fullTransform * Vec4(shape.shape->support(dir), 1.0f);
        const auto dot = direction.dot(sup);
        if (dot > maxDot) {
            maxDot = dot;
            maxPoint = sup;
        }
    }
    return maxPoint;
}

Aabb Collider::getAabb() const
{
    // http://allenchou.net/2014/02/game-physics-updating-aabbs-for-polyhedrons/
    if (aabbDirty_) {
        aabb_.min.x = support(-xAxis).x;
        aabb_.min.y = support(-yAxis).y;
        aabb_.min.z = support(-zAxis).z;
        aabb_.max.x = support(xAxis).x;
        aabb_.max.y = support(yAxis).y;
        aabb_.max.z = support(zAxis).z;
        aabbDirty_ = false;
    }
    return aabb_;
}

std::optional<RayCastResult> Collider::rayCast(const Vec3& position, const Vec3& direction) const
{
    RayCastResult res;
    for (const auto& shape : shapes_) {
        const auto pos = shape.inverseFullTransform * Vec4(position, 1.0f);
        const auto dir = shape.inverseFullTransform * Vec4(direction, 0.0f);
        const auto rc = shape.shape->rayCast(pos, dir);
        if (rc) {
            res.t = std::min(res.t, rc->t);
            res.normal = shape.fullTransform * Vec4(rc->normal, 0.0f);
        }
    }
    if (res.t < std::numeric_limits<float>::max()) {
        return res;
    }
    return std::nullopt;
}

namespace {
    Vec3 support(const Collider& a, const Collider& b, const Vec3& direction)
    {
        return a.support(direction) - b.support(-direction);
    }

    bool sameHalfSpace(const Vec3& a, const Vec3& b)
    {
        return a.dot(b) > 0.0f;
    }

    struct NextSimplexResult {
        Simplex3d simplex;
        Vec3 direction;
        bool containsOrigin;
    };

    NextSimplexResult line(const Simplex3d& simplex, const Vec3& /*direction*/)
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
        const auto a = simplex[0];
        const auto b = simplex[1];
        const auto ab = b - a;
        const auto ao = -a;
        // We need to check whether the origin is between a and b (<1>) or "after" a (in the
        // half-space at a in the direction of b -> a, <2>).
        // If ao and ab are in the same half-space, the origin is between a and b (<1>).
        if (sameHalfSpace(ab, ao) > 0.0f) {
            // <1>
            // ab.cross(ao) will give us a vector perpendicular to ab and ao. The second cross
            // product will give us a vector that is coplanar with ab and ao and perpendicular
            // to ab. This is roughly the vector from the line ab towards the origin.
            auto dir = ab.cross(ao).cross(ab);
            if (dir == Vec3 { 0.0f, 0.0f, 0.0f }) {
                // This will happen if a, b and the origin are collinear!
                // Pick any direction orthogonal to ab.
                // https://box2d.org/posts/2014/02/computing-a-basis/
                dir = ab.normalized();
                dir = std::abs(dir.x) >= 0.57735f ? Vec3 { dir.y, -dir.x, 0.0f }
                                                  : Vec3 { 0.0f, dir.z, -dir.y };
            }
            return { simplex, dir, false };
        } else {
            // <2>
            // If ao and ab are not in the same half-space, the origin is "after"/"beyond" a (<2>).
            // This means that the origin is not in the direction of b and it will not help us
            // enclose the origin, so we throw it away.
            // We keep only a (because it more towards the origin) and search towards the origin
            // again.
            return { { a }, ao, false };
        }
    }

    NextSimplexResult triangle(const Simplex3d& simplex, const Vec3& direction)
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

        const auto a = simplex[0];
        const auto b = simplex[1];
        const auto c = simplex[2];

        const auto ab = b - a;
        const auto ac = c - a;
        const auto ao = -a;
        const auto abc = ab.cross(ac); // normal of the triangle

        // Regions <6>, <7> and <8> are ruled out, because they do not point towards `a`.

        // First we try to rule out region <1> and <4>, because according to Casey, they rule out
        // the largest number of options, but I don't quite understand it.

        // `abc.cross(ac)` is the normal of the plane that contains a and c (coplanar), so we are
        // checking if the origin is in the half-space defined by that plane (<1> and <5>).
        if (sameHalfSpace(abc.cross(ac), ao)) {
            // Now we still need to distinguish <1> and <5> by checking whether the origin is
            // towards C (<1>) or away from C (<5>).
            if (sameHalfSpace(ac, ao)) {
                // <1>
                // The direction is a vector orthogonal to the edge `ac`, but pointing towards
                // the origin.
                return { { a, c }, ac.cross(ao).cross(ac), false };
            } else {
                // <5>
                return line({ a, b }, direction);
            }
        }

        // `ab.cross(abc)` is the normal of the plane that contains a and b, so we are checking
        // for region <4>.
        if (sameHalfSpace(ab.cross(abc), ao)) {
            // <4>
            return line({ a, b }, direction);
        }

        // <2> or <3> are left
        if (sameHalfSpace(abc, ao)) {
            // "above" the triangle
            return { simplex, abc, false };
        } else {
            // "below" the triangle
            // "rewind" the triangle to point "down" instead
            return { { a, c, b }, -abc, false };
        }
    }

    NextSimplexResult tetrahedron(const Simplex3d& simplex, const Vec3& direction)
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

        const auto a = simplex[0];
        const auto b = simplex[1];
        const auto c = simplex[2];
        const auto d = simplex[3];

        const auto ab = b - a;
        const auto ac = c - a;
        const auto ad = d - a;
        const auto ao = -a;

        const auto abc = ab.cross(ac);
        const auto acd = ac.cross(ad);
        const auto adb = ad.cross(ab);

        // Since we chose `a` last, the origin should be in the half-space defined by the plane bcd
        // in the direction of a. We just need to check if the origin is in the half-space defined
        // by the sides of the tetrahedron (pointing outside). If the origin is neither in the space
        // defined by abc, acd and adb, we know it must be inside the tetrahedron, because it can't
        // be in the space defined by bcd (see first statement).

        if (sameHalfSpace(abc, ao)) {
            return triangle({ a, b, c }, direction);
        }
        if (sameHalfSpace(acd, ao)) {
            return triangle({ a, c, d }, direction);
        }
        if (sameHalfSpace(adb, ao)) {
            return triangle({ a, d, b }, direction);
        }

        return { simplex, direction, true };
    }

    NextSimplexResult nextSimplex(const Simplex3d& simplex, const Vec3& direction)
    {
        switch (simplex.size()) {
        case 2:
            return line(simplex, direction);
        case 3:
            return triangle(simplex, direction);
        case 4:
            return tetrahedron(simplex, direction);
        default:
            assert(false && "Invalid number of points in simplex");
        }
    }

}

namespace detail {
    std::optional<Simplex3d> gjk(const Collider& c1, const Collider& c2, GjkDebug* debug)
    {
        // As the initial direction we choose the x-axis. We want to bias the search towards the
        // origin/point of collision, so it would make sense to use the relative vector between the
        // centers of the shapes.
        // Casey says it doesn't really matter what we start with, because it converges really
        // quickly.
        Vec3 direction = Vec3 { 1.0f, 0.0f, 0.0f };

        const auto a0 = support(c1, c2, direction);

        // TODO: Handle a0 == origin

        Simplex3d simplex;
        simplex = { a0 };

        if (debug) {
            debug->iterations.push_back(
                { direction, c1.support(direction), c2.support(-direction), a0, simplex, false });
        }

        // Choose dir towards the origin: a0 -> O = O - a0 = -a0.
        direction = -a0;

        size_t numIterations = 0;
        while (++numIterations < 64) {
            const auto a = support(c1, c2, direction);
            if (debug) {
                debug->iterations.push_back(
                    { direction, c1.support(direction), c2.support(-direction), a });
            }
            if (a.dot(direction) < 0.0f) {
                // No Intersection:
                // We went towards the origin (direction points towards the origin) and the next
                // supporting point we found was away from the origin instead. That means we did not
                // "cross" the origin and there is no way to include the origin.
                return std::nullopt;
            }

            simplex.pushFront(a);
            // std::cout << "after push (" << simplex.size() << "): " << toString(simplex) <<
            // std::endl;

            // std::cout << "before nextSimplex" << std::endl;
            // std::cout << "simplex (" << simplex.size() << "): " << toString(simplex) <<
            // std::endl; std::cout << "direction: " << toString(direction) << std::endl;
            auto res = nextSimplex(simplex, direction);
            if (debug) {
                debug->iterations.back().simplex = res.simplex;
                debug->iterations.back().containsOrigin = res.containsOrigin;
            }
            // std::cout << "after nextSimplex" << std::endl;
            // std::cout << "containsOrigin: " << res.containsOrigin << std::endl;
            // std::cout << "simplex (" << res.simplex.size() << "): " << toString(res.simplex)
            // << std::endl;
            // std::cout << "direction: " << toString(res.direction) << std::endl;
            if (res.containsOrigin) {
                return res.simplex;
            }
            simplex = std::move(res.simplex);
            direction = res.direction;
        }

        // We only reach this if we exceed the maximum number of iterations
        return std::nullopt;
    }
}

bool testCollision(const Collider& c1, const Collider& c2)
{
    if (!c1.getAabb().overlaps(c2.getAabb())) {
        return false;
    }
    return gjk(c1, c2).has_value();
}

namespace {
    void updateNormal(const std::vector<Vec3>& vertices, EpaTriangle& face)
    {
        const auto& v0 = vertices[face.v0];
        const auto& v1 = vertices[face.v1];
        const auto& v2 = vertices[face.v2];

        face.normal = (v1 - v0).cross(v2 - v0).normalized();
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
        face.dist = face.normal.dot(v0);
        if (face.dist < 0.0f) {
            // Fix winding order of the triangle so that v0 -> v1 -> v2 is CCW
            std::swap(face.v1, face.v2);
            face.normal = -face.normal;
            face.dist = -face.dist;
        }
    }

    // Returns the index of the face closest to the origin and its distance.
    std::pair<size_t, float> getClosestFace(const std::vector<EpaTriangle>& faces)
    {
        float minDist = std::numeric_limits<float>::max();
        size_t minFaceIdx = 0;
        assert(faces.size() > 0);
        for (size_t i = 0; i < faces.size(); ++i) {
            assert(faces[i].normal != (Vec3 { 0.0f, 0.0f, 0.0f }));
            assert(faces[i].dist >= 0.0f);
            if (faces[i].dist < minDist) {
                minDist = faces[i].dist;
                minFaceIdx = i;
            }
        }
        return { minFaceIdx, minDist };
    }

}

namespace detail {
    // Expanding Polytope Algorithm
    Collision epa(const Collider& c1, const Collider& c2, const Simplex3d& simplex, EpaDebug* debug)
    {
        // Our variant of GJK always computes a tetrahedron before returning true.
        // Other variants might not do that and would require extension of the simplex to at least 4
        // vertices.
        assert(simplex.size() == 4);

        std::vector<Vec3> polytopeVertices(simplex.begin(), simplex.end());
        std::vector<EpaTriangle> polytopeFaces {
            { 0, 1, 2 },
            { 0, 2, 3 },
            { 0, 3, 1 },
            { 1, 3, 2 },
        };

        for (auto& face : polytopeFaces) {
            updateNormal(polytopeVertices, face);
        }

        size_t minDistFaceIdx = 0;
        float minFaceDist = 0.0f;

        size_t numIterations = 0;
        while (++numIterations < 32) {
            std::tie(minDistFaceIdx, minFaceDist) = getClosestFace(polytopeFaces);

            // Expand the polytope "outside" from the face closest to the origin.
            const auto minDistFaceNormal = polytopeFaces[minDistFaceIdx].normal;
            const auto supPoint = support(c1, c2, minDistFaceNormal);
            const auto supDist = minDistFaceNormal.dot(supPoint);

            if (debug) {
                auto& it = debug->iterations.emplace_back();
                it.polytopeVertices = polytopeVertices;
                it.polytopeFaces = polytopeFaces;
                it.minDistFaceIdx = minDistFaceIdx;
                it.minFaceDist = minFaceDist;
                it.supPoint = supPoint;
                it.supDist = supDist;
            }

            // If we cannot expand the polytope (the new point is on the closest edge), the closest
            // face is actually the closest face possible and we have reached the edge of the
            // minkowski difference.
            if (std::abs(supDist - minFaceDist) < 1e-4f) {
                break;
            }

            using Edge = std::pair<size_t, size_t>;
            std::vector<Edge> edgesToPatch;
            // If the edge is part of multiple triangles, we DO NOT want to patch it, because it
            // would create internal geometry, which will mess up everything.
            // If the edge is part of multiple triangles, it will be one other adjacent triangle at
            // most and the edge's direction will be reversed (if both triangles are CCW, which we
            // made sure of in updateNormal).
            auto addUniqueEdge = [&edgesToPatch](size_t first, size_t second) {
                const auto reverseEdge
                    = std::find(edgesToPatch.begin(), edgesToPatch.end(), Edge(second, first));
                assert(std::find(edgesToPatch.begin(), edgesToPatch.end(), Edge(first, second))
                    == edgesToPatch.end());
                if (reverseEdge != edgesToPatch.end()) {
                    *reverseEdge = edgesToPatch.back();
                    edgesToPatch.pop_back();
                } else {
                    edgesToPatch.emplace_back(first, second);
                }
            };

            if (debug) {
                for (size_t i = 0; i < polytopeFaces.size(); ++i) {
                    if (sameHalfSpace(polytopeFaces[i].normal,
                            supPoint - polytopeVertices[polytopeFaces[i].v0])) {
                        debug->iterations.back().removedFaces.push_back(i);
                    }
                }
            }

            // Remove all faces with a normal towards the new point
            for (size_t i = 0; i < polytopeFaces.size();) {
                // So many implementations check for dot(normal, supPoint) > 0 here, which is
                // clearly wrong.
                // Counter-example: Plane at (2,0) with normal (1,0) and new supporting point
                // at (1,0) (or (1,5) or something for a more realistic scenario). In this case
                // dot(normal, supPoint) would be > 0, but the point is clearly on the wrong side of
                // the plane.
                if (sameHalfSpace(polytopeFaces[i].normal,
                        supPoint - polytopeVertices[polytopeFaces[i].v0])) {
                    addUniqueEdge(polytopeFaces[i].v0, polytopeFaces[i].v1);
                    addUniqueEdge(polytopeFaces[i].v1, polytopeFaces[i].v2);
                    addUniqueEdge(polytopeFaces[i].v2, polytopeFaces[i].v0);

                    // Move last element into place i to remove it.
                    polytopeFaces[i] = polytopeFaces.back();
                    polytopeFaces.pop_back();
                } else {
                    ++i;
                }
            }

            if (debug) {
                debug->iterations.back().edgesToPatch = edgesToPatch;
            }

            // Patch up the freshly cut edges (tasty!) with the new point
            polytopeVertices.push_back(supPoint);
            const auto supPointIdx = polytopeVertices.size() - 1;

            for (const auto& [first, second] : edgesToPatch) {
                polytopeFaces.push_back({ first, second, supPointIdx });
                updateNormal(polytopeVertices, polytopeFaces.back());
            }
        }

        // TODO: Project the origin into the closest face to approximate contact points

        return {
            polytopeFaces[minDistFaceIdx].normal,
            // Add some epsilon to make sure we resolve the collision
            minFaceDist + 1e-4f,
        };
    }
}

std::optional<Collision> getCollision(
    const Collider& a, const Collider& b, detail::GjkDebug* gjkDebug, detail::EpaDebug* epaDebug)
{
    if (!a.getAabb().overlaps(b.getAabb())) {
        return std::nullopt;
    }
    const auto gjkRes = gjk(a, b, gjkDebug);
    if (!gjkRes) {
        return std::nullopt;
    }
    return epa(a, b, *gjkRes, epaDebug);
}

size_t AabbTree::insert(Collider* collider)
{
    const auto idx = getNewNode();
    nodes_[idx].collider = collider;
    updateAabb(idx);
    insertIntoTree(idx, rootIdx_);
    return idx;
}

void AabbTree::remove(Collider* collider)
{
    const auto idx = findNode(collider);
    assert(idx != InvalidIdx);
    removeNode(idx);
}

void AabbTree::removeNode(size_t nodeIdx)
{
    removeFromTree(nodeIdx);

    nodes_[nodeIdx].parentIdx = InvalidIdx;
    nodes_[nodeIdx].leftIdx = InvalidIdx;
    nodes_[nodeIdx].rightIdx = InvalidIdx;
    nodes_[nodeIdx].collider = nullptr;
    nodeFreeList_.push(nodeIdx);
}

void AabbTree::update(Collider* collider)
{
    const auto idx = findNode(collider);
    assert(idx != InvalidIdx);
    updateNode(idx);
}

void AabbTree::updateNode(size_t nodeIdx)
{
    assert(nodeIdx < nodes_.size());
    removeFromTree(nodeIdx);
    updateAabb(nodeIdx);
    insertIntoTree(nodeIdx, rootIdx_);
}

AabbTree::ColliderList AabbTree::query(const Vec3& point) const
{
    std::queue<size_t> q;
    std::vector<Collider*> colliders;
    if (rootIdx_ != InvalidIdx) {
        q.push(rootIdx_);
    }
    while (q.size()) {
        const auto& node = nodes_[q.front()];
        q.pop();

        if (node.aabb.contains(point)) {
            if (node.isLeaf()) {
                colliders.push_back(node.collider);
            } else {
                q.push(node.leftIdx);
                q.push(node.rightIdx);
            }
        }
    }
    return colliders;
}

AabbTree::ColliderList AabbTree::query(const Aabb& aabb) const
{
    std::queue<size_t> q;
    std::vector<Collider*> colliders;
    if (rootIdx_ != InvalidIdx) {
        q.push(rootIdx_);
    }
    while (q.size()) {
        const auto& node = nodes_[q.front()];
        q.pop();

        if (node.aabb.overlaps(aabb)) {
            if (node.isLeaf()) {
                colliders.push_back(node.collider);
            } else {
                q.push(node.leftIdx);
                q.push(node.rightIdx);
            }
        }
    }
    return colliders;
}

AabbTree::ColliderPairList AabbTree::getNeighbours() const
{
    return {};
}

std::optional<std::pair<RayCastResult, Collider*>> AabbTree::rayCast(
    const Vec3& position, const Vec3& direction) const
{
    std::queue<size_t> q;
    std::optional<std::pair<RayCastResult, Collider*>> result;
    if (rootIdx_ != InvalidIdx) {
        q.push(rootIdx_);
    }
    while (q.size()) {
        const auto& node = nodes_[q.front()];
        q.pop();

        const auto rc = node.aabb.rayCast(position, direction);
        if (rc) {
            if (result && result->first.t < rc->t) {
                continue;
            }

            if (node.isLeaf()) {
                const auto crc = node.collider->rayCast(position, direction);
                if (crc && (!result || crc->t < result->first.t)) {
                    result = std::pair(*crc, node.collider);
                }
            } else {
                q.push(node.leftIdx);
                q.push(node.rightIdx);
            }
        }
    }
    return result;
}

std::vector<std::pair<Aabb, uint32_t>> AabbTree::getAabbs() const
{
    std::vector<std::pair<Aabb, uint32_t>> aabbs;
    std::queue<std::pair<size_t, uint32_t>> q;
    q.push({ rootIdx_, 0ul });
    while (q.size()) {
        const auto [nodeIdx, depth] = q.front();
        q.pop();

        aabbs.push_back({ nodes_[nodeIdx].aabb, depth });
        if (!nodes_[nodeIdx].isLeaf()) {
            q.push({ nodes_[nodeIdx].leftIdx, depth + 1 });
            q.push({ nodes_[nodeIdx].rightIdx, depth + 1 });
        }
    }
    return aabbs;
}

bool AabbTree::Node::isLeaf() const
{
    return collider != nullptr;
}

void AabbTree::Node::replaceChild(size_t oldChildIdx, size_t newChildIdx)
{
    assert(leftIdx == oldChildIdx || rightIdx == oldChildIdx);
    if (leftIdx == oldChildIdx) {
        leftIdx = newChildIdx;
    } else {
        rightIdx = newChildIdx;
    }
}

size_t AabbTree::getNewNode()
{
    if (!nodeFreeList_.empty()) {
        const auto idx = nodeFreeList_.front();
        nodeFreeList_.pop();
        return idx;
    }

    nodes_.emplace_back();
    return nodes_.size() - 1;
}

void AabbTree::updateAabb(size_t nodeIdx)
{
    auto& node = nodes_[nodeIdx];
    if (node.collider) {
        node.aabb = node.collider->getAabb();
    } else {
        assert(node.leftIdx != InvalidIdx && node.rightIdx != InvalidIdx);
        const auto& left = nodes_[node.leftIdx];
        const auto& right = nodes_[node.rightIdx];
        node.aabb = left.aabb.combine(right.aabb);
    }
}

size_t AabbTree::findNode(Collider* collider) const
{
    assert(collider != nullptr);
    for (size_t i = 0; i < nodes_.size(); ++i) {
        if (nodes_[i].collider == collider) {
            return i;
        }
    }
    return InvalidIdx;
}

void AabbTree::insertIntoTree(size_t nodeIdx, size_t parentIdx)
{
    // http://allenchou.net/2014/02/game-physics-broadphase-dynamic-aabb-tree/
    // https://box2d.org/files/ErinCatto_DynamicBVH_Full.pdf
    if (rootIdx_ == InvalidIdx) {
        rootIdx_ = nodeIdx;
        return;
    }

    if (nodes_[parentIdx].isLeaf()) {
        const auto newParentIdx = getNewNode();
        nodes_[newParentIdx].leftIdx = nodeIdx;
        nodes_[newParentIdx].rightIdx = parentIdx;

        const auto oldParentIdx = nodes_[parentIdx].parentIdx;
        if (oldParentIdx == InvalidIdx) {
            assert(parentIdx == rootIdx_);
            rootIdx_ = newParentIdx;
        } else {
            nodes_[oldParentIdx].replaceChild(parentIdx, newParentIdx);
            nodes_[newParentIdx].parentIdx = oldParentIdx;
        }

        nodes_[nodeIdx].parentIdx = newParentIdx;
        nodes_[parentIdx].parentIdx = newParentIdx;

        updateAabb(newParentIdx);
    } else {
        assert(nodes_[parentIdx].leftIdx != InvalidIdx && nodes_[parentIdx].rightIdx != InvalidIdx);
        const auto& leftAabb = nodes_[nodes_[parentIdx].leftIdx].aabb;
        const auto& rightAabb = nodes_[nodes_[parentIdx].rightIdx].aabb;
        const auto leftVolDiff
            = leftAabb.combine(nodes_[nodeIdx].aabb).volume() - leftAabb.volume();
        const auto rightVolDiff
            = rightAabb.combine(nodes_[nodeIdx].aabb).volume() - rightAabb.volume();
        if (leftVolDiff <= rightVolDiff) {
            insertIntoTree(nodeIdx, nodes_[parentIdx].leftIdx);
        } else {
            insertIntoTree(nodeIdx, nodes_[parentIdx].rightIdx);
        }
        updateAabb(parentIdx);
    }
}

void AabbTree::removeFromTree(size_t nodeIdx)
{
    assert(nodeIdx < nodes_.size());

    const auto parentIdx = nodes_[nodeIdx].parentIdx;
    if (parentIdx == InvalidIdx) {
        assert(nodeIdx == rootIdx_);
        rootIdx_ = InvalidIdx;
    } else {
        assert(nodes_[parentIdx].leftIdx == nodeIdx || nodes_[parentIdx].rightIdx == nodeIdx);
        const auto siblingIdx = nodes_[parentIdx].rightIdx == nodeIdx ? nodes_[parentIdx].leftIdx
                                                                      : nodes_[parentIdx].rightIdx;
        if (parentIdx == rootIdx_) {
            rootIdx_ = siblingIdx;
            nodes_[siblingIdx].parentIdx = InvalidIdx;
        } else {
            const auto grandparentIdx = nodes_[parentIdx].parentIdx;
            assert(grandparentIdx != InvalidIdx);
            nodes_[siblingIdx].parentIdx = grandparentIdx;
            nodes_[grandparentIdx].replaceChild(parentIdx, siblingIdx);
        }
    }
}

void AabbTree::print() const
{
    print(rootIdx_, 0);
}

void AabbTree::print(size_t nodeIdx, uint32_t depth) const
{
    for (size_t i = 0; i < depth; ++i) {
        std::cout << "  ";
    }
    const auto& node = nodes_[nodeIdx];

    std::printf("%zu (collider = %p, parent = %zu)\n", nodeIdx,
        static_cast<const void*>(nodes_[nodeIdx].collider), nodes_[nodeIdx].parentIdx);

    if (!node.collider) {
        print(node.leftIdx, depth + 1);
        print(node.rightIdx, depth + 1);
    }
}
}
