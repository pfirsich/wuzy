#include "wuzy/wuzy.hpp"

#include <cassert>
#include <cmath>

/* Sources:
 * https://www.youtube.com/watch?v=Qupqu1xe7Io
 */

namespace wuzy {
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

Vec3 Vec3::operator-() const
{
    return Vec3 { -x, -y, -z };
}

Vec3 Vec3::operator-(const Vec3& other) const
{
    return Vec3 { x - other.x, y - other.y, z - other.z };
}

Vec3 Vec3::operator*(float s) const
{
    return Vec3 { s * x, s * y, s * z };
}
Vec3 Vec3::operator/(float s) const
{
    return Vec3 { x / s, y / s, z / s };
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
    float maxDot = std::numeric_limits<float>::min();
    for (const auto& vertex : vertices_) {
        const auto dot = vertex.dot(direction);
        if (dot > maxDot) {
            maxDot = dot;
            maxPoint = vertex;
        }
    }
    return maxPoint;
}

Vec3 Sphere::support(const Vec3& direction) const
{
    return direction.normalized() * radius_;
}

void Collider::addShape(std::unique_ptr<ConvexShape> shape, const Mat4& transform)
{
    shapes_.push_back(ColliderShape { std::move(shape), transform });
}

Vec3 Collider::support(const Vec3& direction) const
{
    Vec3 maxPoint;
    float maxDot = std::numeric_limits<float>::min();
    for (const auto& shape : shapes_) {
        const auto sup = shape.shape->support(direction);
        const auto dot = sup.dot(direction);
        if (dot > maxDot) {
            maxDot = dot;
            maxPoint = sup;
        }
    }
    return maxPoint;
}

namespace {
    Vec3 support(const Collider& a, const Collider& b, const Vec3& direction)
    {
        return a.support(direction) - b.support(-direction);
    }

    // This is not a full-blown StaticVector. Just what I need for the simplex.
    template <typename T, size_t N>
    class StaticVector {
    public:
        StaticVector() = default;
        StaticVector(const StaticVector& other) = default;
        StaticVector(StaticVector&& other) = default;
        StaticVector& operator=(StaticVector&& other) = default;

        StaticVector(std::initializer_list<T> init)
        {
            assert(init.size() <= N);
            for (auto it = init.begin(); it != init.end(); ++it) {
                elements_[std::distance(it, init.begin())] = *it;
            }
            size_ = init.size();
        }

        void pushFront(T v)
        {
            // This actually just needs to go until size_, but with N being a compile-time constant
            // it might be more likely the compiler unrolls this loop.
            for (size_t i = 1; i < N; ++i) {
                elements_[i] = elements_[i - 1];
            }
            elements_[0] = std::move(v);
            size_ = std::max(size_ + 1, N);
        }

        const Vec3& operator[](size_t i) const
        {
            assert(i < size_);
            return elements_[i];
        }

        size_t size() const { return size_; }

    private:
        std::array<T, N> elements_;
        size_t size_ = 0;
    };

    using Simplex3d = StaticVector<Vec3, 4>;

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
            // ab.cross(ao) will give us a vector perpendicular to ab and ao. The second cross
            // product will give us a vector that is coplanar with ab and ao and perpendicular
            // to ab. This is roughly the vector from the line ab towards the origin.
            return { simplex, ab.cross(ao).cross(ab), false };
        } else {
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
                // The direction is a vector orthogonal to the edge `ac`, but pointing towards the
                // origin.
                return { { a, c }, ac.cross(ao).cross(ac), false };
            } else {
                return line({ a, b }, direction);
            }
        } else {
            // `ab.cross(abc)` is the normal of the plane that contains a and b, so we are checking
            // for region <4>.
            if (sameHalfSpace(ab.cross(abc), ao)) {
                return line({ a, b }, direction);
            } else {
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

    std::optional<Simplex3d> gjk(const Collider& c1, const Collider& c2)
    {
        // As the initial direction we choose the x-axis. We want to bias the search towards the
        // origin/point of collision, so it would make sense to use the relative vector between the
        // centers of the shapes.
        // Casey says it doesn't really matter what we start with, because it converges really
        // quickly.
        Vec3 direction = Vec3 { 1.0f, 0.0f, 0.0f };

        const auto a0 = support(c1, c2, direction);

        Simplex3d simplex;
        simplex = { a0 };

        // Choose dir towards the origin: a0 -> O = O - a0 = -a0.
        direction = -a0;

        while (true) {
            const auto a = support(c1, c2, direction);
            if (a.dot(direction) < 0.0f) {
                // No Intersection:
                // We went towards the origin (direction points towards the origin) and the next
                // supporting point we found was away from the origin instead. That means we did not
                // "cross" the origin and there is no way to include the origin.
                return std::nullopt;
            }

            simplex.pushFront(a);

            auto res = nextSimplex(simplex, direction);
            if (res.containsOrigin) {
                return res.simplex;
            }
            simplex = std::move(res.simplex);
            direction = res.direction;
        }
    }
}

bool testCollision(const Collider& c1, const Collider& c2)
{
    return gjk(c1, c2).has_value();
}

std::optional<Collision> getCollision(const Collider& a, const Collider& b)
{
    const auto gjkRes = gjk(a, b);
    if (!gjkRes) {
        return std::nullopt;
    }
    return Collision {};
}
}
