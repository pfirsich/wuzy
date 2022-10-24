#include "wuzy/wuzy.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>

/* Sources:
 * https://www.youtube.com/watch?v=Qupqu1xe7Io
 */

using namespace wuzy::detail;

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

bool Vec3::operator==(const Vec3& other) const
{
    return x == other.x && y == other.y && z == other.z;
}

bool Vec3::operator!=(const Vec3& other) const
{
    return !(*this == other);
}

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
    std::optional<Simplex3d> gjk(const Collider& c1, const Collider& c2)
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

        // Choose dir towards the origin: a0 -> O = O - a0 = -a0.
        direction = -a0;

        size_t numIterations = 0;
        while (++numIterations < 64) {
            const auto a = support(c1, c2, direction);
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
    return gjk(c1, c2).has_value();
}

namespace {
    void updateNormal(const std::vector<Vec3>& vertices, Triangle& face)
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
    std::pair<size_t, float> getClosestFace(const std::vector<Triangle>& faces)
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
        std::vector<Triangle> polytopeFaces {
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

std::optional<Collision> getCollision(const Collider& a, const Collider& b)
{
    const auto gjkRes = gjk(a, b);
    if (!gjkRes) {
        return std::nullopt;
    }
    return epa(a, b, *gjkRes);
}
}
