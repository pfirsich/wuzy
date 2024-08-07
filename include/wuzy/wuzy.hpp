#pragma once

#include <array>
#include <cassert>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <vector>

namespace wuzy {
struct Vec3 {
    float x = 0.0f, y = 0.0f, z = 0.0f;

    Vec3& operator=(const Vec3& other) = default;

    float dot(const Vec3& other) const;
    Vec3 cross(const Vec3& other) const;

    float length() const;
    Vec3 normalized() const;

    float operator[](size_t i) const;

    Vec3 operator-() const;
    Vec3 operator-(const Vec3& other) const;
    Vec3 operator+(const Vec3& other) const;
    Vec3 operator*(float s) const;
    Vec3 operator/(float s) const;
    bool operator==(const Vec3& other) const;
    bool operator!=(const Vec3& other) const;
};

extern Vec3 xAxis;
extern Vec3 yAxis;
extern Vec3 zAxis;

struct Vec4 {
    float x = 0.0f, y = 0.0f, z = 0.0f, w = 0.0f;

    Vec4() = default;
    Vec4(float x, float y, float z, float w);
    Vec4(const Vec3& v, float w);

    Vec4& operator=(const Vec4& other) = default;

    float dot(const Vec4& other) const;
    float length() const;

    Vec4 operator/(float s) const;
    operator Vec3() const;
};

struct Mat4 {
    // OpenGL format
    std::array<Vec4, 4> cols = {
        Vec4 { 1.0f, 0.0f, 0.0f, 0.0f },
        Vec4 { 0.0f, 1.0f, 0.0f, 0.0f },
        Vec4 { 0.0f, 0.0f, 1.0f, 0.0f },
        Vec4 { 0.0f, 0.0f, 0.0f, 1.0f },
    };

    Mat4 transpose() const;
    Mat4 operator*(const Mat4& m) const;
    Vec4 operator*(const Vec4& v) const;

    static Mat4 translate(const Vec3& v);
    static Mat4 scale(const Vec3& v);
};

struct RayCastResult {
    float t = std::numeric_limits<float>::max();
    Vec3 normal = {};
};

struct Aabb {
    Vec3 min;
    Vec3 max;

    Vec3 center() const;
    Vec3 extent() const;
    Vec3 size() const;
    float area() const;
    float volume() const;

    bool contains(const Vec3& point) const;
    bool overlaps(const Aabb& other) const;
    std::optional<RayCastResult> rayCast(const Vec3& position, const Vec3& direction) const;

    Aabb combine(const Aabb& other) const; // `union` is a keyword
};

namespace detail {
    // This is not a full-blown StaticVector. Just what I need for the simplex.
    template <typename T, size_t N>
    class StaticVector {
    public:
        StaticVector() = default;
        StaticVector(const StaticVector& other) = default;
        StaticVector& operator=(const StaticVector& other) = default;
        StaticVector(StaticVector&& other) = default;
        StaticVector& operator=(StaticVector&& other) = default;

        StaticVector(std::initializer_list<T> init)
        {
            assert(init.size() <= N);
            for (auto it = init.begin(); it != init.end(); ++it) {
                elements_[std::distance(init.begin(), it)] = *it;
            }
            size_ = init.size();
        }

        void pushFront(T v)
        {
            // This actually just needs to go until size_, but with N being a compile-time constant
            // it might be more likely the compiler unrolls this loop.
            for (size_t i = N - 1; i > 0; --i) {
                elements_[i] = elements_[i - 1];
            }
            elements_[0] = std::move(v);
            size_ = std::min(size_ + 1, N);
        }

        const Vec3& operator[](size_t i) const
        {
            assert(i < size_);
            return elements_[i];
        }

        size_t size() const { return size_; }

        auto begin() const { return elements_.begin(); }
        auto end() const { return elements_.begin() + size_; }

    private:
        std::array<T, N> elements_;
        size_t size_ = 0;
    };

    using Simplex3d = StaticVector<Vec3, 4>;
}

class ConvexShape {
public:
    // Returns the furthest point along a given direction inside the shape
    // i.e. returns P with dot(direction, P) = max{dot(direction, V) : V in Shape},
    // a "supporting point" of the shape for a given direction.
    // It may not be unique.
    virtual Vec3 support(const Vec3& direction) const = 0;

    virtual std::optional<RayCastResult> rayCast(const Vec3& position, const Vec3& direction) const
        = 0;
};

class Triangle : public ConvexShape {
public:
    Triangle(const Vec3& v0, const Vec3& v1, const Vec3& v2);
    Vec3 support(const Vec3& direction) const override;
    std::optional<RayCastResult> rayCast(
        const Vec3& position, const Vec3& direction) const override;

private:
    std::array<Vec3, 3> vertices_;
    Vec3 normal_;
};

class ConvexPolyhedron : public ConvexShape {
public:
    ConvexPolyhedron(
        std::vector<Vec3> vertices, std::vector<std::tuple<size_t, size_t, size_t>> faces);

    Vec3 support(const Vec3& direction) const override;
    std::optional<RayCastResult> rayCast(
        const Vec3& position, const Vec3& direction) const override;

private:
    std::vector<Vec3> vertices_;
    std::vector<std::tuple<size_t, size_t, size_t>> faces_;
    std::vector<Vec3> normals_;
};

class Sphere : public ConvexShape {
public:
    Sphere(float radius);

    Vec3 support(const Vec3& direction) const override;
    std::optional<RayCastResult> rayCast(
        const Vec3& position, const Vec3& direction) const override;

private:
    float radius_;
};

struct Collider {
public:
    void* userData = nullptr;

    Collider() = default;

    void addShape(std::unique_ptr<ConvexShape> shape, Mat4 transform);

    template <typename T, typename... Args>
    void addShape(Mat4 transform, Args&&... args)
    {
        addShape(std::make_unique<T>(std::forward<Args>(args)...), std::move(transform));
    }

    void setTransform(const Mat4& transform);
    void setTransform(const float* transformMatrix);
    Vec3 support(const Vec3& direction) const;
    Aabb getAabb() const;

    uint64_t getBitmask() const { return bitmask_; }

    void setBitmask(uint64_t v) { bitmask_ = v; }

    std::optional<RayCastResult> rayCast(const Vec3& position, const Vec3& direction) const;

private:
    struct ColliderShape {
        std::unique_ptr<ConvexShape> shape;
        Mat4 transform;
        Mat4 inverseTransform;
        Mat4 fullTransform; // includes Collider transform
        Mat4 inverseFullTransform;
    };

    std::vector<ColliderShape> shapes_;
    Mat4 transform_;
    Mat4 inverseTransform_;
    mutable Aabb aabb_;
    mutable bool aabbDirty_ = true;
    uint64_t bitmask_ = 0xffff'ffff'ffff'ffff;
};

namespace detail {
    struct GjkDebug {
        struct Iteration {
            Vec3 direction;
            Vec3 aSupport;
            Vec3 bSupport;
            Vec3 support;
            Simplex3d simplex = {};
            bool containsOrigin = false;
        };

        std::vector<Iteration> iterations;
    };

    // Returns simplex that contains the origin, if there is a non-empty intersection
    std::optional<Simplex3d> gjk(const Collider& c1, const Collider& c2, GjkDebug* debug = nullptr);
}

bool testCollision(const Collider& a, const Collider& b);

struct Collision {
    // This is the normal for the first collider.
    // To resolve the collision you have to move by -normal * penetrationDepth
    Vec3 normal;
    float penetrationDepth;
};

namespace detail {
    struct EpaTriangle {
        size_t v0;
        size_t v1;
        size_t v2;
        Vec3 normal = {};
        float dist = 0.0f; // Distance to origin
    };

    struct EpaDebug {
        struct Iteration {
            std::vector<Vec3> polytopeVertices;
            std::vector<EpaTriangle> polytopeFaces;
            size_t minDistFaceIdx;
            float minFaceDist;
            Vec3 supPoint;
            float supDist;
            std::vector<size_t> removedFaces;
            std::vector<std::pair<size_t, size_t>> edgesToPatch;
        };

        std::vector<Iteration> iterations;
    };

    Collision epa(const Collider& c1, const Collider& c2, const Simplex3d& simplex,
        EpaDebug* debug = nullptr);
}

std::optional<Collision> getCollision(const Collider& a, const Collider& b,
    detail::GjkDebug* gjkDebug = nullptr, detail::EpaDebug* epaDebug = nullptr);

class AabbTree {
public:
    size_t insert(Collider* collider);
    void remove(Collider* collider);
    void removeNode(size_t nodeIdx);
    void update(Collider* collider);
    void updateNode(size_t nodeIdx);

    // TODO: Add a custom class for this with begin() and end()
    using ColliderList = std::vector<Collider*>;

    ColliderList query(const Vec3& point) const;
    ColliderList query(const Aabb& aabb) const;

    using ColliderPairList = std::vector<std::pair<Collider*, Collider*>>;
    ColliderPairList getNeighbours() const;

    std::optional<std::pair<RayCastResult, Collider*>> rayCast(const Vec3& position,
        const Vec3& direction, uint64_t bitmask = 0xffff'ffff'ffff'ffff) const;

    // Returns vector of AABB + depth
    std::vector<std::pair<Aabb, uint32_t>> getAabbs() const;

    void print() const;

private:
    static constexpr size_t InvalidIdx = std::numeric_limits<size_t>::max();

    struct Node {
        Aabb aabb;
        uint64_t bitmask = 0;

        size_t parentIdx = InvalidIdx;
        size_t leftIdx = InvalidIdx;
        size_t rightIdx = InvalidIdx;

        Collider* collider = nullptr;

        bool isLeaf() const;

        void replaceChild(size_t oldChildIdx, size_t newChildIdx);
    };

    size_t getNewNode();
    void updateNodeData(size_t nodeIdx);
    size_t findNode(Collider* collider) const;
    void insertIntoTree(size_t nodeIdx, size_t parentIdx);
    void removeFromTree(size_t nodeIdx);

    void print(size_t node, uint32_t depth) const;

    std::vector<Node> nodes_;
    size_t rootIdx_ = InvalidIdx;
    std::queue<size_t> nodeFreeList_;
};
}
