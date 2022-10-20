#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace wuzy {
struct Vec3 {
    float x = 0.0f, y = 0.0f, z = 0.0f;

    Vec3& operator=(const Vec3& other) = default;

    float dot(const Vec3& other) const;
    Vec3 cross(const Vec3& other) const;

    float length() const;
    Vec3 normalized() const;

    Vec3 operator-() const;
    Vec3 operator-(const Vec3& other) const;
    Vec3 operator*(float s) const;
    Vec3 operator/(float s) const;
};

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

class ConvexShape {
public:
    // Returns the furthest point along a given direction inside the shape
    // i.e. returns P with dot(direction, P) = max{dot(direction, V) : V in Shape},
    // a "supporting point" of the shape for a given direction.
    // It may not be unique.
    virtual Vec3 support(const Vec3& direction) const = 0;
};

class ConvexPolyhedron : public ConvexShape {
public:
    ConvexPolyhedron(std::vector<Vec3> vertices)
        : vertices_(std::move(vertices))
    {
    }

    Vec3 support(const Vec3& direction) const override;

private:
    std::vector<Vec3> vertices_;
};

class Sphere : public ConvexShape {
public:
    Sphere(float radius)
        : radius_(radius)
    {
    }

    Vec3 support(const Vec3& direction) const override;

private:
    float radius_;
};

struct Collider {
public:
    Collider() = default;

    void addShape(std::unique_ptr<ConvexShape> shape, Mat4 transform);

    template <typename T, typename... Args>
    void addShape(Mat4 transform, Args&&... args)
    {
        addShape(std::make_unique<T>(std::forward<Args>(args)...), std::move(transform));
    }

    void setTransform(const Mat4& transform);
    Vec3 support(const Vec3& direction) const;

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
};

struct Collision {
    Vec3 collisionNormal;
    float penetrationDepth;
};

bool testCollision(const Collider& a, const Collider& b);
std::optional<Collision> getCollision(const Collider& a, const Collider& b);
}
