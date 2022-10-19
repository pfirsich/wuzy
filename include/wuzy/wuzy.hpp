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
};

struct Mat4 {
    std::array<Vec4, 4> cols;
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
    ConvexPolyhedron(const std::vector<Vec3> vertices);
    // May be sped up to O(log N), but we use a trivial linear search through all vertices
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
    void addShape(std::unique_ptr<ConvexShape> shape, const Mat4& transform);
    void setTransform(const Mat4& transform);
    Vec3 support(const Vec3& direction) const;

private:
    struct ColliderShape {
        std::unique_ptr<ConvexShape> shape;
        Mat4 transform;
    };

    std::vector<ColliderShape> shapes_;
    Mat4 transform_;
};

struct Collision { };

bool testCollision(const Collider& a, const Collider& b);
std::optional<Collision> getCollision(const Collider& a, const Collider& b);
}
