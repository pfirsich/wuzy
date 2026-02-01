// This was generated with Claude and only verified very superficially,
// because I want to have tests, but I don't want to write them.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <cmath>
#include <cstdlib>
#include <random>

#include "wuzy/wuzy.h"

// Helper functions for vec3 math
static float dot(const float a[3], const float b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static float length(const float v[3])
{
    return std::sqrt(dot(v, v));
}

static void normalize(float v[3])
{
    const float len = length(v);
    if (len > 0.0f) {
        v[0] /= len;
        v[1] /= len;
        v[2] /= len;
    }
}

static void sub(float out[3], const float a[3], const float b[3])
{
    out[0] = a[0] - b[0];
    out[1] = a[1] - b[1];
    out[2] = a[2] - b[2];
}

static void add(float out[3], const float a[3], const float b[3])
{
    out[0] = a[0] + b[0];
    out[1] = a[1] + b[1];
    out[2] = a[2] + b[2];
}

static void scale(float out[3], const float v[3], float s)
{
    out[0] = v[0] * s;
    out[1] = v[1] * s;
    out[2] = v[2] * s;
}

// Returns distance from point to capsule surface (negative if inside)
// Capsule defined by half_up and radius, centered at origin
static float distance_to_capsule_surface(const float point[3], const float half_up[3], float radius)
{
    const float half_length = length(half_up);

    if (half_length < 1e-6f) {
        // Degenerate: just a sphere
        return length(point) - radius;
    }

    // Normalized axis
    float axis[3];
    scale(axis, half_up, 1.0f / half_length);

    // Project point onto axis
    const float proj = dot(point, axis);

    if (proj > half_length) {
        // Above top cap - distance to top sphere center
        float to_top[3];
        sub(to_top, point, half_up);
        return length(to_top) - radius;
    } else if (proj < -half_length) {
        // Below bottom cap - distance to bottom sphere center
        float neg_half_up[3];
        scale(neg_half_up, half_up, -1.0f);
        float to_bottom[3];
        sub(to_bottom, point, neg_half_up);
        return length(to_bottom) - radius;
    } else {
        // On cylinder body - distance to axis
        float proj_vec[3];
        scale(proj_vec, axis, proj);
        float perp[3];
        sub(perp, point, proj_vec);
        return length(perp) - radius;
    }
}

// Check if normal points outward from capsule at the given point
static bool normal_points_outward(const float point[3], const float normal[3],
    const float half_up[3], float radius)
{
    const float half_length = length(half_up);

    if (half_length < 1e-6f) {
        // Sphere case: normal should point same direction as point from origin
        return dot(point, normal) > 0.0f;
    }

    float axis[3];
    scale(axis, half_up, 1.0f / half_length);

    const float proj = dot(point, axis);

    if (proj > half_length) {
        // Top cap: normal should point away from top sphere center
        float from_center[3];
        sub(from_center, point, half_up);
        return dot(from_center, normal) > 0.0f;
    } else if (proj < -half_length) {
        // Bottom cap: normal should point away from bottom sphere center
        float neg_half_up[3];
        scale(neg_half_up, half_up, -1.0f);
        float from_center[3];
        sub(from_center, point, neg_half_up);
        return dot(from_center, normal) > 0.0f;
    } else {
        // Cylinder: normal should point away from axis
        float proj_vec[3];
        scale(proj_vec, axis, proj);
        float from_axis[3];
        sub(from_axis, point, proj_vec);
        return dot(from_axis, normal) > 0.0f;
    }
}

TEST_CASE("capsule support function")
{
    const float tolerance = 1e-5f;

    // Test capsule: vertical, half_length=1, radius=0.5
    wuzy_capsule_collider_userdata userdata;
    userdata.half_up[0] = 0.0f;
    userdata.half_up[1] = 1.0f;
    userdata.half_up[2] = 0.0f;
    userdata.radius = 0.5f;

    wuzy_collider collider;
    wuzy_capsule_collider_init(&collider, &userdata);

    std::mt19937 rng(42);
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);

    SUBCASE("random directions")
    {
        for (int i = 0; i < 100; ++i) {
            float dir[3] = { dist(rng), dist(rng), dist(rng) };

            // Skip near-zero directions
            if (length(dir) < 0.1f) continue;

            float sup[3];
            wuzy_capsule_collider_support(&userdata, dir, sup);

            // Support point should be on the surface
            float dist_to_surface = distance_to_capsule_surface(sup, userdata.half_up, userdata.radius);
            CHECK(std::abs(dist_to_surface) < tolerance);
        }
    }

    SUBCASE("axis-aligned directions")
    {
        // +Y direction: should hit top of capsule
        {
            float dir[3] = { 0.0f, 1.0f, 0.0f };
            float sup[3];
            wuzy_capsule_collider_support(&userdata, dir, sup);
            CHECK(sup[1] == doctest::Approx(1.5f).epsilon(tolerance));
        }

        // -Y direction: should hit bottom of capsule
        {
            float dir[3] = { 0.0f, -1.0f, 0.0f };
            float sup[3];
            wuzy_capsule_collider_support(&userdata, dir, sup);
            CHECK(sup[1] == doctest::Approx(-1.5f).epsilon(tolerance));
        }

        // +X direction: should hit cylinder side
        {
            float dir[3] = { 1.0f, 0.0f, 0.0f };
            float sup[3];
            wuzy_capsule_collider_support(&userdata, dir, sup);
            CHECK(sup[0] == doctest::Approx(0.5f).epsilon(tolerance));
        }
    }
}

TEST_CASE("capsule ray cast")
{
    const float tolerance = 1e-4f;

    // Test capsule: vertical, half_length=1, radius=0.5
    wuzy_capsule_collider_userdata userdata;
    userdata.half_up[0] = 0.0f;
    userdata.half_up[1] = 1.0f;
    userdata.half_up[2] = 0.0f;
    userdata.radius = 0.5f;

    wuzy_collider collider;
    wuzy_capsule_collider_init(&collider, &userdata);

    SUBCASE("ray hitting cylinder body")
    {
        // Ray from +X towards origin at y=0
        float start[3] = { 2.0f, 0.0f, 0.0f };
        float dir[3] = { -1.0f, 0.0f, 0.0f };
        wuzy_ray_cast_result result;

        bool hit = wuzy_capsule_collider_ray_cast(&userdata, start, dir, &result);
        CHECK(hit);

        if (hit) {
            // Hit position should be on surface
            float dist_to_surface = distance_to_capsule_surface(result.hit_position,
                userdata.half_up, userdata.radius);
            CHECK(std::abs(dist_to_surface) < tolerance);

            // hit_position should equal start + t * dir
            float expected[3];
            scale(expected, dir, result.t);
            add(expected, start, expected);
            CHECK(std::abs(result.hit_position[0] - expected[0]) < tolerance);
            CHECK(std::abs(result.hit_position[1] - expected[1]) < tolerance);
            CHECK(std::abs(result.hit_position[2] - expected[2]) < tolerance);

            // Normal should be normalized
            float normal_len = length(result.normal);
            CHECK(normal_len == doctest::Approx(1.0f).epsilon(tolerance));

            // Normal should point outward
            CHECK(normal_points_outward(result.hit_position, result.normal,
                userdata.half_up, userdata.radius));

            // For this specific case, normal should be roughly +X
            CHECK(result.normal[0] == doctest::Approx(1.0f).epsilon(tolerance));
        }
    }

    SUBCASE("ray hitting top cap")
    {
        // Ray from above towards origin
        float start[3] = { 0.0f, 3.0f, 0.0f };
        float dir[3] = { 0.0f, -1.0f, 0.0f };
        wuzy_ray_cast_result result;

        bool hit = wuzy_capsule_collider_ray_cast(&userdata, start, dir, &result);
        CHECK(hit);

        if (hit) {
            // Should hit at y = 1.5 (half_length + radius)
            CHECK(result.hit_position[1] == doctest::Approx(1.5f).epsilon(tolerance));

            // Normal should point up
            CHECK(result.normal[1] == doctest::Approx(1.0f).epsilon(tolerance));
        }
    }

    SUBCASE("ray hitting bottom cap")
    {
        // Ray from below towards origin
        float start[3] = { 0.0f, -3.0f, 0.0f };
        float dir[3] = { 0.0f, 1.0f, 0.0f };
        wuzy_ray_cast_result result;

        bool hit = wuzy_capsule_collider_ray_cast(&userdata, start, dir, &result);
        CHECK(hit);

        if (hit) {
            // Should hit at y = -1.5
            CHECK(result.hit_position[1] == doctest::Approx(-1.5f).epsilon(tolerance));

            // Normal should point down
            CHECK(result.normal[1] == doctest::Approx(-1.0f).epsilon(tolerance));
        }
    }

    SUBCASE("ray missing capsule")
    {
        // Ray parallel to capsule, outside radius
        float start[3] = { 2.0f, 0.0f, 0.0f };
        float dir[3] = { 0.0f, 1.0f, 0.0f };
        wuzy_ray_cast_result result;

        bool hit = wuzy_capsule_collider_ray_cast(&userdata, start, dir, &result);
        CHECK_FALSE(hit);
    }

    SUBCASE("random rays from outside")
    {
        std::mt19937 rng(123);
        std::uniform_real_distribution<float> angle_dist(0.0f, 2.0f * 3.14159f);
        std::uniform_real_distribution<float> height_dist(-2.0f, 2.0f);

        int hits = 0;
        for (int i = 0; i < 100; ++i) {
            // Start from random point on a sphere around the capsule
            float angle = angle_dist(rng);
            float y = height_dist(rng);
            float start[3] = { 3.0f * std::cos(angle), y, 3.0f * std::sin(angle) };

            // Direction towards a random point near origin
            float target[3] = {
                0.3f * (angle_dist(rng) - 3.14159f),
                height_dist(rng) * 0.5f,
                0.3f * (angle_dist(rng) - 3.14159f)
            };
            float dir[3];
            sub(dir, target, start);
            normalize(dir);

            wuzy_ray_cast_result result;
            bool hit = wuzy_capsule_collider_ray_cast(&userdata, start, dir, &result);

            if (hit) {
                hits++;

                // Hit position should be on surface
                float dist_to_surface = distance_to_capsule_surface(result.hit_position,
                    userdata.half_up, userdata.radius);
                CHECK(std::abs(dist_to_surface) < tolerance);

                // hit_position should equal start + t * dir
                float expected[3];
                scale(expected, dir, result.t);
                add(expected, start, expected);
                CHECK(std::abs(result.hit_position[0] - expected[0]) < tolerance);
                CHECK(std::abs(result.hit_position[1] - expected[1]) < tolerance);
                CHECK(std::abs(result.hit_position[2] - expected[2]) < tolerance);

                // Normal should be normalized
                float normal_len = length(result.normal);
                CHECK(normal_len == doctest::Approx(1.0f).epsilon(tolerance));

                // Normal should point outward
                CHECK(normal_points_outward(result.hit_position, result.normal,
                    userdata.half_up, userdata.radius));

                // t should be positive (hit in front of ray)
                CHECK(result.t >= 0.0f);
            }
        }

        // Should have gotten at least some hits
        CHECK(hits > 10);
    }
}

TEST_CASE("capsule with non-axis-aligned orientation")
{
    const float tolerance = 1e-4f;

    // Capsule tilted 45 degrees in XY plane
    wuzy_capsule_collider_userdata userdata;
    float sqrt2_2 = std::sqrt(2.0f) / 2.0f;
    userdata.half_up[0] = sqrt2_2;
    userdata.half_up[1] = sqrt2_2;
    userdata.half_up[2] = 0.0f;
    userdata.radius = 0.3f;

    wuzy_collider collider;
    wuzy_capsule_collider_init(&collider, &userdata);

    SUBCASE("support along capsule axis")
    {
        float dir[3] = { sqrt2_2, sqrt2_2, 0.0f };
        float sup[3];
        wuzy_capsule_collider_support(&userdata, dir, sup);

        // Should be on surface
        float dist_to_surface = distance_to_capsule_surface(sup, userdata.half_up, userdata.radius);
        CHECK(std::abs(dist_to_surface) < tolerance);
    }

    SUBCASE("ray perpendicular to axis")
    {
        // Ray from +Z towards origin
        float start[3] = { 0.0f, 0.0f, 2.0f };
        float dir[3] = { 0.0f, 0.0f, -1.0f };
        wuzy_ray_cast_result result;

        bool hit = wuzy_capsule_collider_ray_cast(&userdata, start, dir, &result);
        CHECK(hit);

        if (hit) {
            float dist_to_surface = distance_to_capsule_surface(result.hit_position,
                userdata.half_up, userdata.radius);
            CHECK(std::abs(dist_to_surface) < tolerance);

            // Normal should point roughly in +Z
            CHECK(result.normal[2] > 0.9f);
        }
    }
}
