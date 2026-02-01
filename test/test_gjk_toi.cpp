// This was generated with Claude and only verified very superficially,
// because I want to have tests, but I don't want to write them.

#include "doctest.h"

#include <cmath>

#include "wuzy/wuzy.h"

static float length(const float v[3])
{
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

TEST_CASE("gjk_toi: sphere moving toward static sphere")
{
    const float tolerance = 1e-3f; // Binary search precision with 16 iterations

    // Sphere A at origin, radius 0.5
    wuzy_sphere_collider_userdata userdata_a;
    userdata_a.radius = 0.5f;
    wuzy_collider collider_a;
    wuzy_sphere_collider_init(&collider_a, &userdata_a);

    // Sphere B at (3, 0, 0), radius 0.5
    wuzy_sphere_collider_userdata userdata_b;
    userdata_b.radius = 0.5f;
    wuzy_collider collider_b;
    wuzy_sphere_collider_init(&collider_b, &userdata_b);

    // Move sphere B to (3, 0, 0)
    float transform_b[16] = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        3, 0, 0, 1  // translation in column 3
    };
    wuzy_collider_set_transform(&collider_b, transform_b);

    // Move sphere A toward sphere B with velocity (4, 0, 0)
    // Distance between centers: 3
    // Combined radii: 1
    // Distance to travel before contact: 3 - 1 = 2
    // With velocity magnitude 4, t = 2/4 = 0.5
    float velocity[3] = { 4.0f, 0.0f, 0.0f };
    float expected_t = 0.5f;

    float out_t;
    bool hit = wuzy_gjk_toi(&collider_a, &collider_b, velocity, 16, &out_t);

    CHECK(hit);
    CHECK(out_t == doctest::Approx(expected_t).epsilon(tolerance));
}

TEST_CASE("gjk_toi: no collision - sphere moving parallel (miss)")
{
    // Sphere A at origin, radius 0.5
    wuzy_sphere_collider_userdata userdata_a;
    userdata_a.radius = 0.5f;
    wuzy_collider collider_a;
    wuzy_sphere_collider_init(&collider_a, &userdata_a);

    // Sphere B at (0, 2, 0), radius 0.5
    wuzy_sphere_collider_userdata userdata_b;
    userdata_b.radius = 0.5f;
    wuzy_collider collider_b;
    wuzy_sphere_collider_init(&collider_b, &userdata_b);

    float transform_b[16] = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 2, 0, 1
    };
    wuzy_collider_set_transform(&collider_b, transform_b);

    // Move sphere A in X direction - will miss sphere B
    float velocity[3] = { 10.0f, 0.0f, 0.0f };

    float out_t;
    bool hit = wuzy_gjk_toi(&collider_a, &collider_b, velocity, 16, &out_t);

    CHECK_FALSE(hit);
}

TEST_CASE("gjk_toi: already overlapping at t=0")
{
    const float tolerance = 1e-5f;

    // Sphere A at origin, radius 1.0
    wuzy_sphere_collider_userdata userdata_a;
    userdata_a.radius = 1.0f;
    wuzy_collider collider_a;
    wuzy_sphere_collider_init(&collider_a, &userdata_a);

    // Sphere B at (0.5, 0, 0), radius 1.0 - overlapping with A
    wuzy_sphere_collider_userdata userdata_b;
    userdata_b.radius = 1.0f;
    wuzy_collider collider_b;
    wuzy_sphere_collider_init(&collider_b, &userdata_b);

    float transform_b[16] = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0.5f, 0, 0, 1
    };
    wuzy_collider_set_transform(&collider_b, transform_b);

    // Any velocity - should report t=0 since already overlapping
    float velocity[3] = { 1.0f, 0.0f, 0.0f };

    float out_t;
    bool hit = wuzy_gjk_toi(&collider_a, &collider_b, velocity, 16, &out_t);

    CHECK(hit);
    CHECK(out_t == doctest::Approx(0.0f).epsilon(tolerance));
}

TEST_CASE("gjk_toi: no collision - moving away from target")
{
    // Sphere A at origin, radius 0.5
    wuzy_sphere_collider_userdata userdata_a;
    userdata_a.radius = 0.5f;
    wuzy_collider collider_a;
    wuzy_sphere_collider_init(&collider_a, &userdata_a);

    // Sphere B at (2, 0, 0), radius 0.5
    wuzy_sphere_collider_userdata userdata_b;
    userdata_b.radius = 0.5f;
    wuzy_collider collider_b;
    wuzy_sphere_collider_init(&collider_b, &userdata_b);

    float transform_b[16] = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        2, 0, 0, 1
    };
    wuzy_collider_set_transform(&collider_b, transform_b);

    // Move sphere A away from sphere B (negative X)
    float velocity[3] = { -5.0f, 0.0f, 0.0f };

    float out_t;
    bool hit = wuzy_gjk_toi(&collider_a, &collider_b, velocity, 16, &out_t);

    CHECK_FALSE(hit);
}
