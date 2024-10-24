#include <fmt/format.h>

#include <glwx/debug.hpp>
#include <glwx/meshgen.hpp>
#include <glwx/shader.hpp>
#include <glwx/texture.hpp>
#include <glwx/transform.hpp>
#include <glwx/window.hpp>

#include "debugdraw.hpp"
#include "wuzy/wuzy.hpp"

using namespace std::string_literals;
using namespace wuzy;

glm::vec3 vec3(const wuzy_vec3& v)
{
    return glm::vec3(v.x, v.y, v.z);
}

wuzy_mat4 mat4(const glm::mat4& m)
{
    wuzy_mat4 r;
    std::memcpy(&r.cols[0], &m[0][0], sizeof(float) * 16);
    return r;
}

std::string str(const wuzy_vec3& v)
{
    return fmt::format("{{{}, {}, {}}}", v.x, v.y, v.z);
}

void print(const wuzy_epa_debug& debug, size_t it_idx)
{
    if (debug.num_iterations == 0) {
        fmt::print("no iterations\n");
    }
    const auto idx = std::min(it_idx, debug.num_iterations - 1);
    fmt::print("iteration {}\n", idx);
    const auto& it = debug.iterations[idx];
    const auto v = std::span(it.polytope_vertices, it.num_polytope_vertices);
    fmt::print("vertices ({}):\n", it.num_polytope_vertices);
    for (size_t i = 0; i < it.num_polytope_vertices; ++i) {
        fmt::print("{}: {}\n", i, str(v[i]));
    }
    fmt::print("faces ({}):\n", it.num_polytope_faces);
    for (const auto& face : std::span(it.polytope_faces, it.num_polytope_faces)) {
        fmt::print(
            "{}, {}, {} - n: {}, d: {}\n", face.v0, face.v1, face.v2, str(face.normal), face.dist);
    }
    fmt::print("min_dist_face_idx: {}\n", it.min_dist_face_index);
    fmt::print("min_face_dist: {}\n", it.min_face_dist);
    fmt::print("support_point: {}\n", str(it.support_point));
    fmt::print("support_dist: {}\n", it.support_dist);
    fmt::print("face_removed: ");
    bool first = true;
    for (size_t i = 0; i < it.num_polytope_faces; ++i) {
        if (it.face_removed[i]) {
            if (!first) {
                fmt::print(", ");
            }
            first = false;
            fmt::print("{}", i);
        }
    }
    fmt::print("\n");
    fmt::print("edges_to_patch ({}):\n", it.num_edges_to_patch);
    for (const auto [first, second] : std::span(it.edges_to_patch, it.num_edges_to_patch)) {
        fmt::print("{}, {}\n", first, second);
    }
}

// We calculate up to a fixed iteration because this is also used to debug crashes
void recalculate(const Collider& collider_a, const Collider& collider_b, wuzy_epa_debug& epa_debug,
    size_t num_iterations)
{
    wuzy_gjk_debug gjk_debug = {};
    const auto gjk_res = wuzy::gjk(collider_a, collider_b, &gjk_debug);
    assert(gjk_res.has_value());
    wuzy_gjk_debug_free(&gjk_debug);
    if (epa_debug.iterations) {
        wuzy_epa_debug_free(&epa_debug);
    }
    epa_debug.max_num_iterations = num_iterations;
    wuzy::epa(collider_a, collider_b, *gjk_res, &epa_debug);
    print(epa_debug, epa_debug.num_iterations - 1);
}

int main()
{
    const auto window = glwx::makeWindow("Wuzy EPA Debug", 1920, 1080).value();
    glw::State::instance().setViewport(window.getSize().x, window.getSize().y);

#ifndef NDEBUG
    glwx::debug::init();
#endif

    DebugDraw debug_draw;

    const auto plain_texture = glwx::makeTexture2D(glm::vec4(1.0f));
    const auto checker_texture = glwx::makeTexture2D(256, 256, 16);

    float camera_pitch = 0.0f, camera_yaw = 0.0f;
    glwx::Transform camera_trafo;
    camera_trafo.setPosition(glm::vec3(0.0f, 0.0f, 2.0f));
    SDL_SetRelativeMouseMode(SDL_TRUE);

    const auto aspect = static_cast<float>(window.getSize().x) / window.getSize().y;
    glm::mat4 projection_matrix = glm::perspective(glm::radians(45.0f), aspect, 0.1f, 100.0f);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    const auto sphere_radius = 0.5f;

    wuzy::SphereCollider collider_a(sphere_radius);
    const wuzy_mat4 transform_a {
        {
            wuzy_vec4 { .x = 0.645449042f, .y = 0.0f, .z = 0.763803363f, .w = 0.0f },
            wuzy_vec4 { .x = 0.13072744f, .y = 0.985244393f, .z = -0.110470697f, .w = 0.0f },
            wuzy_vec4 { .x = -0.752532959f, .y = 0.171153262f, .z = 0.635925055f, .w = 0.0f },
            wuzy_vec4 { .x = 2.2353394f, .y = -2.8240099f, .z = 0.950788021f, .w = 1.0f },
        },
    };
    collider_a.set_transform(transform_a);

    const wuzy_vec3 vertices[3] {
        wuzy_vec3 { .x = 2.27832198f, .y = -3.64938211f, .z = 1.01063895f },
        wuzy_vec3 { .x = 3.47832203f, .y = -1.24938095f, .z = 2.210639f },
        wuzy_vec3 { .x = 3.47832203f, .y = -1.24938095f, .z = 1.01063895f },
    };
    wuzy::TriangleCollider collider_b(vertices[0], vertices[1], vertices[2]);
    const wuzy_mat4 transform_b {
        {
            wuzy_vec4 { .x = 1.0f, .y = 0.0f, .z = 0.0f, .w = 0.0f },
            wuzy_vec4 { .x = 0.0f, .y = 1.0f, .z = 0.0f, .w = 0.0f },
            wuzy_vec4 { .x = 0.0f, .y = 0.0f, .z = 1.0f, .w = 0.0f },
            wuzy_vec4 { .x = 0.0f, .y = 0.0f, .z = 0.0f, .w = 1.0f },
        },
    };
    collider_b.set_transform(transform_b);

    std::vector<glm::vec3> minkowski_difference;
    const auto num_md_stacks = 32;
    const auto num_md_slices = 32;
    for (size_t stack = 0; stack < num_md_stacks; ++stack) {
        const float stack_angle = glm::pi<float>() / (num_md_stacks - 1) * stack;
        for (size_t slice = 0; slice < num_md_slices; ++slice) {
            const auto slice_angle = 2.0f * glm::pi<float>() / num_md_slices * slice;
            const auto dir = wuzy_vec3 { glm::cos(slice_angle) * glm::sin(stack_angle),
                glm::cos(stack_angle), glm::sin(slice_angle) * glm::sin(stack_angle) };
            const auto ndir = wuzy_vec3 { -dir.x, -dir.y, -dir.z };
            const auto sup = vec3(collider_a.support(dir)) - vec3(collider_b.support(ndir));
            minkowski_difference.push_back(sup);
        }
    }

    wuzy_epa_debug epa_debug = {};
    recalculate(collider_a, collider_b, epa_debug, 1);

    SDL_Event event;
    bool running = true;
    float time = glwx::getTime();
    while (running) {
        while (SDL_PollEvent(&event) != 0) {
            switch (event.type) {
            case SDL_QUIT:
                running = false;
                break;
            case SDL_KEYDOWN:
                switch (event.key.keysym.sym) {
                case SDLK_ESCAPE:
                    running = false;
                    break;
                case SDLK_PLUS:
                    recalculate(collider_a, collider_b, epa_debug, epa_debug.num_iterations + 1);
                    break;
                case SDLK_MINUS:
                    recalculate(collider_a, collider_b, epa_debug, epa_debug.num_iterations - 1);
                    break;
                }
            }
        }

        const auto now = glwx::getTime();
        const auto dt = now - time;
        time = now;

        int mouse_x = 0, mouse_y = 0;
        SDL_GetRelativeMouseState(&mouse_x, &mouse_y);

        const auto kb_state = SDL_GetKeyboardState(nullptr);
        const auto move_x = kb_state[SDL_SCANCODE_D] - kb_state[SDL_SCANCODE_A];
        const auto move_y = kb_state[SDL_SCANCODE_R] - kb_state[SDL_SCANCODE_F];
        const auto move_z = kb_state[SDL_SCANCODE_S] - kb_state[SDL_SCANCODE_W];
        const auto move = glm::vec3(move_x, move_y, move_z);
        const auto speed = kb_state[SDL_SCANCODE_LSHIFT] ? 0.2f : 2.0f;
        const auto vel = move * speed * dt;

        camera_trafo.moveLocal(vel);
        const auto look = glm::vec2(mouse_x, mouse_y) * 0.002f;
        camera_pitch
            = std::clamp(camera_pitch - look.y, -glm::half_pi<float>(), glm::half_pi<float>());
        camera_yaw += -look.x;
        const auto pitch_quat = glm::angleAxis(camera_pitch, glm::vec3(1.0f, 0.0f, 0.0f));
        const auto yaw_quat = glm::angleAxis(camera_yaw, glm::vec3(0.0f, 1.0f, 0.0f));
        camera_trafo.setOrientation(yaw_quat * pitch_quat);

        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        const auto view_matrix = glm::inverse(camera_trafo.getMatrix());
        debug_draw.set_view_projection_matrix(projection_matrix * view_matrix);

        // origin
        debug_draw.diamond(glm::vec4(1.0f, 1.0f, 0.0f, 1.0f), glm::vec3(0.0f), 0.005f); // yellow

        for (const auto& p : minkowski_difference) {
            debug_draw.diamond(glm::vec4(0.5f, 0.5f, 0.5f, 0.5f), p, 0.005f); // grey
        }

        const auto& it = epa_debug.iterations[epa_debug.num_iterations - 1];
        for (size_t i = 0; i < it.num_polytope_faces; ++i) {
            const auto& face = it.polytope_faces[i];
            const auto v0 = vec3(it.polytope_vertices[face.v0]);
            const auto v1 = vec3(it.polytope_vertices[face.v1]);
            const auto v2 = vec3(it.polytope_vertices[face.v2]);

            const auto face_color = it.face_removed[i] ? glm::vec4(0.5f, 1.0f, 1.0f, 1.0f) // teal
                                                       : glm::vec4(1.0f); // white
            debug_draw.lines(face_color, { v0, v1, v2, v0 });

            const auto normal_start = (v0 + v1 + v2) / 3.0f;
            const auto normal_end = normal_start + vec3(face.normal) * 0.1f;
            const auto normal_color = i == it.min_dist_face_index
                ? glm::vec4(0.0f, 1.0f, 0.0f, 1.0f) // green
                : glm::vec4(0.0f, 1.0f, 1.0f, 1.0f); // teal
            debug_draw.arrow(normal_color, normal_start, normal_end);
        }

        const auto edges_to_patch = std::span(it.edges_to_patch, it.num_edges_to_patch);
        for (const auto [first, second] : edges_to_patch) {
            const auto v0 = it.polytope_vertices[first];
            const auto v1 = it.polytope_vertices[second];
            debug_draw.lines(glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), { vec3(v0), vec3(v1) }); // red
        }

        debug_draw.diamond(
            glm::vec4(0.0f, 0.0f, 1.0f, 1.0f), vec3(it.support_point), 0.01f); // blue

        window.swap();
    }

    return 0;
}
