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

std::string str(const float v[3])
{
    return fmt::format("{{{}, {}, {}}}", v[0], v[1], v[2]);
}

void print(const wuzy_gjk_debug_iteration& it)
{
    fmt::print("direction: {}\n", str(it.direction));
    fmt::print("a_support: {}\n", str(it.a_support));
    fmt::print("b_support: {}\n", str(it.b_support));
    fmt::print("support: {}\n", str(it.support));
    fmt::print("simplex ({}): {{\n", it.simplex.num_vertices);
    for (size_t i = 0; i < it.simplex.num_vertices; ++i) {
        fmt::print("  {},\n", str(it.simplex.vertices[i]));
    }
    fmt::print("}}\n");
    fmt::print("contains_origin: {}\n", it.contains_origin);
}

glm::vec3 make_vec3(const float v[3])
{
    return { v[0], v[1], v[2] };
}

int main()
{
    const auto window = glwx::makeWindow("Wuzy Debug", 1920, 1080).value();
    glw::State::instance().setViewport(window.getSize().x, window.getSize().y);

#ifndef NDEBUG
    glwx::debug::init();
#endif

    DebugDraw debug_draw(8192);

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

    const std::vector<float> triangle_positions {
        // clang-format off
        -1.0f, -1.0f, -1.0f,
        -1.0f,  1.0f,  1.0f,
         1.0f,  1.0f, -1.0f,
        // clang-format on
    };
    const std::vector<size_t> triangle_indices { 0, 1, 2 };

    wuzy::ConvexPolyhedronCollider collider_a(triangle_positions, triangle_indices);

    const auto sphere_radius = 0.8f;
    wuzy::SphereCollider collider_b(sphere_radius);
    // const auto trafo = glwx::Transform(glm::vec3(0.1f, 0.0f, 0.5f));
    const auto trafo = glwx::Transform(glm::vec3(0.0f, 0.0f, 0.0f));
    collider_b.set_transform(trafo.getMatrix());

    std::vector<glm::vec3> a_support;
    std::vector<glm::vec3> b_support;
    std::vector<glm::vec3> minkowski_difference;
    const auto num_md_stacks = 32;
    const auto num_md_slices = 32;
    for (size_t stack = 0; stack < num_md_stacks; ++stack) {
        const float stack_angle = glm::pi<float>() / (num_md_stacks - 1) * stack;
        for (size_t slice = 0; slice < num_md_slices; ++slice) {
            const auto slice_angle = 2.0f * glm::pi<float>() / num_md_slices * slice;
            const glm::vec3 dir = { 
                glm::cos(slice_angle) * glm::sin(stack_angle),
                glm::cos(stack_angle), 
                glm::sin(slice_angle) * glm::sin(stack_angle),
            };
            const glm::vec3 ndir = { -dir.x, -dir.y, -dir.z };

            a_support.push_back(collider_a.support(dir));
            b_support.push_back(collider_b.support(dir));
            minkowski_difference.push_back(collider_a.support(dir) - collider_b.support(ndir));
        }
    }

    wuzy_gjk_debug gjk_debug = {};
    const auto gjk_res = wuzy::gjk(collider_a, collider_b, &gjk_debug);
    size_t debug_it = 0;
    print(gjk_debug.iterations[debug_it]);

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
                    if (debug_it < gjk_debug.num_iterations - 1) {
                        debug_it++;
                        fmt::print("iteration: {}\n", debug_it);
                        print(gjk_debug.iterations[debug_it]);
                    }
                    break;
                case SDLK_MINUS:
                    if (debug_it > 0) {
                        debug_it--;
                        fmt::print("iteration: {}\n", debug_it);
                        print(gjk_debug.iterations[debug_it]);
                    }
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
        const auto vel = move * 2.0f * dt;

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
        debug_draw.diamond(glm::vec4(1.0f, 1.0f, 0.0f, 1.0f), glm::vec3(0.0f), 0.01f); // yellow

        for (const auto& p : a_support) {
            debug_draw.diamond(
                glm::vec4(0.65f, 0.54f, 0.08f, 0.5f), p, 0.007f); // dark yellow/brown
        }

        for (const auto& p : b_support) {
            debug_draw.diamond(glm::vec4(0.09f, 0.57f, 0.26f, 0.5f), p, 0.007f); // dark green
        }

        for (const auto& p : minkowski_difference) {
            debug_draw.diamond(glm::vec4(0.5f, 0.5f, 0.5f, 0.5f), p, 0.007f); // dark grey
        }

        // shapes

        debug_draw.sphere( // light blue
            glm::vec4(0.32f, 0.7f, 0.9f, 1.0f), trafo.getPosition(), sphere_radius, 32, 32);
        debug_draw.lines(glm::vec4(0.6f, 0.6f, 0.1f, 1.0f), // yellow/green
            {
                make_vec3(&triangle_positions[0]),
                make_vec3(&triangle_positions[1]),
                make_vec3(&triangle_positions[1]),
                make_vec3(&triangle_positions[2]),
                make_vec3(&triangle_positions[2]),
                make_vec3(&triangle_positions[0]),
            });

        const auto& it = gjk_debug.iterations[debug_it];

        debug_draw.arrow(glm::vec4(0.0f, 1.0f, 1.0f, 1.0f), glm::vec3(0.0f),
            make_vec3(it.direction) * 0.5f); // turquoise

        const auto simplex_color = glm::vec4(1.0f); // white

        for (size_t i = 0; i < it.simplex.num_vertices; ++i) {
            debug_draw.diamond(simplex_color, make_vec3(it.simplex.vertices[i]), 0.01f);
        }

        if (it.simplex.num_vertices == 2) {
            // clang-format off
            debug_draw.lines(simplex_color,
                {
                    make_vec3(it.simplex.vertices[0]), make_vec3(it.simplex.vertices[1]),
                });
            // clang-format on
        } else if (it.simplex.num_vertices == 3) {
            // clang-format off
            debug_draw.lines(simplex_color,
                {
                    make_vec3(it.simplex.vertices[0]), make_vec3(it.simplex.vertices[1]),
                    make_vec3(it.simplex.vertices[1]), make_vec3(it.simplex.vertices[2]),
                    make_vec3(it.simplex.vertices[2]), make_vec3(it.simplex.vertices[0]),
                });
            // clang-format on
        } else if (it.simplex.num_vertices == 4) {
            // clang-format off
            debug_draw.lines(simplex_color,
                {
                    make_vec3(it.simplex.vertices[0]), make_vec3(it.simplex.vertices[1]),
                    make_vec3(it.simplex.vertices[1]), make_vec3(it.simplex.vertices[2]),
                    make_vec3(it.simplex.vertices[2]), make_vec3(it.simplex.vertices[0]),

                    make_vec3(it.simplex.vertices[0]), make_vec3(it.simplex.vertices[3]),
                    make_vec3(it.simplex.vertices[1]), make_vec3(it.simplex.vertices[3]),
                    make_vec3(it.simplex.vertices[2]), make_vec3(it.simplex.vertices[3]),
                });
            // clang-format on
        }

        debug_draw.diamond(glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), make_vec3(it.a_support), 0.005f); // red
        debug_draw.diamond(glm::vec4(0.0f, 1.0f, 1.0f, 1.0f), make_vec3(it.b_support), 0.005f); // turq
        debug_draw.diamond(glm::vec4(0.0f, 0.0f, 1.0f, 1.0f), make_vec3(it.support), 0.005f); // blue

        window.swap();
    }

    return 0;
}
