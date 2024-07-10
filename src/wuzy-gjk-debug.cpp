#include <iostream>

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

glm::vec3 vec3(const Vec3& v)
{
    return glm::vec3(v.x, v.y, v.z);
}

std::string str(const Vec3& v)
{
    char buf[64];
    std::sprintf(buf, "{%f, %f, %f}", v.x, v.y, v.z);
    return buf;
}

void print(const detail::GjkDebug::Iteration& it)
{
    std::cout << "direction: " << str(it.direction) << std::endl;
    std::cout << "aSupport: " << str(it.aSupport) << std::endl;
    std::cout << "bSupport: " << str(it.bSupport) << std::endl;
    std::cout << "support: " << str(it.support) << std::endl;
    std::cout << "simplex: {" << std::endl;
    for (size_t i = 0; i < it.simplex.size(); ++i) {
        std::cout << "  " << str(it.simplex[i]) << "," << std::endl;
    }
    std::cout << "}" << std::endl;
    std::cout << "containsOrigin: " << it.containsOrigin << std::endl;
}

int main()
{
    const auto window = glwx::makeWindow("Wuzy Debug", 1920, 1080).value();
    glw::State::instance().setViewport(window.getSize().x, window.getSize().y);

#ifndef NDEBUG
    glwx::debug::init();
#endif

    DebugDraw debugDraw(8192);

    const auto plainTexture = glwx::makeTexture2D(glm::vec4(1.0f));
    const auto checkerTexture = glwx::makeTexture2D(256, 256, 16);

    float cameraPitch = 0.0f, cameraYaw = 0.0f;
    glwx::Transform cameraTrafo;
    cameraTrafo.setPosition(glm::vec3(0.0f, 0.0f, 2.0f));
    SDL_SetRelativeMouseMode(SDL_TRUE);

    const auto aspect = static_cast<float>(window.getSize().x) / window.getSize().y;
    glm::mat4 projectionMatrix = glm::perspective(glm::radians(45.0f), aspect, 0.1f, 100.0f);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    const std::vector<Vec3> trianglePositions {
        Vec3 { -1.0f, -1.0f, -1.0f },
        Vec3 { -1.0f, 1.0f, 1.0f },
        Vec3 { 1.0f, 1.0f, -1.0f },
    };
    const std::vector<std::tuple<size_t, size_t, size_t>> triangleIndices { { 0, 1, 2 } };

    Collider colliderA;
    colliderA.addShape<ConvexPolyhedron>(Mat4 {}, trianglePositions, triangleIndices);

    Collider colliderB;
    const auto sphereRadius = 0.8f;
    colliderB.addShape<Sphere>(Mat4 {}, sphereRadius);
    // const auto trafo = glwx::Transform(glm::vec3(0.1f, 0.0f, 0.5f));
    const auto trafo = glwx::Transform(glm::vec3(0.0f, 0.0f, 0.0f));
    colliderB.setTransform(glm::value_ptr(trafo.getMatrix()));

    std::vector<glm::vec3> aSupport;
    std::vector<glm::vec3> bSupport;
    std::vector<glm::vec3> minkowskiDifference;
    const auto numMdStacks = 32;
    const auto numMdSlices = 32;
    for (size_t stack = 0; stack < numMdStacks; ++stack) {
        const float stackAngle = glm::pi<float>() / (numMdStacks - 1) * stack;
        for (size_t slice = 0; slice < numMdSlices; ++slice) {
            const auto sliceAngle = 2.0f * glm::pi<float>() / numMdSlices * slice;
            const auto dir = Vec3 { glm::cos(sliceAngle) * glm::sin(stackAngle),
                glm::cos(stackAngle), glm::sin(sliceAngle) * glm::sin(stackAngle) };

            aSupport.push_back(vec3(colliderA.support(dir)));
            bSupport.push_back(vec3(colliderB.support(dir)));
            minkowskiDifference.push_back(vec3(colliderA.support(dir) - colliderB.support(-dir)));
        }
    }

    detail::GjkDebug gjkDebug;
    const auto gjkRes = detail::gjk(colliderA, colliderB, &gjkDebug);
    size_t debugIt = 0;
    print(gjkDebug.iterations[debugIt]);

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
                    if (debugIt < gjkDebug.iterations.size() - 1) {
                        debugIt++;
                        std::cout << "iteration: " << debugIt << std::endl;
                        print(gjkDebug.iterations[debugIt]);
                    }
                    break;
                case SDLK_MINUS:
                    if (debugIt > 0) {
                        debugIt--;
                        std::cout << "iteration: " << debugIt << std::endl;
                        print(gjkDebug.iterations[debugIt]);
                    }
                    break;
                }
            }
        }

        const auto now = glwx::getTime();
        const auto dt = now - time;
        time = now;

        int mouseX = 0, mouseY = 0;
        SDL_GetRelativeMouseState(&mouseX, &mouseY);

        const auto kbState = SDL_GetKeyboardState(nullptr);
        const auto moveX = kbState[SDL_SCANCODE_D] - kbState[SDL_SCANCODE_A];
        const auto moveY = kbState[SDL_SCANCODE_R] - kbState[SDL_SCANCODE_F];
        const auto moveZ = kbState[SDL_SCANCODE_S] - kbState[SDL_SCANCODE_W];
        const auto move = glm::vec3(moveX, moveY, moveZ);
        const auto vel = move * 2.0f * dt;

        cameraTrafo.moveLocal(vel);
        const auto look = glm::vec2(mouseX, mouseY) * 0.002f;
        cameraPitch
            = std::clamp(cameraPitch - look.y, -glm::half_pi<float>(), glm::half_pi<float>());
        cameraYaw += -look.x;
        const auto pitchQuat = glm::angleAxis(cameraPitch, glm::vec3(1.0f, 0.0f, 0.0f));
        const auto yawQuat = glm::angleAxis(cameraYaw, glm::vec3(0.0f, 1.0f, 0.0f));
        cameraTrafo.setOrientation(yawQuat * pitchQuat);

        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        const auto viewMatrix = glm::inverse(cameraTrafo.getMatrix());
        debugDraw.setViewProjectionMatrix(projectionMatrix * viewMatrix);

        // origin
        debugDraw.diamond(glm::vec4(1.0f, 1.0f, 0.0f, 1.0f), glm::vec3(0.0f), 0.01f);

        for (const auto& p : aSupport) {
            debugDraw.diamond(glm::vec4(0.65f, 0.54f, 0.08f, 0.5f), p, 0.007f);
        }

        for (const auto& p : bSupport) {
            debugDraw.diamond(glm::vec4(0.09f, 0.57f, 0.26f, 0.5f), p, 0.007f);
        }

        for (const auto& p : minkowskiDifference) {
            debugDraw.diamond(glm::vec4(0.5f, 0.5f, 0.5f, 0.5f), p, 0.007f);
        }

        // shapes
        debugDraw.sphere(
            glm::vec4(0.32f, 0.7f, 0.9f, 1.0f), trafo.getPosition(), sphereRadius, 32, 32);
        debugDraw.lines(glm::vec4(0.6f, 0.6f, 0.1f, 1.0f),
            {
                vec3(trianglePositions[0]),
                vec3(trianglePositions[1]),
                vec3(trianglePositions[1]),
                vec3(trianglePositions[2]),
                vec3(trianglePositions[2]),
                vec3(trianglePositions[0]),
            });

        const auto& it = gjkDebug.iterations[debugIt];

        debugDraw.arrow(
            glm::vec4(0.0f, 1.0f, 1.0f, 1.0f), glm::vec3(0.0f), vec3(it.direction) * 0.5f);

        const auto simplexColor = glm::vec4(1.0f);

        for (size_t i = 0; i < it.simplex.size(); ++i) {
            debugDraw.diamond(simplexColor, vec3(it.simplex[i]), 0.01f);
        }

        if (it.simplex.size() == 2) {
            // clang-format off
            debugDraw.lines(simplexColor,
                {
                    vec3(it.simplex[0]), vec3(it.simplex[1]),
                });
            // clang-format on
        } else if (it.simplex.size() == 3) {
            // clang-format off
            debugDraw.lines(simplexColor,
                {
                    vec3(it.simplex[0]), vec3(it.simplex[1]),
                    vec3(it.simplex[1]), vec3(it.simplex[2]),
                    vec3(it.simplex[2]), vec3(it.simplex[0]),
                });
            // clang-format on
        } else if (it.simplex.size() == 4) {
            // clang-format off
            debugDraw.lines(simplexColor,
                {
                    vec3(it.simplex[0]), vec3(it.simplex[1]),
                    vec3(it.simplex[1]), vec3(it.simplex[2]),
                    vec3(it.simplex[2]), vec3(it.simplex[0]),

                    vec3(it.simplex[0]), vec3(it.simplex[3]),
                    vec3(it.simplex[1]), vec3(it.simplex[3]),
                    vec3(it.simplex[2]), vec3(it.simplex[3]),
                });
            // clang-format on
        }

        debugDraw.diamond(glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), vec3(it.aSupport), 0.005f);
        debugDraw.diamond(glm::vec4(0.0f, 1.0f, 1.0f, 1.0f), vec3(it.bSupport), 0.005f);
        debugDraw.diamond(glm::vec4(0.0f, 0.0f, 1.0f, 1.0f), vec3(it.support), 0.005f);

        window.swap();
    }

    return 0;
}
