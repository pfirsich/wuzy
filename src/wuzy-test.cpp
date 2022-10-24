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

const auto vert = R"(
    #version 330 core
    uniform mat4 modelMatrix;
    uniform mat4 viewMatrix;
    uniform mat4 projectionMatrix;
    uniform mat3 normalMatrix;
    layout (location = 0) in vec3 attrPosition;
    layout (location = 1) in vec2 attrTexCoords;
    layout (location = 2) in vec3 attrNormal;
    out vec2 texCoords;
    out vec3 normal; // view space
    void main() {
        texCoords = attrTexCoords;
        normal = normalMatrix * attrNormal;
        gl_Position = projectionMatrix * viewMatrix * modelMatrix * vec4(attrPosition, 1.0);
    }
)"s;

const auto frag = R"(
    #version 330 core
    uniform sampler2D albedo;
    uniform vec3 lightDir; // view space
    uniform vec4 color;
    in vec2 texCoords;
    in vec3 normal;
    out vec4 fragColor;
    void main() {
        vec4 base = texture2D(albedo, texCoords);
        float nDotL = max(dot(lightDir, normal), 0.0);
        fragColor = color * vec4(base.rgb * nDotL + vec3(0.1), 1.0);
        // fragColor = vec4(normal, 1.0);
    }
)"s;

void drawMesh(const glwx::Mesh& mesh, const glm::vec4& color, const glw::Texture& texture,
    const glwx::Transform& trafo, const glm::mat4& viewMatrix, const glm::mat4& projectionMatrix)
{
    static const auto prog
        = glwx::makeShaderProgram(std::string_view(vert), std::string_view(frag)).value();

    texture.bind(0);

    const auto modelMatrix = trafo.getMatrix();
    const auto modelViewMatrix = viewMatrix * modelMatrix;
    const auto normalMatrix = glm::mat3(glm::transpose(glm::inverse(modelViewMatrix)));

    prog.bind();
    prog.setUniform("albedo", 0);
    prog.setUniform("modelMatrix", modelMatrix);
    prog.setUniform("viewMatrix", viewMatrix);
    prog.setUniform("projectionMatrix", projectionMatrix);
    prog.setUniform("normalMatrix", normalMatrix);
    prog.setUniform("lightDir", glm::vec3(0.0f, 0.0f, 1.0f));
    prog.setUniform("color", color);

    mesh.draw();
}

glm::vec3 vec3(const Vec3& v)
{
    return glm::vec3(v.x, v.y, v.z);
}

int main()
{
    const auto window = glwx::makeWindow("Wuzy Test", 1920, 1080).value();
    glw::State::instance().setViewport(window.getSize().x, window.getSize().y);

#ifndef NDEBUG
    glwx::debug::init();
#endif

    DebugDraw debugDraw;

    glw::VertexFormat vertFmt;
    vertFmt.add(0, 3, glw::AttributeType::F32);
    vertFmt.add(1, 2, glw::AttributeType::U16, true);
    vertFmt.add(2, 4, glw::AttributeType::IW2Z10Y10X10, true);

    const auto texture = glwx::makeTexture2D(glm::vec4(1.0f));

    const auto boxSize = 1.0f;
    const auto hBoxSize = boxSize / 2.0f;
    auto boxMesh = glwx::makeBoxMesh(vertFmt, { 0, 1, 2 }, boxSize, boxSize, boxSize);
    std::vector<Vec3> boxVertices = {
        Vec3 { -hBoxSize, -hBoxSize, -hBoxSize },
        Vec3 { hBoxSize, -hBoxSize, -hBoxSize },
        Vec3 { hBoxSize, hBoxSize, -hBoxSize },
        Vec3 { -hBoxSize, hBoxSize, -hBoxSize },

        Vec3 { -hBoxSize, -hBoxSize, hBoxSize },
        Vec3 { hBoxSize, -hBoxSize, hBoxSize },
        Vec3 { hBoxSize, hBoxSize, hBoxSize },
        Vec3 { -hBoxSize, hBoxSize, hBoxSize },
    };

    struct Obstacle {
        enum class Type { Box, Sphere };

        glwx::Transform trafo;
        Collider collider;
        Type type;
        bool collision = false;
    };

    auto randf = []() { return (rand() % 10000) / 10000.0f; };
    auto lerp = [](float t, float a, float b) { return a + (b - a) * t; };
    auto randfRange = [randf, lerp](float min, float max) { return lerp(randf(), min, max); };
    srand(42);

    std::vector<Obstacle> obstacles;
    const auto range = 8;
    for (size_t i = 0; i < 10; ++i) {
        const auto x = i == 0 ? hBoxSize * 2.0f + 0.1f : randf() * range * 2.0f - range;
        const auto z = i == 0 ? 0.0f : randf() * range * 2.0f - range;
        auto trafo = glwx::Transform(glm::vec3(x, 0.0f, z));
        /*trafo.setScale(
            glm::vec3(randfRange(0.5f, 1.5f), randfRange(0.5f, 1.5f), randfRange(0.5f, 1.5f)));
        trafo.setOrientation(
            glm::angleAxis(randf() * glm::two_pi<float>(), glm::vec3(1.0f, 0.0f, 0.0f))
            * glm::angleAxis(randf() * glm::two_pi<float>(), glm::vec3(0.0f, 1.0f, 0.0f))
            * glm::angleAxis(randf() * glm::two_pi<float>(), glm::vec3(0.0f, 0.0f, 1.0f)));*/
        const auto type = rand() % 2 == 0 ? Obstacle::Type::Box : Obstacle::Type::Sphere;
        Collider collider;
        if (type == Obstacle::Type::Box) {
            collider.addShape<ConvexPolyhedron>(Mat4 {}, boxVertices);
        } else if (type == Obstacle::Type::Sphere) {
            collider.addShape<Sphere>(Mat4 {}, hBoxSize);
        }
        collider.setTransform(glm::value_ptr(trafo.getMatrix()));
        obstacles.push_back(Obstacle { std::move(trafo), std::move(collider), type });
    }

    glwx::Transform playerTrafo;
    Collider playerCollider;
    bool playerCollision = false;
    const auto playerRadius = 0.5f;
    auto sphereMesh = glwx::makeSphereMesh(vertFmt, { 0, 1, 2 }, playerRadius, 32, 32);
    playerCollider.addShape<Sphere>(Mat4 {}, playerRadius);

    float cameraPitch = 0.0f, cameraYaw = 0.0f;
    glwx::Transform cameraTrafo;
    cameraTrafo.setPosition(glm::vec3(0.0f, 20.0f, 15.0f));
    cameraTrafo.lookAt(glm::vec3(0.0f));
    SDL_SetRelativeMouseMode(SDL_TRUE);

    const auto aspect = static_cast<float>(window.getSize().x) / window.getSize().y;
    glm::mat4 projectionMatrix = glm::perspective(glm::radians(45.0f), aspect, 0.1f, 100.0f);

    glEnable(GL_DEPTH_TEST);

    enum class InputMode { Player, Camera };
    InputMode inputMode = InputMode::Player;

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
                case SDLK_SPACE:
                    if (inputMode == InputMode::Player) {
                        inputMode = InputMode::Camera;
                        std::cout << "Input Mode: Camera\n";
                    } else if (inputMode == InputMode::Camera) {
                        inputMode = InputMode::Player;
                        std::cout << "Input Mode: Player\n";
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

        if (inputMode == InputMode::Player && glm::length(move) > 0.0f) {
            playerTrafo.move(vel);
            playerCollider.setTransform(glm::value_ptr(playerTrafo.getMatrix()));

            playerCollision = false;
            for (auto& obstacle : obstacles) {
                const auto col = getCollision(playerCollider, obstacle.collider);
                obstacle.collision = col.has_value();
                if (col) {
                    const auto mtv = -glm::vec3(col->normal.x, col->normal.y, col->normal.z)
                        * col->penetrationDepth * 1.0f;
                    playerTrafo.move(mtv);
                    playerCollider.setTransform(glm::value_ptr(playerTrafo.getMatrix()));
                    playerCollision = true;
                }
            }
        } else if (inputMode == InputMode::Camera) {
            cameraTrafo.moveLocal(vel);
            const auto look = glm::vec2(mouseX, mouseY) * 0.002f;
            cameraPitch
                = std::clamp(cameraPitch - look.y, -glm::half_pi<float>(), glm::half_pi<float>());
            cameraYaw += -look.x;
            const auto pitchQuat = glm::angleAxis(cameraPitch, glm::vec3(1.0f, 0.0f, 0.0f));
            const auto yawQuat = glm::angleAxis(cameraYaw, glm::vec3(0.0f, 1.0f, 0.0f));
            cameraTrafo.setOrientation(yawQuat * pitchQuat);
        }

        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        const auto viewMatrix = glm::inverse(cameraTrafo.getMatrix());
        debugDraw.setViewProjectionMatrix(projectionMatrix * viewMatrix);

        for (const auto& obstacle : obstacles) {
            const auto color
                = obstacle.collision ? glm::vec4(1.0f, 0.0f, 0.0f, 1.0f) : glm::vec4(1.0f);
            if (obstacle.type == Obstacle::Type::Box) {
                drawMesh(boxMesh, color, texture, obstacle.trafo, viewMatrix, projectionMatrix);
            } else if (obstacle.type == Obstacle::Type::Sphere) {
                drawMesh(sphereMesh, color, texture, obstacle.trafo, viewMatrix, projectionMatrix);
            }
            const auto aabb = obstacle.collider.getAabb();
            debugDraw.aabb(glm::vec4(1.0f), vec3(aabb.min), vec3(aabb.max));
        }

        const auto color = playerCollision ? glm::vec4(1.0f, 0.0f, 0.0f, 1.0f) : glm::vec4(1.0f);
        drawMesh(sphereMesh, color, texture, playerTrafo, viewMatrix, projectionMatrix);

        window.swap();
    }

    return 0;
}
