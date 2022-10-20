#include <iostream>

#include <glwx/debug.hpp>
#include <glwx/meshgen.hpp>
#include <glwx/shader.hpp>
#include <glwx/texture.hpp>
#include <glwx/transform.hpp>
#include <glwx/window.hpp>

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
    const glwx::Transform& trafo, const glm::mat4& projectionMatrix,
    const glwx::Transform& cameraTrafo)
{
    static const auto prog
        = glwx::makeShaderProgram(std::string_view(vert), std::string_view(frag)).value();

    texture.bind(0);

    const auto modelMatrix = trafo.getMatrix();
    auto viewMatrix = cameraTrafo.getMatrix();
    viewMatrix
        = glm::lookAt(glm::vec3(0.0f, 20.0f, 1.0f), glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
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

int main()
{
    const auto window = glwx::makeWindow("Wuzy Test", 1024, 768).value();
    glw::State::instance().setViewport(window.getSize().x, window.getSize().y);

#ifndef NDEBUG
    glwx::debug::init();
#endif

    glw::VertexFormat vertFmt;
    vertFmt.add(0, 3, glw::AttributeType::F32);
    vertFmt.add(1, 2, glw::AttributeType::U16, true);
    vertFmt.add(2, 4, glw::AttributeType::IW2Z10Y10X10, true);

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

    const auto sphereRadius = 0.5f;
    auto sphereMesh = glwx::makeSphereMesh(vertFmt, { 0, 1, 2 }, sphereRadius, 32, 32);

    struct Obstacle {
        glwx::Transform trafo;
        Collider collider;
        bool collision = false;
    };

    auto randf = []() { return (rand() % 10000) / 10000.0f; };

    std::vector<Obstacle> obstacles;
    const auto range = 8;
    for (size_t i = 0; i < 8; ++i) {
        const auto x = i == 0 ? 0.0f : randf() * range * 2.0f - range;
        const auto z = i == 0 ? 0.0f : randf() * range * 2.0f - range;
        auto trafo = glwx::Transform(glm::vec3(x, 0.0f, z));
        Collider collider;
        collider.addShape<ConvexPolyhedron>(Mat4 {}, boxVertices);
        // collider.addShape<Sphere>(Mat4 {}, hBoxSize);
        collider.setTransform(glm::value_ptr(trafo.getMatrix()));
        obstacles.push_back(Obstacle { std::move(trafo), std::move(collider) });
    }

    glwx::Transform playerTrafo;
    Collider playerCollider;
    bool playerCollision = false;
    playerCollider.addShape<Sphere>(Mat4 {}, sphereRadius);

    const auto plainTexture = glwx::makeTexture2D(glm::vec4(1.0f));
    const auto checkerTexture = glwx::makeTexture2D(256, 256, 16);

    glwx::Transform cameraTrafo;

    const auto aspect = static_cast<float>(window.getSize().x) / window.getSize().y;
    glm::mat4 projectionMatrix = glm::perspective(glm::radians(45.0f), aspect, 0.1f, 100.0f);

    glEnable(GL_DEPTH_TEST);

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
                }
            }
        }

        const auto now = glwx::getTime();
        const auto dt = now - time;
        time = now;

        const auto kbState = SDL_GetKeyboardState(nullptr);
        const auto moveX = kbState[SDL_SCANCODE_D] - kbState[SDL_SCANCODE_A];
        const auto moveZ = kbState[SDL_SCANCODE_S] - kbState[SDL_SCANCODE_W];
        const auto move = glm::vec3(moveX, 0.0f, moveZ);
        if (move.x != 0.0f || move.z != 0.0f) {
            playerTrafo.move(move * 2.0f * dt);
            playerCollider.setTransform(glm::value_ptr(playerTrafo.getMatrix()));

            playerCollision = false;
            for (auto& obstacle : obstacles) {
                obstacle.collision = testCollision(playerCollider, obstacle.collider);
                playerCollision = playerCollision || obstacle.collision;
            }
        }

        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        for (const auto& obstacle : obstacles) {
            drawMesh(boxMesh,
                obstacle.collision ? glm::vec4(1.0f, 0.0f, 0.0f, 1.0f) : glm::vec4(1.0f),
                plainTexture, obstacle.trafo, projectionMatrix, cameraTrafo);
        }

        drawMesh(sphereMesh, playerCollision ? glm::vec4(1.0f, 0.0f, 0.0f, 1.0f) : glm::vec4(1.0f),
            plainTexture, playerTrafo, projectionMatrix, cameraTrafo);

        window.swap();
    }

    return 0;
}
