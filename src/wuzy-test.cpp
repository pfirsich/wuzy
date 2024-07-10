#include <iostream>
#include <span>

#include <glwx/debug.hpp>
#include <glwx/indexaccessor.hpp>
#include <glwx/meshgen.hpp>
#include <glwx/shader.hpp>
#include <glwx/texture.hpp>
#include <glwx/transform.hpp>
#include <glwx/vertexaccessor.hpp>
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

glwx::Mesh makeTriangleMesh(const glw::VertexFormat& vfmt, const glwx::AttributeLocations& loc,
    std::span<const glm::vec3> positions, std::span<const glm::vec2> texCoords,
    std::span<const glm::vec3> normals, std::span<const size_t> indices)
{
    glwx::Mesh mesh;

    auto& vbuf = mesh.addVertexBuffer(vfmt, glw::Buffer::UsageHint::StaticDraw);
    vbuf.resize(positions.size());

    assert(vfmt.get(loc.position));
    auto pAcc = glwx::VertexAccessor<glm::vec3>(vbuf, loc.position);
    for (size_t i = 0; i < positions.size(); ++i) {
        pAcc[i] = positions[i];
    }

    if (loc.normal) {
        assert(vfmt.get(*loc.normal));
        assert(normals.size() == positions.size());
        auto nAcc = glwx::VertexAccessor<glm::vec3>(vbuf, *loc.normal);
        for (size_t i = 0; i < normals.size(); ++i) {
            nAcc[i] = normals[i];
        }
    }

    if (loc.texCoords) {
        assert(vfmt.get(*loc.texCoords));
        assert(texCoords.size() == positions.size());
        auto tAcc = glwx::VertexAccessor<glm::vec2>(vbuf, *loc.texCoords);
        for (size_t i = 0; i < texCoords.size(); ++i) {
            tAcc[i] = texCoords[i];
        }
    }

    vbuf.update();

    auto& ibuf = mesh.addIndexBuffer(glw::IndexType::U8, glw::Buffer::UsageHint::StaticDraw);
    ibuf.resize(indices.size());
    auto iAcc = glwx::IndexAccessor(ibuf);
    for (size_t i = 0; i < indices.size(); ++i) {
        iAcc[i] = indices[i];
    }
    ibuf.update();

    mesh.primitive.indexRange = glwx::Primitive::Range { 0, indices.size() };

    return mesh;
}

std::vector<Vec3> to_wuzy(std::span<const glm::vec3> vs)
{
    std::vector<Vec3> ret;
    for (const auto& v : vs) {
        ret.emplace_back(Vec3 { v.x, v.y, v.z });
    }
    return ret;
}

std::vector<std::tuple<size_t, size_t, size_t>> to_wuzy(std::span<const size_t> is)
{
    assert(is.size() % 3 == 0);
    std::vector<std::tuple<size_t, size_t, size_t>> ret;
    for (size_t i = 0; i < is.size(); i += 3) {
        ret.emplace_back(std::tuple<size_t, size_t, size_t> { is[i + 0], is[i + 1], is[i + 2] });
    }
    return ret;
}

int main()
{
    const auto window = glwx::makeWindow("Wuzy Test", 1920, 1080).value();
    glw::State::instance().setViewport(window.getSize().x, window.getSize().y);

#ifndef NDEBUG
    glwx::debug::init();
#endif

    DebugDraw debugDraw;

    const glw::VertexFormat vertFmt {
        { 0, 3, glw::AttributeType::F32 },
        { 1, 2, glw::AttributeType::U16, true },
        { 2, 4, glw::AttributeType::IW2Z10Y10X10, true },
    };

    const auto texture = glwx::makeTexture2D(glm::vec4(1.0f));

    AabbTree broadphase;

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

    std::vector<std::tuple<size_t, size_t, size_t>> boxFaces = {
        { 3, 0, 4 }, { 3, 4, 7 }, // -x
        { 6, 5, 1 }, { 6, 1, 2 }, // +x
        { 4, 0, 1 }, { 4, 1, 5 }, // -y
        { 3, 7, 6 }, { 3, 6, 2 }, // +y
        { 2, 1, 0 }, { 2, 0, 3 }, // -z
        { 7, 4, 5 }, { 7, 5, 6 }, // +z
    };

    const std::vector<glm::vec3> trianglePositions {
        glm::vec3(-1.0f, -1.0f, -1.0f),
        glm::vec3(-1.0f, 1.0f, 1.0f),
        glm::vec3(1.0f, 1.0f, -1.0f),
    };

    const auto normal = glm::normalize(glm::cross(
        trianglePositions[1] - trianglePositions[0], trianglePositions[2] - trianglePositions[1]));
    const std::vector<glm::vec3> triangleNormals { normal, normal, normal };

    const std::vector<glm::vec2> triangleTexCoords {
        glm::vec2(0.0f, 0.0f),
        glm::vec2(1.0f, 0.0f),
        glm::vec2(0.0f, 1.0f),
    };

    const std::vector<size_t> triangleIndices { 0, 1, 2 };

    const auto triangleMesh
        = makeTriangleMesh(vertFmt, { .position = 0, .texCoords = 1, .normal = 2 },
            trianglePositions, triangleTexCoords, triangleNormals, triangleIndices);

    struct Obstacle {
        enum class Type { Box, Sphere, Triangle };

        glwx::Transform trafo;
        std::unique_ptr<Collider> collider; // need stable pointers for broadphase
        Type type;
        bool candidate = false;
        bool collision = false;
    };

    auto randf = []() { return (rand() % 10000) / 10000.0f; };
    auto lerp = [](float t, float a, float b) { return a + (b - a) * t; };
    auto randfRange = [randf, lerp](float min, float max) { return lerp(randf(), min, max); };
    srand(42);

    constexpr std::array<Obstacle::Type, 3> obstacleTypes {
        Obstacle::Type::Box,
        Obstacle::Type::Sphere,
        Obstacle::Type::Triangle,
    };

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
        const auto type = obstacleTypes[rand() % obstacleTypes.size()];
        auto collider = std::make_unique<Collider>();
        collider->userData = reinterpret_cast<void*>(obstacles.size());
        if (type == Obstacle::Type::Box) {
            collider->addShape<ConvexPolyhedron>(Mat4 {}, boxVertices, boxFaces);
        } else if (type == Obstacle::Type::Sphere) {
            collider->addShape<Sphere>(Mat4 {}, hBoxSize);
        } else if (type == Obstacle::Type::Triangle) {
            collider->addShape<ConvexPolyhedron>(
                Mat4 {}, to_wuzy(trianglePositions), to_wuzy(triangleIndices));
        }
        collider->setTransform(glm::value_ptr(trafo.getMatrix()));
        broadphase.insert(collider.get());
        obstacles.push_back(Obstacle { std::move(trafo), std::move(collider), type });
    }

    glwx::Transform playerTrafo;
    Collider playerCollider;
    bool playerCollision = false;
    const auto playerRadius = 0.5f;
    auto sphereMesh = glwx::makeSphereMesh(vertFmt, { 0, 1, 2 }, playerRadius, 32, 32);
    playerCollider.addShape<Sphere>(Mat4 {}, playerRadius);
    broadphase.insert(&playerCollider);

    broadphase.print();

    float cameraPitch = 0.0f, cameraYaw = 0.0f;
    glwx::Transform cameraTrafo;
    cameraTrafo.setPosition(glm::vec3(0.0f, 10.0f, 5.0f));
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
            playerCollision = false;
            for (auto& obstacle : obstacles) {
                obstacle.candidate = false;
                obstacle.collision = false;
            }

            playerTrafo.move(vel);
            playerCollider.setTransform(glm::value_ptr(playerTrafo.getMatrix()));

            // This is kind of racey, because the broadphase query depends on the transform of the
            // player collider, which is changed in the loop.
            // In a real game, you would do this totally differently (many possible ways)!
            broadphase.update(&playerCollider);
            for (auto collider : broadphase.query(playerCollider.getAabb())) {
                if (collider == &playerCollider) {
                    continue;
                }
                const auto obstacleIdx = reinterpret_cast<size_t>(collider->userData);
                auto& obstacle = obstacles[obstacleIdx];
                assert(obstacle.collider.get() == collider);
                obstacle.candidate = true;
                if (const auto col = getCollision(playerCollider, *obstacle.collider)) {
                    obstacle.collision = true;
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
            const auto color = [&obstacle]() {
                if (obstacle.collision) {
                    return glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
                } else if (obstacle.candidate) {
                    return glm::vec4(1.0f, 1.0f, 0.0f, 1.0f);
                } else {
                    return glm::vec4(1.0f);
                }
            }();
            if (obstacle.type == Obstacle::Type::Box) {
                drawMesh(boxMesh, color, texture, obstacle.trafo, viewMatrix, projectionMatrix);
            } else if (obstacle.type == Obstacle::Type::Sphere) {
                drawMesh(sphereMesh, color, texture, obstacle.trafo, viewMatrix, projectionMatrix);
            } else if (obstacle.type == Obstacle::Type::Triangle) {
                drawMesh(
                    triangleMesh, color, texture, obstacle.trafo, viewMatrix, projectionMatrix);
            }
            const auto aabb = obstacle.collider->getAabb();
            debugDraw.aabb(glm::vec4(1.0f), vec3(aabb.min), vec3(aabb.max));
        }

        const auto color = playerCollision ? glm::vec4(1.0f, 0.0f, 0.0f, 1.0f) : glm::vec4(1.0f);
        drawMesh(sphereMesh, color, texture, playerTrafo, viewMatrix, projectionMatrix);

        std::vector<glm::vec4> aabbColors {
            glm::vec4(1.0f, 0.0f, 0.0f, 1.0f),
            glm::vec4(0.0f, 1.0f, 0.0f, 1.0f),
            glm::vec4(0.0f, 0.0f, 1.0f, 1.0f),
            glm::vec4(1.0f, 1.0f, 0.0f, 1.0f),
            glm::vec4(1.0f, 0.0f, 1.0f, 1.0f),
            glm::vec4(0.0f, 1.0f, 1.0f, 1.0f),
            glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
        };
        for (const auto& [aabb, depth] : broadphase.getAabbs()) {
            const auto& color = aabbColors[depth % aabbColors.size()];
            debugDraw.aabb(color, vec3(aabb.min), vec3(aabb.max));
        }

        const auto camPos = cameraTrafo.getPosition();
        const auto camFwd = cameraTrafo.getForward();
        const auto rayStart = Vec3 { camPos.x, camPos.y, camPos.z };
        const auto rayDir = Vec3 { camFwd.x, camFwd.y, camFwd.z };
        const auto rc = broadphase.rayCast(rayStart, rayDir);
        if (rc) {
            const auto [res, collider] = *rc;
            const auto markerPos = camPos + res.t * camFwd;
            debugDraw.diamond(glm::vec4(0.0f, 1.0f, 0.0f, 1.0f), markerPos, 0.05f);
            debugDraw.arrow(
                glm::vec4(1.0f, 1.0f, 0.0f, 1.0f), markerPos, markerPos + vec3(res.normal) * 0.2f);
        }

        window.swap();
    }

    return 0;
}
