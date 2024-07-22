#include <iostream>
#include <set>
#include <span>

#include <glm/gtx/norm.hpp>
#include <glm/gtx/transform.hpp>
#include <glwx/aabb.hpp>
#include <glwx/debug.hpp>
#include <glwx/indexaccessor.hpp>
#include <glwx/meshgen.hpp>
#include <glwx/primitive.hpp>
#include <glwx/shader.hpp>
#include <glwx/texture.hpp>
#include <glwx/transform.hpp>
#include <glwx/vertexaccessor.hpp>
#include <glwx/window.hpp>

#include "debugdraw.hpp"
#include "wuzy/wuzy.hpp"

#define TINYOBJLOADER_IMPLEMENTATION // define this in only *one* .cc
#include "tiny_obj_loader.h"

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
    uniform mat4 viewMatrix;
    in vec2 texCoords;
    in vec3 normal;
    out vec4 fragColor;
    void main() {
        vec4 base = texture2D(albedo, texCoords);
        float nDotLCam = max(dot(lightDir, normal), 0.0);
        float nDotLWorld = max(dot(viewMatrix*normalize(vec4(0.5, 1.0, 0.5, 0.0)), vec4(normal, 0.0)), 0.0);
        fragColor = color * vec4(base.rgb * (nDotLCam * 0.2 + nDotLWorld * 0.8 + vec3(0.15)), 1.0);
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

Vec3 vec3(const glm::vec3& v)
{
    return Vec3 { v.x, v.y, v.z };
}

Vec3 vec3(const float* data)
{
    return Vec3 { data[0], data[1], data[2] };
}

glwx::Mesh getMesh(const tinyobj::ObjReader& reader, const glw::VertexFormat& vfmt)
{
    glwx::Mesh mesh;
    auto& vbuf = mesh.addVertexBuffer(vfmt, glw::Buffer::UsageHint::StaticDraw);

    const auto vertexCount = [&reader]() {
        size_t n = 0;
        for (const auto& shape : reader.GetShapes()) {
            for (const auto& num_face_vertices : shape.mesh.num_face_vertices) {
                n += num_face_vertices;
            }
        }
        return n;
    }();
    vbuf.resize(vertexCount);

    const auto& attrib = reader.GetAttrib();
    auto pAcc = glwx::VertexAccessor<glm::vec3>(vbuf, 0);
    auto tAcc = glwx::VertexAccessor<glm::vec2>(vbuf, 1);
    auto nAcc = glwx::VertexAccessor<glm::vec3>(vbuf, 2);
    size_t vertIdx = 0;
    for (const auto& shape : reader.GetShapes()) {
        const auto numFaces = shape.mesh.num_face_vertices.size();
        size_t indexOffset = 0;
        for (size_t f = 0; f < numFaces; ++f) {
            const auto numFaceVertices = shape.mesh.num_face_vertices[f];
            assert(numFaceVertices == 3);
            for (size_t v = 0; v < numFaceVertices; ++v) {
                assert(vertIdx < vertexCount);
                const auto& idx = shape.mesh.indices[indexOffset + v];
                pAcc[vertIdx] = glm::make_vec3(attrib.vertices.data() + 3 * idx.vertex_index);
                nAcc[vertIdx] = idx.normal_index >= 0
                    ? glm::make_vec3(attrib.normals.data() + 3 * idx.normal_index)
                    : glm::vec3(0.0f);
                tAcc[vertIdx] = idx.texcoord_index >= 0
                    ? glm::make_vec2(attrib.texcoords.data() + 2 * idx.texcoord_index)
                    : glm::vec2(0.0f);
                vertIdx++;
            }
            indexOffset += numFaceVertices;
        }
    }
    vbuf.update();
    mesh.primitive.vertexRange = { 0, vertexCount };
    return mesh;
}

std::vector<Collider> getColliders(const tinyobj::ObjReader& reader)
{
    std::vector<Collider> colliders;
    const auto vertData = reader.GetAttrib().vertices.data();
    const std::vector<std::tuple<size_t, size_t, size_t>> shapeIndices { { 0, 1, 2 } };
    for (const auto& shape : reader.GetShapes()) {
        const auto& indices = shape.mesh.indices;
        const auto numFaces = shape.mesh.num_face_vertices.size();
        size_t indexOffset = 0;
        for (size_t f = 0; f < numFaces; ++f) {
            const auto numFaceVertices = shape.mesh.num_face_vertices[f];
            assert(numFaceVertices == 3);
            const auto v0 = vec3(vertData + 3 * indices[indexOffset + 0].vertex_index);
            const auto v1 = vec3(vertData + 3 * indices[indexOffset + 1].vertex_index);
            const auto v2 = vec3(vertData + 3 * indices[indexOffset + 2].vertex_index);
            colliders.emplace_back().addShape<Triangle>(Mat4 {}, v0, v1, v2);
            indexOffset += numFaceVertices;
        }
    }
    return colliders;
}

bool movePlayer(AabbTree& broadphase, Collider& collider, glwx::Transform& trafo,
    glm::vec3& velocity, const glm::vec3& move, bool jump, float dt)
{
    constexpr float accel = 20.0f;
    constexpr float maxSpeed = 5.0f;
    constexpr float deccelTime = 0.25f;
    constexpr float gravity = 20.0f;
    constexpr float maxFallSpeed = 15.0f;
    constexpr float jumpHeight = 1.0f;

    const auto rayStart = vec3(trafo.getPosition());
    const auto rayDir = Vec3 { 0.0f, -1.0f, 0.0f };
    const auto rc = broadphase.rayCast(rayStart, rayDir);
    const auto onGround = rc && rc->first.t < 0.71f; // kind of controls walkable slope height
    if (onGround) {
        if (velocity.y < 0.0f) {
            velocity.y = 0.0f;
        }
        if (jump) {
            // v(t) = v0 - g*t => s(t) = v0*t - 0.5*g*t*t
            // v(T) = 0 <=> v0 = g*T
            // s(T) = v0*T - 0.5*g*T*T = v0*v0/g - 0.5*v0*v0/g = 0.5*v0*v0/g
            // <=> sqrt(2*s(T)*g) = v0
            velocity.y = glm::sqrt(2 * jumpHeight * gravity);
        }
    } else {
        velocity.y -= gravity * dt;
        if (velocity.y < -maxFallSpeed) {
            velocity.y = -maxFallSpeed;
        }
    }

    const auto localMove = trafo.getOrientation() * move;
    const auto xzMove = glm::vec3(localMove.x, 0.0f, localMove.z);
    const auto xzMoveLen = glm::length(xzMove);
    if (glm::length2(xzMove) > 0.0f) {
        velocity += xzMove / xzMoveLen * accel * dt;
        const auto velXZ = glm::vec3(velocity.x, 0.0f, velocity.z);
        const auto moveSpeed = glm::length(velXZ);
        if (moveSpeed > maxSpeed) {
            velocity *= maxSpeed / moveSpeed;
        }
    } else {
        const auto velXZ = glm::vec3(velocity.x, 0.0f, velocity.z);
        const auto moveSpeed = glm::length(velXZ);
        const auto deccel = maxSpeed / deccelTime * dt;
        if (moveSpeed > deccel) {
            velocity -= velXZ / moveSpeed * deccel;
        } else {
            velocity = glm::vec3(0.0f, velocity.y, 0.0f);
        }
    }

    trafo.move(velocity * dt);
    collider.setTransform(glm::value_ptr(trafo.getMatrix()));

    // This is kind of racey, because the broadphase query depends on the transform of the
    // player collider, which is changed in the loop.
    // In a real game, you would do this totally differently (many possible ways)!
    broadphase.update(&collider);
    bool collision = false;
    for (auto other : broadphase.query(collider.getAabb())) {
        if (other == &collider) {
            continue;
        }
        if (const auto col = getCollision(collider, *other)) {
            const auto mtv = -glm::vec3(col->normal.x, col->normal.y, col->normal.z)
                * col->penetrationDepth * 1.0f;
            trafo.move(mtv);
            collider.setTransform(glm::value_ptr(trafo.getMatrix()));
            collision = true;
        }
    }
    return collision;
}

glm::quat cameraLook(float& pitch, float& yaw, const glm::vec2& look)
{
    pitch = std::clamp(pitch - look.y, -glm::half_pi<float>(), glm::half_pi<float>());
    yaw += -look.x;
    const auto pitchQuat = glm::angleAxis(pitch, glm::vec3(1.0f, 0.0f, 0.0f));
    const auto yawQuat = glm::angleAxis(yaw, glm::vec3(0.0f, 1.0f, 0.0f));
    return yawQuat * pitchQuat;
}

int main()
{
    const auto window = glwx::makeWindow("Wuzy Test", 1920, 1080).value();
    glw::State::instance().setViewport(window.getSize().x, window.getSize().y);

#ifndef NDEBUG
    glwx::debug::init();
#endif

    DebugDraw debugDraw;

    const glw::VertexFormat vfmt {
        { 0, 3, glw::AttributeType::F32 },
        { 1, 2, glw::AttributeType::U16, true },
        { 2, 4, glw::AttributeType::IW2Z10Y10X10, true },
    };

    const auto texture = glwx::makeTexture2D(glm::vec4(1.0f));

    AabbTree broadphase;

    tinyobj::ObjReader reader;
    if (!reader.ParseFromFile("assets/meshes/quakey.obj", {})) {
        if (!reader.Error().empty()) {
            std::cerr << "TinyObjReader: " << reader.Error();
        }
        return 1;
    }

    if (!reader.Warning().empty()) {
        std::cout << "TinyObjReader: " << reader.Warning();
    }

    const auto level = getMesh(reader, vfmt);
    auto levelColliders = getColliders(reader);

    for (auto& collider : levelColliders) {
        broadphase.insert(&collider);
    }

    glwx::Transform playerTrafo;
    glm::vec3 playerVelocity = glm::vec3(0.0f);
    Collider playerCollider;
    const auto playerRadius = 0.5f;
    bool lastJump = false;
    auto sphereMesh = glwx::makeSphereMesh(vfmt, { 0, 1, 2 }, playerRadius, 32, 32);
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
    glEnable(GL_CULL_FACE);

    enum class InputMode { Camera, Fps };
    InputMode inputMode = InputMode::Fps;

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
                case SDLK_m:
                    if (inputMode == InputMode::Camera) {
                        inputMode = InputMode::Fps;
                        std::cout << "Input Mode: Fps\n";
                    } else if (inputMode == InputMode::Fps) {
                        inputMode = InputMode::Camera;
                        std::cout << "Input Mode: Player\n";
                    }
                    break;
                }
            }
        }

        const auto now = glwx::getTime();
        const auto dt = now - time;
        time = now;

        int mouseRelX = 0, mouseRelY = 0;
        SDL_GetRelativeMouseState(&mouseRelX, &mouseRelY);
        const auto sensitivity = 0.002f;
        const auto look = glm::vec2(mouseRelX, mouseRelY) * sensitivity;

        const auto kbState = SDL_GetKeyboardState(nullptr);
        const auto moveX = kbState[SDL_SCANCODE_D] - kbState[SDL_SCANCODE_A];
        const auto moveY = kbState[SDL_SCANCODE_R] - kbState[SDL_SCANCODE_F];
        const auto moveZ = kbState[SDL_SCANCODE_S] - kbState[SDL_SCANCODE_W];
        const auto jump = kbState[SDL_SCANCODE_SPACE] && !lastJump;
        lastJump = kbState[SDL_SCANCODE_SPACE];
        const auto moveVec = glm::vec3(moveX, moveY, moveZ);
        const auto move = glm::length(moveVec) > 0.0f ? glm::normalize(moveVec) : glm::vec3(0.0f);

        if (inputMode == InputMode::Fps) {
            playerTrafo.setOrientation(cameraLook(cameraPitch, cameraYaw, look));
            movePlayer(broadphase, playerCollider, playerTrafo, playerVelocity, move, jump, dt);
            cameraTrafo = playerTrafo;
        }
        if (inputMode == InputMode::Camera) {
            cameraTrafo.setOrientation(cameraLook(cameraPitch, cameraYaw, look));
            cameraTrafo.moveLocal(move * dt * 3.0f);
        }

        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        const auto viewMatrix = glm::inverse(cameraTrafo.getMatrix());
        debugDraw.setViewProjectionMatrix(projectionMatrix * viewMatrix);

        drawMesh(level, glm::vec4(0.8f), texture, {}, viewMatrix, projectionMatrix);

        drawMesh(sphereMesh, glm::vec4(1.0f), texture, playerTrafo, viewMatrix, projectionMatrix);

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
