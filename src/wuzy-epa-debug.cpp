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

class DebugDraw {
public:
    DebugDraw(size_t maxNumVertices = 2048)
        : shader_(glwx::makeShaderProgram(vert, frag).value())
    {

        vertexArray_.bind();
        vertexBuffer_.bind(glw::Buffer::Target::Array);

        glw::VertexFormat fmt;
        fmt.add(0, 3, glw::AttributeType::F32);
        fmt.set();

        vertexArray_.unbind();
        vertexBuffer_.unbind(glw::Buffer::Target::Array);

        vertexBuffer_.data(glw::Buffer::Target::Array, glw::Buffer::UsageHint::StreamDraw, nullptr,
            sizeof(float) * 3 * maxNumVertices);
    }

    void lines(const glm::vec4& color, const std::vector<glm::vec3>& vertices)
    {
        vertexBuffer_.subData(glw::Buffer::Target::Array, vertices);

        shader_.bind();
        shader_.setUniform("viewProjectionMatrix", viewProjectionMatrix_);
        shader_.setUniform("color", color);

        vertexArray_.bind();
        glDrawArrays(GL_LINES, 0, vertices.size());

        vertexArray_.unbind();
        shader_.unbind();
    }

    void sphere(const glm::vec4& color, const glm::vec3& position, float radius, size_t slices,
        size_t stacks)
    {
        assert(slices >= 4);
        assert(stacks >= 3);

        const auto deltaStackAngle = glm::pi<float>() / (stacks - 1);
        const auto deltaSliceAngle = 2.0f * glm::pi<float>() / slices;

        std::vector<glm::vec3> points;
        for (size_t stack = 0; stack < stacks; ++stack) {
            const auto stackAngle = deltaStackAngle * stack;
            const auto xzRadius = glm::sin(stackAngle) * radius;
            const auto y = glm::cos(stackAngle) * radius;
            for (size_t slice = 0; slice < slices; ++slice) {
                const float sliceAngle = deltaSliceAngle * slice;
                // north-south line
                points.push_back(position
                    + glm::vec3(
                        glm::cos(sliceAngle) * xzRadius, y, glm::sin(sliceAngle) * xzRadius));
                points.push_back(position
                    + glm::vec3(
                        glm::cos(sliceAngle) * glm::sin(stackAngle + deltaStackAngle) * radius,
                        glm::cos(stackAngle + deltaStackAngle) * radius,
                        glm::sin(sliceAngle) * glm::sin(stackAngle + deltaStackAngle) * radius));
                if (stack != 0 && stack != stacks - 1) {
                    // west-east line
                    points.push_back(position
                        + glm::vec3(
                            glm::cos(sliceAngle) * xzRadius, y, glm::sin(sliceAngle) * xzRadius));
                    points.push_back(position
                        + glm::vec3(glm::cos(sliceAngle + deltaSliceAngle) * xzRadius, y,
                            glm::sin(sliceAngle + deltaSliceAngle) * xzRadius));
                }
            }
        }

        lines(color, points);
    }

    void diamond(const glm::vec4& color, const glm::vec3& position, float radius)
    {
        sphere(color, position, radius, 4, 3);
    }

    void arrow(const glm::vec4& color, const glm::vec3& start, const glm::vec3& end)
    {
        lines(color, { start, end });
    }

    void setViewProjectionMatrix(const glm::mat4& viewProjectionMatrix)
    {
        viewProjectionMatrix_ = viewProjectionMatrix;
    }

private:
    static constexpr std::string_view vert = R"(
        #version 150
        in vec3 position;
        uniform mat4 viewProjectionMatrix;

        void main()
        {
            gl_Position = viewProjectionMatrix * vec4(position, 1.0);
        }
    )";
    static constexpr std::string_view frag = R"(
        #version 150
        out vec4 fragColor;
        uniform vec4 color;

        void main()
        {
            fragColor = color;
        }
    )";

    glw::VertexArray vertexArray_;
    glw::Buffer vertexBuffer_;
    glw::ShaderProgram shader_;
    glm::mat4 viewProjectionMatrix_;
};

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

void print(const wuzy::detail::EpaDebug::Iteration& it)
{
    const auto& v = it.polytopeVertices;
    std::cout << "vertices: " << std::endl;
    for (size_t i = 0; i < v.size(); ++i) {
        std::cout << i << ": " << str(v[i]) << std::endl;
    }
    std::cout << "faces:" << std::endl;
    for (const auto& face : it.polytopeFaces) {
        std::cout << face.v0 << " " << str(v[face.v0]) << ", " << face.v1 << " " << str(v[face.v1])
                  << ", " << face.v2 << " " << str(v[face.v2]) << " - n: " << str(face.normal)
                  << ", d: " << face.dist << std::endl;
    }
    std::cout << "minDistFaceIdx: " << it.minDistFaceIdx << std::endl;
    std::cout << "minFaceDist: " << it.minFaceDist << std::endl;
    std::cout << "supPoint: " << str(it.supPoint) << std::endl;
    std::cout << "supDist: " << it.supDist << std::endl;
    std::cout << "removedFaces: ";
    bool first = true;
    for (const auto f : it.removedFaces) {
        if (!first) {
            std::cout << ", ";
        }
        first = false;
        std::cout << f;
    }
    std::cout << std::endl;
    std::cout << "edges to patch: " << std::endl;
    for (const auto [first, second] : it.edgesToPatch) {
        std::cout << first << ", " << second << std::endl;
    }
}

int main()
{
    const auto window = glwx::makeWindow("Wuzy Debug", 1920, 1080).value();
    glw::State::instance().setViewport(window.getSize().x, window.getSize().y);

#ifndef NDEBUG
    glwx::debug::init();
#endif

    DebugDraw debugDraw;

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

    const auto sphereRadius = 0.5f;

    Collider colliderA;
    colliderA.addShape<Sphere>(Mat4 {}, sphereRadius);

    Collider colliderB;
    colliderB.addShape<Sphere>(Mat4 {}, sphereRadius);
    const auto trafo = glwx::Transform(glm::vec3(2.0f * sphereRadius - 0.1f, 0.0f, 0.0f));
    colliderB.setTransform(glm::value_ptr(trafo.getMatrix()));

    std::vector<glm::vec3> minkowskiDifference;
    const auto numMdStacks = 32;
    const auto numMdSlices = 32;
    for (size_t stack = 0; stack < numMdStacks; ++stack) {
        const float stackAngle = glm::pi<float>() / (numMdStacks - 1) * stack;
        for (size_t slice = 0; slice < numMdSlices; ++slice) {
            const auto sliceAngle = 2.0f * glm::pi<float>() / numMdSlices * slice;
            const auto dir = Vec3 { glm::cos(sliceAngle) * glm::sin(stackAngle),
                glm::cos(stackAngle), glm::sin(sliceAngle) * glm::sin(stackAngle) };
            const auto sup = colliderA.support(dir) - colliderB.support(-dir);
            minkowskiDifference.push_back(vec3(sup));
        }
    }

    const auto gjkRes = wuzy::detail::gjk(colliderA, colliderB);
    assert(gjkRes.has_value());
    wuzy::detail::EpaDebug epaDebug;
    const auto epaRes = wuzy::detail::epa(colliderA, colliderB, *gjkRes, &epaDebug);
    size_t debugIt = 0;
    print(epaDebug.iterations[debugIt]);

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
                    if (debugIt < epaDebug.iterations.size() - 1) {
                        debugIt++;
                    }
                    std::cout << "iteration: " << debugIt << std::endl;
                    print(epaDebug.iterations[debugIt]);
                    break;
                case SDLK_MINUS:
                    if (debugIt > 0) {
                        debugIt--;
                    }
                    std::cout << "iteration: " << debugIt << std::endl;
                    print(epaDebug.iterations[debugIt]);
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

        for (const auto& p : minkowskiDifference) {
            debugDraw.diamond(glm::vec4(0.5f, 0.5f, 0.5f, 0.5f), p, 0.005f);
        }

        const auto& it = epaDebug.iterations[debugIt];
        for (size_t i = 0; i < it.polytopeFaces.size(); ++i) {
            const auto& face = it.polytopeFaces[i];
            const auto v0 = vec3(it.polytopeVertices[face.v0]);
            const auto v1 = vec3(it.polytopeVertices[face.v1]);
            const auto v2 = vec3(it.polytopeVertices[face.v2]);

            const auto isRemoved = std::find(it.removedFaces.begin(), it.removedFaces.end(), i)
                != it.removedFaces.end();
            const auto faceColor = isRemoved ? glm::vec4(0.5f, 1.0f, 1.0f, 1.0f) : glm::vec4(1.0f);
            debugDraw.lines(faceColor, { v0, v1, v2, v0 });

            const auto normalStart = (v0 + v1 + v2) / 3.0f;
            const auto normalEnd = normalStart + vec3(face.normal) * 0.1f;
            const auto normalColor = i == it.minDistFaceIdx ? glm::vec4(0.0f, 1.0f, 0.0f, 1.0f)
                                                            : glm::vec4(0.0f, 1.0f, 1.0f, 1.0f);
            debugDraw.arrow(normalColor, normalStart, normalEnd);
        }

        for (const auto [first, second] : it.edgesToPatch) {
            const auto v0 = it.polytopeVertices[first];
            const auto v1 = it.polytopeVertices[second];
            debugDraw.lines(glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), { vec3(v0), vec3(v1) });
        }

        debugDraw.diamond(glm::vec4(0.0f, 0.0f, 1.0f, 1.0f), vec3(it.supPoint), 0.005f);

        window.swap();
    }

    return 0;
}
