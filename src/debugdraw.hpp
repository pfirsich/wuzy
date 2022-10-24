#pragma once

#include <glwx/shader.hpp>
#include <glwx/texture.hpp>
#include <glwx/transform.hpp>
#include <glwx/window.hpp>

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

    void aabb(const glm::vec4& color, const glm::vec3& min, const glm::vec3& max)
    {
        // clang-format off
        lines(color, {
            // top
            { min.x, min.y, min.z }, { max.x, min.y, min.z },
            { max.x, min.y, min.z }, { max.x, min.y, max.z },
            { max.x, min.y, max.z }, { min.x, min.y, max.z },
            { min.x, min.y, max.z }, { min.x, min.y, min.z },

            // bottom
            { min.x, max.y, min.z }, { max.x, max.y, min.z },
            { max.x, max.y, min.z }, { max.x, max.y, max.z },
            { max.x, max.y, max.z }, { min.x, max.y, max.z },
            { min.x, max.y, max.z }, { min.x, max.y, min.z },

            // connecting lines
            { min.x, min.y, min.z }, { min.x, max.y, min.z },
            { max.x, min.y, min.z }, { max.x, max.y, min.z },
            { max.x, min.y, max.z }, { max.x, max.y, max.z },
            { min.x, min.y, max.z }, { min.x, max.y, max.z },
        });
        // clang-format on
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
