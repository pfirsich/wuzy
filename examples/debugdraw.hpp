#pragma once

#include <glwx/shader.hpp>
#include <glwx/texture.hpp>
#include <glwx/transform.hpp>
#include <glwx/window.hpp>

class DebugDraw {
public:
    DebugDraw(size_t max_num_vertices = 2048) : shader_(glwx::makeShaderProgram(vert, frag).value())
    {
        vertex_array_.bind();
        vertex_buffer_.bind(glw::Buffer::Target::Array);

        glw::VertexFormat fmt;
        fmt.add(0, 3, glw::AttributeType::F32);
        fmt.set();

        vertex_array_.unbind();
        vertex_buffer_.unbind(glw::Buffer::Target::Array);

        vertex_buffer_.data(glw::Buffer::Target::Array, glw::Buffer::UsageHint::StreamDraw, nullptr,
            sizeof(float) * 3 * max_num_vertices);
    }

    void lines(const glm::vec4& color, const std::vector<glm::vec3>& vertices)
    {
        vertex_buffer_.subData(glw::Buffer::Target::Array, vertices);

        shader_.bind();
        shader_.setUniform("view_projection_matrix", view_projection_matrix_);
        shader_.setUniform("color", color);

        vertex_array_.bind();
        glDrawArrays(GL_LINES, 0, vertices.size());

        vertex_array_.unbind();
        shader_.unbind();
    }

    void sphere(const glm::vec4& color, const glm::vec3& position, float radius, size_t slices,
        size_t stacks)
    {
        assert(slices >= 4);
        assert(stacks >= 3);

        const auto delta_stack_angle = glm::pi<float>() / (stacks - 1);
        const auto delta_slice_angle = 2.0f * glm::pi<float>() / slices;

        std::vector<glm::vec3> points;
        for (size_t stack = 0; stack < stacks; ++stack) {
            const auto stack_angle = delta_stack_angle * stack;
            const auto xz_radius = glm::sin(stack_angle) * radius;
            const auto y = glm::cos(stack_angle) * radius;
            for (size_t slice = 0; slice < slices; ++slice) {
                const float slice_angle = delta_slice_angle * slice;
                // north-south line
                points.push_back(position
                    + glm::vec3(
                        glm::cos(slice_angle) * xz_radius, y, glm::sin(slice_angle) * xz_radius));
                points.push_back(position
                    + glm::vec3(
                        glm::cos(slice_angle) * glm::sin(stack_angle + delta_stack_angle) * radius,
                        glm::cos(stack_angle + delta_stack_angle) * radius,
                        glm::sin(slice_angle) * glm::sin(stack_angle + delta_stack_angle)
                            * radius));
                if (stack != 0 && stack != stacks - 1) {
                    // west-east line
                    points.push_back(position
                        + glm::vec3(glm::cos(slice_angle) * xz_radius, y,
                            glm::sin(slice_angle) * xz_radius));
                    points.push_back(position
                        + glm::vec3(glm::cos(slice_angle + delta_slice_angle) * xz_radius, y,
                            glm::sin(slice_angle + delta_slice_angle) * xz_radius));
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

    void set_view_projection_matrix(const glm::mat4& view_projection_matrix)
    {
        view_projection_matrix_ = view_projection_matrix;
    }

private:
    static constexpr std::string_view vert = R"(
        #version 150
        in vec3 position;
        uniform mat4 view_projection_matrix;

        void main()
        {
            gl_Position = view_projection_matrix * vec4(position, 1.0);
        }
    )";
    static constexpr std::string_view frag = R"(
        #version 150
        out vec4 frag_color;
        uniform vec4 color;

        void main()
        {
            frag_color = color;
        }
    )";

    glw::VertexArray vertex_array_;
    glw::Buffer vertex_buffer_;
    glw::ShaderProgram shader_;
    glm::mat4 view_projection_matrix_;
};
