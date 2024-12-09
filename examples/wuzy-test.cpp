#include <span>
#include <unordered_map>

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

const auto vert = R"(
    #version 330 core
    uniform mat4 model_matrix;
    uniform mat4 view_matrix;
    uniform mat4 projection_matrix;
    uniform mat3 normal_matrix;
    layout (location = 0) in vec3 a_position;
    layout (location = 1) in vec2 a_tex_coords;
    layout (location = 2) in vec3 a_normal;
    out vec2 tex_coords;
    out vec3 normal; // view space
    void main() {
        tex_coords = a_tex_coords;
        normal = normal_matrix * a_normal;
        gl_Position = projection_matrix * view_matrix * model_matrix * vec4(a_position, 1.0);
    }
)"s;

const auto frag = R"(
    #version 330 core
    uniform sampler2D albedo;
    uniform vec3 light_dir; // view space
    uniform vec4 color;
    in vec2 tex_coords;
    in vec3 normal;
    out vec4 frag_color;
    void main() {
        vec4 base = texture2D(albedo, tex_coords);
        float n_dot_l = max(dot(light_dir, normal), 0.0);
        frag_color = color * vec4(base.rgb * n_dot_l + vec3(0.1), 1.0);
        // frag_color = vec4(normal, 1.0);
    }
)"s;

void draw_mesh(const glwx::Mesh& mesh, const glm::vec4& color, const glw::Texture& texture,
    const glwx::Transform& trafo, const glm::mat4& view_matrix, const glm::mat4& projection_matrix)
{
    static const auto prog
        = glwx::makeShaderProgram(std::string_view(vert), std::string_view(frag)).value();

    texture.bind(0);

    const auto model_matrix = trafo.getMatrix();
    const auto model_view_matrix = view_matrix * model_matrix;
    const auto normal_matrix = glm::mat3(glm::transpose(glm::inverse(model_view_matrix)));

    prog.bind();
    prog.setUniform("albedo", 0);
    prog.setUniform("model_matrix", model_matrix);
    prog.setUniform("view_matrix", view_matrix);
    prog.setUniform("projection_matrix", projection_matrix);
    prog.setUniform("normal_matrix", normal_matrix);
    prog.setUniform("light_dir", glm::vec3(0.0f, 0.0f, 1.0f));
    prog.setUniform("color", color);

    mesh.draw();
}

glm::vec3 vec3(const wuzy_vec3& v)
{
    return glm::vec3(v.x, v.y, v.z);
}

wuzy_vec3 vec3(const glm::vec3 v)
{
    return wuzy_vec3 { v.x, v.y, v.z };
}

wuzy_mat4 mat4(const glm::mat4& m)
{
    wuzy_mat4 r;
    std::memcpy(&r.cols[0], &m[0][0], sizeof(float) * 16);
    return r;
}

glwx::Mesh make_triangle_mesh(const glw::VertexFormat& vfmt, const glwx::AttributeLocations& loc,
    std::span<const glm::vec3> positions, std::span<const glm::vec2> tex_coords,
    std::span<const glm::vec3> normals, std::span<const size_t> indices)
{
    glwx::Mesh mesh;

    auto& vbuf = mesh.addVertexBuffer(vfmt, glw::Buffer::UsageHint::StaticDraw);
    vbuf.resize(positions.size());

    assert(vfmt.get(loc.position));
    auto p_acc = glwx::VertexAccessor<glm::vec3>(vbuf, loc.position);
    for (size_t i = 0; i < positions.size(); ++i) {
        p_acc[i] = positions[i];
    }

    if (loc.normal) {
        assert(vfmt.get(*loc.normal));
        assert(normals.size() == positions.size());
        auto n_acc = glwx::VertexAccessor<glm::vec3>(vbuf, *loc.normal);
        for (size_t i = 0; i < normals.size(); ++i) {
            n_acc[i] = normals[i];
        }
    }

    if (loc.texCoords) {
        assert(vfmt.get(*loc.texCoords));
        assert(tex_coords.size() == positions.size());
        auto t_acc = glwx::VertexAccessor<glm::vec2>(vbuf, *loc.texCoords);
        for (size_t i = 0; i < tex_coords.size(); ++i) {
            t_acc[i] = tex_coords[i];
        }
    }

    vbuf.update();

    auto& ibuf = mesh.addIndexBuffer(glw::IndexType::U8, glw::Buffer::UsageHint::StaticDraw);
    ibuf.resize(indices.size());
    auto i_acc = glwx::IndexAccessor(ibuf);
    for (size_t i = 0; i < indices.size(); ++i) {
        i_acc[i] = indices[i];
    }
    ibuf.update();

    mesh.primitive.indexRange = glwx::Primitive::Range { 0, indices.size() };

    return mesh;
}

std::vector<wuzy_vec3> to_wuzy(std::span<const glm::vec3> vs)
{
    std::vector<wuzy_vec3> ret;
    for (const auto& v : vs) {
        ret.emplace_back(wuzy_vec3 { v.x, v.y, v.z });
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

uint32_t collect_aabbs(
    std::vector<std::pair<wuzy_aabb, uint32_t>>& aabbs, const wuzy_aabb_tree_dump_node* node)
{
    if (node->collider) {
        aabbs.push_back({ node->aabb, 0 });
        return 0;
    } else {
        const auto d1 = collect_aabbs(aabbs, node->left);
        const auto d2 = collect_aabbs(aabbs, node->right);
        aabbs.push_back({ node->aabb, (d1 > d2 ? d1 : d2) + 1 });
        return aabbs.back().second;
    }
}

std::vector<std::pair<wuzy_aabb, uint32_t>> get_aabbs(
    const std::vector<wuzy_aabb_tree_dump_node> debug_nodes)
{
    std::vector<std::pair<wuzy_aabb, uint32_t>> aabbs;
    collect_aabbs(aabbs, &debug_nodes[0]);
    return aabbs;
}

int main()
{
    const auto window = glwx::makeWindow("Wuzy Test", 1920, 1080).value();
    glw::State::instance().setViewport(window.getSize().x, window.getSize().y);

#ifndef NDEBUG
    glwx::debug::init();
#endif

    DebugDraw debug_draw;

    const glw::VertexFormat vert_fmt {
        { 0, 3, glw::AttributeType::F32 },
        { 1, 2, glw::AttributeType::U16, true },
        { 2, 4, glw::AttributeType::IW2Z10Y10X10, true },
    };

    const auto texture = glwx::makeTexture2D(glm::vec4(1.0f));

    wuzy::AabbTree broadphase(64);
    auto bp_query = broadphase.create_node_query();

    const auto box_size = 1.0f;
    const auto half_box_size = box_size / 2.0f;
    auto box_mesh = glwx::makeBoxMesh(vert_fmt, { 0, 1, 2 }, box_size, box_size, box_size);
    std::vector<wuzy_vec3> box_verts = {
        wuzy_vec3 { -half_box_size, -half_box_size, -half_box_size },
        wuzy_vec3 { half_box_size, -half_box_size, -half_box_size },
        wuzy_vec3 { half_box_size, half_box_size, -half_box_size },
        wuzy_vec3 { -half_box_size, half_box_size, -half_box_size },

        wuzy_vec3 { -half_box_size, -half_box_size, half_box_size },
        wuzy_vec3 { half_box_size, -half_box_size, half_box_size },
        wuzy_vec3 { half_box_size, half_box_size, half_box_size },
        wuzy_vec3 { -half_box_size, half_box_size, half_box_size },
    };

    std::vector<wuzy_face_indices> box_faces = {
        wuzy_face_indices { 3, 0, 4 }, wuzy_face_indices { 3, 4, 7 }, // -x
        wuzy_face_indices { 6, 5, 1 }, wuzy_face_indices { 6, 1, 2 }, // +x
        wuzy_face_indices { 4, 0, 1 }, wuzy_face_indices { 4, 1, 5 }, // -y
        wuzy_face_indices { 3, 7, 6 }, wuzy_face_indices { 3, 6, 2 }, // +y
        wuzy_face_indices { 2, 1, 0 }, wuzy_face_indices { 2, 0, 3 }, // -z
        wuzy_face_indices { 7, 4, 5 }, wuzy_face_indices { 7, 5, 6 }, // +z
    };

    const std::vector<glm::vec3> tri_verts {
        glm::vec3(-1.0f, -1.0f, -1.0f),
        glm::vec3(-1.0f, 1.0f, 1.0f),
        glm::vec3(1.0f, 1.0f, -1.0f),
    };

    const auto tri_normal
        = glm::normalize(glm::cross(tri_verts[1] - tri_verts[0], tri_verts[2] - tri_verts[1]));
    const std::vector<glm::vec3> tri_normals { tri_normal, tri_normal, tri_normal };

    const std::vector<glm::vec2> tri_tex_coords {
        glm::vec2(0.0f, 0.0f),
        glm::vec2(1.0f, 0.0f),
        glm::vec2(0.0f, 1.0f),
    };

    const std::vector<size_t> tri_indices { 0, 1, 2 };

    const auto triangle_mesh
        = make_triangle_mesh(vert_fmt, { .position = 0, .texCoords = 1, .normal = 2 }, tri_verts,
            tri_tex_coords, tri_normals, tri_indices);

    struct Obstacle {
        enum class Type { Box, Sphere, Triangle };

        glwx::Transform trafo;
        std::unique_ptr<wuzy::Collider> collider; // need stable pointers for broadphase
        wuzy_aabb_tree_node bp_node;
        Type type;
        bool candidate = false;
        bool collision = false;
    };

    auto randf = []() { return (rand() % 10000) / 10000.0f; };
    auto lerp = [](float t, float a, float b) { return a + (b - a) * t; };
    auto randf_range = [randf, lerp](float min, float max) { return lerp(randf(), min, max); };
    srand(42);

    constexpr std::array<Obstacle::Type, 3> obstacle_types {
        Obstacle::Type::Box,
        Obstacle::Type::Sphere,
        Obstacle::Type::Triangle,
    };

    std::vector<Obstacle> obstacles;
    std::unordered_map<uint32_t, size_t> obstacle_idx_map;
    const auto range = 8;
    for (size_t i = 0; i < 10; ++i) {
        const auto x = i == 0 ? half_box_size * 2.0f + 0.1f : randf() * range * 2.0f - range;
        const auto z = i == 0 ? 0.0f : randf() * range * 2.0f - range;
        auto trafo = glwx::Transform(glm::vec3(x, 0.0f, z));
        /*trafo.setScale(
            glm::vec3(randf_range(0.5f, 1.5f), randf_range(0.5f, 1.5f), randf_range(0.5f, 1.5f)));
        trafo.setOrientation(
            glm::angleAxis(randf() * glm::two_pi<float>(), glm::vec3(1.0f, 0.0f, 0.0f))
            * glm::angleAxis(randf() * glm::two_pi<float>(), glm::vec3(0.0f, 1.0f, 0.0f))
            * glm::angleAxis(randf() * glm::two_pi<float>(), glm::vec3(0.0f, 0.0f, 1.0f)));*/
        const auto type = obstacle_types[rand() % obstacle_types.size()];
        std::unique_ptr<wuzy::Collider> collider;
        if (type == Obstacle::Type::Box) {
            collider = std::make_unique<wuzy::ConvexPolyhedronCollider>(box_verts, box_faces);
        } else if (type == Obstacle::Type::Sphere) {
            collider = std::make_unique<wuzy::SphereCollider>(half_box_size);
        } else if (type == Obstacle::Type::Triangle) {
            collider = std::make_unique<wuzy::TriangleCollider>(
                vec3(tri_verts[0]), vec3(tri_verts[1]), vec3(tri_verts[2]));
        }
        collider->set_transform(mat4((trafo.getMatrix())));
        const auto bp_node = broadphase.insert(*collider);
        obstacles.push_back(Obstacle { std::move(trafo), std::move(collider), bp_node, type });
        obstacle_idx_map.emplace(bp_node.id, i);
    }

    glwx::Transform player_trafo;
    bool player_collision = false;
    const auto player_radius = 0.5f;
    auto sphere_mesh = glwx::makeSphereMesh(vert_fmt, { 0, 1, 2 }, player_radius, 32, 32);
    wuzy::SphereCollider player_collider(player_radius);
    const auto player_node = broadphase.insert(player_collider);

    std::vector<wuzy_aabb_tree_dump_node> bp_debug_nodes(128);

    float camera_pitch = 0.0f, camera_yaw = 0.0f;
    glwx::Transform camera_trafo;
    camera_trafo.setPosition(glm::vec3(0.0f, 10.0f, 5.0f));
    camera_trafo.lookAt(glm::vec3(0.0f));
    SDL_SetRelativeMouseMode(SDL_TRUE);

    const auto aspect = static_cast<float>(window.getSize().x) / window.getSize().y;
    glm::mat4 projection_matrix = glm::perspective(glm::radians(45.0f), aspect, 0.1f, 100.0f);

    glEnable(GL_DEPTH_TEST);

    enum class InputMode { Player, Camera };
    InputMode input_mode = InputMode::Player;

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
                    if (input_mode == InputMode::Player) {
                        input_mode = InputMode::Camera;
                        fmt::print("Input Mode: Camera\n");
                    } else if (input_mode == InputMode::Camera) {
                        input_mode = InputMode::Player;
                        fmt::print("Input Mode: Player\n");
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

        if (input_mode == InputMode::Player) {
            player_collision = false;
            for (auto& obstacle : obstacles) {
                obstacle.candidate = false;
                obstacle.collision = false;
            }

            player_trafo.move(vel);
            player_collider.set_transform(mat4(player_trafo.getMatrix()));

            // This is kind of racey, because the broadphase query depends on the transform of the
            // player collider, which is changed in the loop.
            // In a real game, you would do this totally differently (many possible ways)!
            broadphase.update(player_node);
            bp_query.begin(player_collider.get_aabb());
            for (auto node : bp_query.all()) {
                if (node.id == player_node.id) {
                    continue;
                }
                const auto obstacle_idx = obstacle_idx_map.at(node.id);
                auto& obstacle = obstacles[obstacle_idx];
                assert(obstacle.bp_node.id == node.id);
                obstacle.candidate = true;
                if (const auto col = wuzy::get_collision(player_collider, *obstacle.collider)) {
                    obstacle.collision = true;
                    const auto mtv = -glm::vec3(col->normal.x, col->normal.y, col->normal.z)
                        * col->depth * 1.0f;
                    player_trafo.move(mtv);
                    player_collider.set_transform(mat4(player_trafo.getMatrix()));
                    player_collision = true;
                }
            }
        } else if (input_mode == InputMode::Camera) {
            camera_trafo.moveLocal(vel);
            const auto look = glm::vec2(mouse_x, mouse_y) * 0.002f;
            camera_pitch
                = std::clamp(camera_pitch - look.y, -glm::half_pi<float>(), glm::half_pi<float>());
            camera_yaw += -look.x;
            const auto pitch_quat = glm::angleAxis(camera_pitch, glm::vec3(1.0f, 0.0f, 0.0f));
            const auto yaw_quat = glm::angleAxis(camera_yaw, glm::vec3(0.0f, 1.0f, 0.0f));
            camera_trafo.setOrientation(yaw_quat * pitch_quat);
        }

        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        const auto view_matrix = glm::inverse(camera_trafo.getMatrix());
        debug_draw.set_view_projection_matrix(projection_matrix * view_matrix);

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
                draw_mesh(box_mesh, color, texture, obstacle.trafo, view_matrix, projection_matrix);
            } else if (obstacle.type == Obstacle::Type::Sphere) {
                draw_mesh(
                    sphere_mesh, color, texture, obstacle.trafo, view_matrix, projection_matrix);
            } else if (obstacle.type == Obstacle::Type::Triangle) {
                draw_mesh(
                    triangle_mesh, color, texture, obstacle.trafo, view_matrix, projection_matrix);
            }
        }

        const auto color = player_collision ? glm::vec4(1.0f, 0.0f, 0.0f, 1.0f) : glm::vec4(1.0f);
        draw_mesh(sphere_mesh, color, texture, player_trafo, view_matrix, projection_matrix);

        std::vector<glm::vec4> aabb_colors {
            glm::vec4(1.0f, 0.0f, 0.0f, 1.0f),
            glm::vec4(0.0f, 1.0f, 0.0f, 1.0f),
            glm::vec4(0.0f, 0.0f, 1.0f, 1.0f),
            glm::vec4(1.0f, 1.0f, 0.0f, 1.0f),
            glm::vec4(1.0f, 0.0f, 1.0f, 1.0f),
            glm::vec4(0.0f, 1.0f, 1.0f, 1.0f),
            glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
        };

        const auto debug_nodes = broadphase.dump_nodes();
        const auto aabbs = get_aabbs(debug_nodes);
        for (const auto& [aabb, depth] : aabbs) {
            const auto& color = aabb_colors[depth % aabb_colors.size()];
            const auto height_offset = glm::vec3(0.0f, depth * 0.3f, 0.0f);
            debug_draw.aabb(color, vec3(aabb.min), vec3(aabb.max) + height_offset);
        }

        const auto cam_pos = camera_trafo.getPosition();
        const auto cam_fwd = camera_trafo.getForward();
        const auto rc = bp_query.ray_cast(vec3(cam_pos), vec3(cam_fwd));
        if (rc) {
            const auto [node, res] = *rc;
            const auto marker_pos = vec3(res.hit_position);
            debug_draw.diamond(glm::vec4(0.0f, 1.0f, 0.0f, 1.0f), marker_pos, 0.05f);
            debug_draw.arrow(glm::vec4(1.0f, 1.0f, 0.0f, 1.0f), marker_pos,
                marker_pos + vec3(res.normal) * 0.2f);
        }

        window.swap();
    }

    return 0;
}
