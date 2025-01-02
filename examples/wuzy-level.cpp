#include <chrono>
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

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

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
    uniform mat4 view_matrix;
    in vec2 tex_coords;
    in vec3 normal;
    out vec4 frag_color;
    void main() {
        vec4 base = texture2D(albedo, tex_coords);
        float n_dot_l_cam = max(dot(light_dir, normal), 0.0);
        float n_dot_l_world = max(dot(view_matrix*normalize(vec4(0.5, 1.0, 0.5, 0.0)), vec4(normal, 0.0)), 0.0);
        frag_color = color * vec4(base.rgb * (n_dot_l_cam * 0.2 + n_dot_l_world * 0.8 + vec3(0.15)), 1.0);
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

wuzy_vec3 vec3(const glm::vec3& v)
{
    return wuzy_vec3 { v.x, v.y, v.z };
}

wuzy_vec3 vec3(const float* data)
{
    return wuzy_vec3 { data[0], data[1], data[2] };
}

glwx::Mesh get_mesh(const tinyobj::ObjReader& reader, const glw::VertexFormat& vfmt)
{
    glwx::Mesh mesh;
    auto& vbuf = mesh.addVertexBuffer(vfmt, glw::Buffer::UsageHint::StaticDraw);

    const auto vertex_count = [&reader]() {
        size_t n = 0;
        for (const auto& shape : reader.GetShapes()) {
            for (const auto& num_face_vertices : shape.mesh.num_face_vertices) {
                n += num_face_vertices;
            }
        }
        return n;
    }();
    vbuf.resize(vertex_count);

    const auto& attrib = reader.GetAttrib();
    auto p_acc = glwx::VertexAccessor<glm::vec3>(vbuf, 0);
    auto t_acc = glwx::VertexAccessor<glm::vec2>(vbuf, 1);
    auto n_acc = glwx::VertexAccessor<glm::vec3>(vbuf, 2);
    size_t vert_idx = 0;
    for (const auto& shape : reader.GetShapes()) {
        const auto num_faces = shape.mesh.num_face_vertices.size();
        size_t index_offset = 0;
        for (size_t f = 0; f < num_faces; ++f) {
            const auto num_face_vertices = shape.mesh.num_face_vertices[f];
            assert(num_face_vertices == 3);
            for (size_t v = 0; v < num_face_vertices; ++v) {
                assert(vert_idx < vertex_count);
                const auto& idx = shape.mesh.indices[index_offset + v];
                p_acc[vert_idx] = glm::make_vec3(attrib.vertices.data() + 3 * idx.vertex_index);
                n_acc[vert_idx] = idx.normal_index >= 0
                    ? glm::make_vec3(attrib.normals.data() + 3 * idx.normal_index)
                    : glm::vec3(0.0f);
                t_acc[vert_idx] = idx.texcoord_index >= 0
                    ? glm::make_vec2(attrib.texcoords.data() + 2 * idx.texcoord_index)
                    : glm::vec2(0.0f);
                vert_idx++;
            }
            index_offset += num_face_vertices;
        }
    }
    vbuf.update();
    mesh.primitive.vertexRange = { 0, vertex_count };
    return mesh;
}

std::vector<wuzy::TriangleCollider> get_colliders(const tinyobj::ObjReader& reader)
{
    std::vector<wuzy::TriangleCollider> colliders;
    const auto vert_data = reader.GetAttrib().vertices.data();
    for (const auto& shape : reader.GetShapes()) {
        const auto& indices = shape.mesh.indices;
        const auto num_faces = shape.mesh.num_face_vertices.size();
        size_t index_offset = 0;
        for (size_t f = 0; f < num_faces; ++f) {
            const auto num_face_vertices = shape.mesh.num_face_vertices[f];
            assert(num_face_vertices == 3);
            const auto v0 = vec3(vert_data + 3 * indices[index_offset + 0].vertex_index);
            const auto v1 = vec3(vert_data + 3 * indices[index_offset + 1].vertex_index);
            const auto v2 = vec3(vert_data + 3 * indices[index_offset + 2].vertex_index);
            const auto& c = colliders.emplace_back(v0, v1, v2);
            index_offset += num_face_vertices;
        }
    }
    return colliders;
}

wuzy_mat4 mat4(const glm::mat4& m)
{
    wuzy_mat4 r;
    std::memcpy(&r.cols[0], &m[0][0], sizeof(float) * 16);
    return r;
}

bool move_player(wuzy::AabbTree& broadphase, wuzy::AabbTree::NodeQuery& bp_query,
    wuzy::Collider& collider, glwx::Transform& trafo, glm::vec3& velocity, const glm::vec3& move,
    bool jump, float dt)
{
    constexpr float accel = 20.0f;
    constexpr float max_speed = 5.0f;
    constexpr float deccel_time = 0.25f;
    constexpr float gravity = 20.0f;
    constexpr float max_fall_speed = 15.0f;
    constexpr float jump_height = 1.0f;

    const auto ray_start = vec3(trafo.getPosition());
    const auto ray_dir = wuzy_vec3 { 0.0f, -1.0f, 0.0f };
    const auto rc = bp_query.ray_cast(ray_start, ray_dir);
    const auto on_ground = rc && rc->result.t < 0.71f; // kind of controls walkable slope height
    if (on_ground) {
        if (velocity.y < 0.0f) {
            velocity.y = 0.0f;
        }
        if (jump) {
            // v(t) = v0 - g*t => s(t) = v0*t - 0.5*g*t*t
            // v(T) = 0 <=> v0 = g*T
            // s(T) = v0*T - 0.5*g*T*T = v0*v0/g - 0.5*v0*v0/g = 0.5*v0*v0/g
            // <=> sqrt(2*s(T)*g) = v0
            velocity.y = glm::sqrt(2 * jump_height * gravity);
        }
    } else {
        velocity.y -= gravity * dt;
        if (velocity.y < -max_fall_speed) {
            velocity.y = -max_fall_speed;
        }
    }

    const auto local_move = trafo.getOrientation() * move;
    const auto xz_move = glm::vec3(local_move.x, 0.0f, local_move.z);
    const auto xz_move_len = glm::length(xz_move);
    if (glm::length2(xz_move) > 0.0f) {
        velocity += xz_move / xz_move_len * accel * dt;
        const auto vel_xz = glm::vec3(velocity.x, 0.0f, velocity.z);
        const auto move_speed = glm::length(vel_xz);
        if (move_speed > max_speed) {
            velocity *= max_speed / move_speed;
        }
    } else {
        const auto vel_xz = glm::vec3(velocity.x, 0.0f, velocity.z);
        const auto move_speed = glm::length(vel_xz);
        const auto deccel = max_speed / deccel_time * dt;
        if (move_speed > deccel) {
            velocity -= vel_xz / move_speed * deccel;
        } else {
            velocity = glm::vec3(0.0f, velocity.y, 0.0f);
        }
    }

    trafo.move(velocity * dt);
    collider.set_transform(mat4(trafo.getMatrix()));

    // This is kind of racey, because the broadphase query depends on the transform of the
    // player collider, which is changed in the loop.
    // In a real game, you would do this totally differently (many possible ways)!
    wuzy_query_debug debug = {};
    bp_query.begin(collider.get_aabb(), 0, &debug);
    const auto candidates = bp_query.all();
    fmt::println("collision: nodes={}, bitmasks={}, aabbs={}, leaves={}, full={}",
        debug.nodes_checked, debug.bitmask_checks_passed, debug.aabb_checks_passed,
        debug.leaves_checked, debug.full_checks_passed);

    bool collision = false;
    for (auto node : candidates) {
        const auto other_collider = broadphase.get_collider(node);
        if (other_collider == &collider) {
            continue;
        }
        wuzy_gjk_debug gjk_debug = {};
        wuzy_epa_debug epa_debug = {};
        if (const auto col
            = wuzy::get_collision(collider, *other_collider, &gjk_debug, &epa_debug)) {
            const auto mtv
                = -glm::vec3(col->normal.x, col->normal.y, col->normal.z) * col->depth * 1.0f;
            trafo.move(mtv);
            collider.set_transform(mat4(trafo.getMatrix()));
            collision = true;
        }
        wuzy_gjk_debug_free(&gjk_debug);
        wuzy_epa_debug_free(&epa_debug);
    }
    return collision;
}

glm::quat camera_look(float& pitch, float& yaw, const glm::vec2& look)
{
    pitch = std::clamp(pitch - look.y, -glm::half_pi<float>(), glm::half_pi<float>());
    yaw += -look.x;
    const auto pitch_quat = glm::angleAxis(pitch, glm::vec3(1.0f, 0.0f, 0.0f));
    const auto yaw_quat = glm::angleAxis(yaw, glm::vec3(0.0f, 1.0f, 0.0f));
    return yaw_quat * pitch_quat;
}

void collect_aabbs(std::vector<std::pair<wuzy_aabb, uint32_t>>& aabbs,
    const wuzy_aabb_tree_dump_node* node, uint32_t depth = 0)
{
    aabbs.push_back({ node->aabb, depth });
    if (!node->collider) {
        collect_aabbs(aabbs, node->left, depth + 1);
        collect_aabbs(aabbs, node->right, depth + 1);
    }
}

std::vector<std::pair<wuzy_aabb, uint32_t>> get_aabbs(
    const std::vector<wuzy_aabb_tree_dump_node> debug_nodes)
{
    std::vector<std::pair<wuzy_aabb, uint32_t>> aabbs;
    collect_aabbs(aabbs, &debug_nodes[0]);
    return aabbs;
}

void print_node(const wuzy_aabb_tree_dump_node& node, size_t indent = 0)
{
    const auto indent_str = std::string(indent * 2, ' ');
    auto vol = [](const wuzy_aabb& b) {
        const auto ext = vec3(b.max) - vec3(b.min);
        return ext.x * ext.y * ext.z;
    };
    fmt::println(
        "{}{} ({}) (vol: {})", indent_str, node.id.id, fmt::ptr(node.collider), vol(node.aabb));
    if (node.left) {
        print_node(*node.left, indent + 1);
    }
    if (node.right) {
        print_node(*node.right, indent + 1);
    }
}

void print_tree(const wuzy::AabbTree& tree)
{
    const auto debug_nodes = tree.dump_nodes();
    assert(!debug_nodes[0].parent); // root
    print_node(debug_nodes[0]);
}

void draw_broadphase_debug(wuzy::AabbTree& broadphase, DebugDraw& debug_draw)
{
    static const std::vector<glm::vec4> aabb_colors {
        glm::vec4(1.0f, 0.0f, 0.0f, 1.0f),
        glm::vec4(0.0f, 1.0f, 0.0f, 1.0f),
        glm::vec4(0.0f, 0.0f, 1.0f, 1.0f),
        glm::vec4(1.0f, 1.0f, 0.0f, 1.0f),
        glm::vec4(1.0f, 0.0f, 1.0f, 1.0f),
        glm::vec4(0.0f, 1.0f, 1.0f, 1.0f),
        glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
    };

    const auto debug_nodes = broadphase.dump_nodes();
    for (const auto& [aabb, depth] : get_aabbs(debug_nodes)) {
        const auto& color = aabb_colors[depth % aabb_colors.size()];
        debug_draw.aabb(color, vec3(aabb.min), vec3(aabb.max));
    }
}

int main()
{
    const auto window = glwx::makeWindow("Wuzy Test", 1920, 1080).value();
    glw::State::instance().setViewport(window.getSize().x, window.getSize().y);

#ifndef NDEBUG
    glwx::debug::init();
#endif

    DebugDraw debug_draw;

    const glw::VertexFormat vfmt {
        { 0, 3, glw::AttributeType::F32 },
        { 1, 2, glw::AttributeType::U16, true },
        { 2, 4, glw::AttributeType::IW2Z10Y10X10, true },
    };

    const auto texture = glwx::makeTexture2D(glm::vec4(1.0f));

    wuzy::AabbTree broadphase(2048);
    auto bp_query = broadphase.create_node_query();

    tinyobj::ObjReader reader;
    if (!reader.ParseFromFile("assets/meshes/quakey.obj", {})) {
        if (!reader.Error().empty()) {
            fmt::print(stderr, "TinyObjReader: {}\n", reader.Error());
        }
        return 1;
    }

    if (!reader.Warning().empty()) {
        fmt::print("TinyObjReader: {}\n", reader.Warning());
    }

    const auto level = get_mesh(reader, vfmt);
    auto level_colliders = get_colliders(reader);
    for (auto& collider : level_colliders) {
        broadphase.insert(collider);
    }
    print_tree(broadphase);

    const auto stats = broadphase.get_stats();
    fmt::println("colliders={}, nodes={}, max_nodes={}", stats.num_colliders, stats.num_nodes,
        stats.max_num_nodes);

    glwx::Transform player_trafo;
    glm::vec3 player_velocity = glm::vec3(0.0f);
    const auto player_radius = 0.5f;
    bool last_jump = false;
    auto sphere_mesh = glwx::makeSphereMesh(vfmt, { 0, 1, 2 }, player_radius, 32, 32);
    wuzy::SphereCollider player_collider(player_radius);
    // We don't need to insert the player into the broadphase

    float camera_pitch = 0.0f, camera_yaw = 0.0f;
    glwx::Transform camera_trafo;
    camera_trafo.setPosition(glm::vec3(0.0f, 10.0f, 5.0f));
    camera_trafo.lookAt(glm::vec3(0.0f));
    SDL_SetRelativeMouseMode(SDL_TRUE);

    const auto aspect = static_cast<float>(window.getSize().x) / window.getSize().y;
    glm::mat4 projection_matrix = glm::perspective(glm::radians(45.0f), aspect, 0.1f, 100.0f);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    enum class InputMode { Camera, Fps };
    InputMode input_mode = InputMode::Fps;

    bool broadphase_debug = false;

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
                case SDLK_b:
                    broadphase_debug = !broadphase_debug;
                    break;
                case SDLK_m:
                    if (input_mode == InputMode::Camera) {
                        input_mode = InputMode::Fps;
                        fmt::print("Input Mode: Fps\n");
                    } else if (input_mode == InputMode::Fps) {
                        input_mode = InputMode::Camera;
                        fmt::print("Input Mode: Player\n");
                    }
                    break;
                }
            }
        }

        const auto now = glwx::getTime();
        const auto dt = now - time;
        time = now;

        int mouse_rel_x = 0, mouse_rel_y = 0;
        SDL_GetRelativeMouseState(&mouse_rel_x, &mouse_rel_y);
        const auto sensitivity = 0.002f;
        const auto look = glm::vec2(mouse_rel_x, mouse_rel_y) * sensitivity;

        const auto kb_state = SDL_GetKeyboardState(nullptr);
        const auto move_x = kb_state[SDL_SCANCODE_D] - kb_state[SDL_SCANCODE_A];
        const auto move_y = kb_state[SDL_SCANCODE_R] - kb_state[SDL_SCANCODE_F];
        const auto move_z = kb_state[SDL_SCANCODE_S] - kb_state[SDL_SCANCODE_W];
        const auto jump = kb_state[SDL_SCANCODE_SPACE] && !last_jump;
        last_jump = kb_state[SDL_SCANCODE_SPACE];
        const auto move_vec = glm::vec3(move_x, move_y, move_z);
        const auto move = glm::length(move_vec) > 0.0f ? glm::normalize(move_vec) : glm::vec3(0.0f);

        if (input_mode == InputMode::Fps) {
            player_trafo.setOrientation(camera_look(camera_pitch, camera_yaw, look));
            move_player(broadphase, bp_query, player_collider, player_trafo, player_velocity, move,
                jump, dt);
            camera_trafo = player_trafo;
        }
        if (input_mode == InputMode::Camera) {
            camera_trafo.setOrientation(camera_look(camera_pitch, camera_yaw, look));
            camera_trafo.moveLocal(move * dt * 3.0f);
        }

        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        const auto view_matrix = glm::inverse(camera_trafo.getMatrix());
        debug_draw.set_view_projection_matrix(projection_matrix * view_matrix);

        draw_mesh(level, glm::vec4(0.8f), texture, {}, view_matrix, projection_matrix);

        draw_mesh(
            sphere_mesh, glm::vec4(1.0f), texture, player_trafo, view_matrix, projection_matrix);

        if (broadphase_debug) {
            draw_broadphase_debug(broadphase, debug_draw);
        }

        const auto ray_start = vec3(camera_trafo.getPosition());
        const auto ray_dir = vec3(camera_trafo.getForward());
        wuzy_query_debug debug = {};
        const auto rc = bp_query.ray_cast(ray_start, ray_dir, 0, &debug);
        if (rc) {
            const auto [node, res] = *rc;
            const auto marker_pos = vec3(res.hit_position);
            debug_draw.diamond(glm::vec4(0.0f, 1.0f, 0.0f, 1.0f), marker_pos, 0.05f);
            debug_draw.arrow(glm::vec4(1.0f, 1.0f, 0.0f, 1.0f), marker_pos,
                marker_pos + vec3(res.normal) * 0.2f);
        }

        fmt::println("raycast: nodes={}, bitmasks={}, aabbs={}, leaves={}, full={}",
            debug.nodes_checked, debug.bitmask_checks_passed, debug.aabb_checks_passed,
            debug.leaves_checked, debug.full_checks_passed);

        window.swap();
    }

    return 0;
}
