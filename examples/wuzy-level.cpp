#include <chrono>
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
            const auto v0 = glm::make_vec3(vert_data + 3 * indices[index_offset + 0].vertex_index);
            const auto v1 = glm::make_vec3(vert_data + 3 * indices[index_offset + 1].vertex_index);
            const auto v2 = glm::make_vec3(vert_data + 3 * indices[index_offset + 2].vertex_index);
            const auto& c = colliders.emplace_back(v0, v1, v2);
            index_offset += num_face_vertices;
        }
    }
    return colliders;
}

void move_kinematic(wuzy::AabbTree& broadphase, wuzy::AabbTree::NodeQuery& bp_query,
    wuzy::Collider& collider, glwx::Transform& trafo, const glm::vec3& delta)
{
    trafo.move(delta);
    collider.set_transform(trafo.getMatrix());

    const auto [aabb_min, aabb_max] = collider.get_aabb<glm::vec3>();
    bp_query.begin(aabb_min, aabb_max, 0, nullptr);
    const auto candidates = bp_query.all();

    for (const auto& res : candidates) {
        const auto other_collider = broadphase.get_collider(res.node);
        if (other_collider == &collider) {
            continue;
        }
        if (const auto col = wuzy::get_collision<glm::vec3>(collider, *other_collider)) {
            const auto mtv = col->normal * col->depth;
            trafo.move(mtv);
            collider.set_transform(trafo.getMatrix());
        }
    }
}

glm::quat camera_look(float& pitch, float& yaw, const glm::vec2& look)
{
    pitch = std::clamp(pitch - look.y, -glm::half_pi<float>(), glm::half_pi<float>());
    yaw += -look.x;
    const auto pitch_quat = glm::angleAxis(pitch, glm::vec3(1.0f, 0.0f, 0.0f));
    const auto yaw_quat = glm::angleAxis(yaw, glm::vec3(0.0f, 1.0f, 0.0f));
    return yaw_quat * pitch_quat;
}

struct Aabb {
    glm::vec3 min;
    glm::vec3 max;
};

Aabb get_aabb(const wuzy_aabb_tree_dump_node* node)
{
    return {
        { node->aabb_min[0], node->aabb_min[1], node->aabb_min[2] },
        { node->aabb_max[0], node->aabb_max[1], node->aabb_max[2] },
    };
}

void collect_aabbs(std::vector<std::pair<Aabb, uint32_t>>& aabbs,
    const wuzy_aabb_tree_dump_node* node, uint32_t depth = 0)
{
    aabbs.push_back({ get_aabb(node), depth });
    if (!node->userdata) { // leaf
        collect_aabbs(aabbs, node->left, depth + 1);
        collect_aabbs(aabbs, node->right, depth + 1);
    }
}

std::vector<std::pair<Aabb, uint32_t>> get_aabbs(
    const std::vector<wuzy_aabb_tree_dump_node> debug_nodes)
{
    std::vector<std::pair<Aabb, uint32_t>> aabbs;
    collect_aabbs(aabbs, &debug_nodes[0]);
    return aabbs;
}

void print_node(const wuzy_aabb_tree_dump_node& node, size_t indent = 0)
{
    const auto indent_str = std::string(indent * 2, ' ');
    auto vol = [](const wuzy_aabb_tree_dump_node& node) {
        const glm::vec3 ext = {
            node.aabb_max[0] - node.aabb_min[0],
            node.aabb_max[1] - node.aabb_min[1],
            node.aabb_max[2] - node.aabb_min[2],
        };
        return ext.x * ext.y * ext.z;
    };
    fmt::println("{}{} ({}) (vol: {})", indent_str, node.id.id, fmt::ptr(node.userdata), vol(node));
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
        debug_draw.aabb(color, aabb.min, aabb.max);
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
    std::vector<wuzy_collider*> collider_ptrs(level_colliders.size(), nullptr);
    for (size_t i = 0; i < level_colliders.size(); ++i) {
        collider_ptrs[i] = &level_colliders[i].collider;
    }
    // Colliders are inserted by pointer, so their pointer have to be stable after insertion!
    /*for (auto& collider : level_colliders) {
        broadphase.insert(collider);
    }*/
    broadphase.build(collider_ptrs);
    const auto start = std::chrono::high_resolution_clock::now();
    // broadphase.rebuild();
    const auto delta = std::chrono::high_resolution_clock::now() - start;
    print_tree(broadphase);
    fmt::println(
        "rebuild time: {}us", std::chrono::duration_cast<std::chrono::microseconds>(delta).count());

    const auto stats = broadphase.get_stats();
    fmt::println("colliders={}, nodes={}, max_nodes={}", stats.num_leaves, stats.num_nodes,
        stats.max_num_nodes);

    glwx::Transform player_trafo;
    const auto player_radius = 0.35f;
    auto player_mesh = glwx::makeSphereMesh(vfmt, { 0, 1, 2 }, player_radius, 32, 32);
    wuzy::SphereCollider player_collider(player_radius);
    // We don't need to insert the player into the broadphase

    float camera_pitch = 0.0f, camera_yaw = 0.0f;
    glwx::Transform camera_trafo;
    camera_trafo.setPosition(glm::vec3(0.0f, 10.0f, 5.0f));
    camera_trafo.lookAt(glm::vec3(0.0f));

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
            // On Wayland you cannot simply SDL_SetRelativeMouseMode(SDL_TRUE).
            // Compositors may only allow mouse grab in certain conditions.
            // Grabbing the mouse in response to a click usually works.
            // (Can't wait to learn when it doesn't).
            case SDL_MOUSEBUTTONDOWN: {
                SDL_SetWindowMouseGrab(window.getSdlWindow(), SDL_TRUE);
                SDL_SetRelativeMouseMode(SDL_TRUE);
                break;
            }
            case SDL_WINDOWEVENT:
                switch (event.window.event) {
                case SDL_WINDOWEVENT_FOCUS_LOST:
                    SDL_SetWindowMouseGrab(window.getSdlWindow(), SDL_FALSE);
                    SDL_SetRelativeMouseMode(SDL_FALSE);
                }
                break;
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
        camera_trafo.setOrientation(camera_look(camera_pitch, camera_yaw, look));

        const auto kb_state = SDL_GetKeyboardState(nullptr);
        const auto move_x = kb_state[SDL_SCANCODE_D] - kb_state[SDL_SCANCODE_A];
        const auto move_z = kb_state[SDL_SCANCODE_W] - kb_state[SDL_SCANCODE_S];
        const auto move_vec = camera_trafo.getRight() * static_cast<float>(move_x)
            + camera_trafo.getForward() * static_cast<float>(move_z);
        const auto move
            = glm::length(move_vec) > 0.0f ? glm::normalize(move_vec) : glm::vec3(0.0f);

        if (input_mode == InputMode::Fps) {
            static constexpr float move_speed = 3.0f;
            move_kinematic(
                broadphase, bp_query, player_collider, player_trafo, move * dt * move_speed);
            camera_trafo.setPosition(player_trafo.getPosition());
        }
        if (input_mode == InputMode::Camera) {
            camera_trafo.move(move * dt * 3.0f);
        }

        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        const auto view_matrix = glm::inverse(camera_trafo.getMatrix());
        debug_draw.set_view_projection_matrix(projection_matrix * view_matrix);

        draw_mesh(level, glm::vec4(0.8f), texture, {}, view_matrix, projection_matrix);

        draw_mesh(
            player_mesh, glm::vec4(1.0f), texture, player_trafo, view_matrix, projection_matrix);

        if (broadphase_debug) {
            draw_broadphase_debug(broadphase, debug_draw);
        }

        const auto ray_start = camera_trafo.getPosition();
        const auto ray_dir = camera_trafo.getForward();
        wuzy_query_debug debug = {};
        const auto rc = bp_query.ray_cast(ray_start, ray_dir, 0, &debug);
        if (rc) {
            const auto [node, collider, hit] = *rc;
            const auto hit_pos = glm::make_vec3(hit.hit_position);
            debug_draw.diamond(glm::vec4(0.0f, 1.0f, 0.0f, 1.0f), hit_pos, 0.05f);
            debug_draw.arrow(glm::vec4(1.0f, 1.0f, 0.0f, 1.0f), hit_pos,
                hit_pos + glm::make_vec3(hit.normal) * 0.2f);
        }

        window.swap();
    }

    return 0;
}
