#include <chrono>

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
#include <span>

#include "debugdraw.hpp"
#include "wuzy/wuzy-hl.h"

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

void create_colliders(const tinyobj::ObjReader& reader)
{
    std::vector<uint32_t> indices;
    const auto vert_data = reader.GetAttrib().vertices.data();
    for (const auto& shape : reader.GetShapes()) {
        size_t index_offset = 0;
        for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); ++f) {
            const auto num_face_vertices = shape.mesh.num_face_vertices[f];
            assert(num_face_vertices == 3);
            indices.push_back(shape.mesh.indices[index_offset + 0].vertex_index);
            indices.push_back(shape.mesh.indices[index_offset + 1].vertex_index);
            indices.push_back(shape.mesh.indices[index_offset + 2].vertex_index);
            index_offset += num_face_vertices;
        }
    }
    const auto mesh = wuzy_hl_mesh_create(
        vert_data, reader.GetAttrib().vertices.size(), indices.data(), indices.size() / 3);
    wuzy_hl_collider_create_mesh(mesh);
}

glm::vec3 compute_mtv(std::span<wuzy_collision_result> cols)
{
    // first take weighted average
    glm::vec3 mtv = { 0.0f, 0.0f, 0.0f };
    for (const auto& col : cols) {
        mtv += glm::make_vec3(col.normal) * col.depth;
    }
    // find a length that clears the largest collision
    const auto n = glm::make_vec3(cols[0].normal);
    const auto d = cols[0].depth;
    // project mtv onto deepest collision normal
    mtv *= d / glm::dot(mtv, glm::normalize(n));
    return mtv;
}

bool move_player(wuzy_hl_collider_id collider, glwx::Transform& trafo, glm::vec3& velocity,
    const glm::vec3& move, bool jump, float dt)
{
    constexpr float accel = 20.0f;
    constexpr float max_speed = 5.0f;
    constexpr float deccel_time = 0.25f;
    constexpr float gravity = 20.0f;
    constexpr float max_fall_speed = 15.0f;
    constexpr float jump_height = 1.0f;

    const auto ray_start = trafo.getPosition();
    const glm::vec3 ray_dir = { 0.0f, -1.0f, 0.0f };
    wuzy_hl_ray_cast_result rc;
    const auto hit = wuzy_hl_ray_cast(&ray_start.x, &ray_dir.x, 0, &rc, 1);
    // kind of controls walkable slope height
    const auto on_ground = hit && rc.hit.t < 0.71f;
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
    wuzy_hl_collider_set_transform(collider, &trafo.getMatrix()[0].x);

    // This is kind of racey, because the broadphase query depends on the transform of the
    // player collider, which is changed in the loop.
    // In a real game, you would do this totally differently (many possible ways)!

    static std::array<wuzy_hl_collider_id, 64> candidates_buf;

    const auto n
        = wuzy_hl_query_candidates(collider, 0, candidates_buf.data(), candidates_buf.size());
    const auto candidates = std::span(candidates_buf).first(n);

    bool collision = false;
    for (const auto other : candidates) {
        for (size_t i = 0; i < 8; ++i) {
            static std::array<wuzy_collision_result, 4> cols;
            const auto n = wuzy_hl_get_collisions(collider, other, cols.data(), cols.size());
            if (n > 0) {
                trafo.move(compute_mtv(std::span(cols).first(n)));
                wuzy_hl_collider_set_transform(collider, &trafo.getMatrix()[0].x);
                collision = true;
            }
        }
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

    wuzy_hl_init({
        .max_num_colliders = 1024,
    });

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
    create_colliders(reader);
    const auto start = std::chrono::high_resolution_clock::now();
    // broadphase.rebuild();
    const auto delta = std::chrono::high_resolution_clock::now() - start;

    glwx::Transform player_trafo;
    glm::vec3 player_velocity = glm::vec3(0.0f);
    const auto player_radius = 0.35f;
    const auto player_height = 0.25f;
    bool last_jump = false;
    auto player_mesh
        = glwx::makeCapsuleMesh(vfmt, { 0, 1, 2 }, player_radius, player_height / 2.0f, 32, 16, 1);
    const float half_up[3] = { 0.0f, player_height / 2.0f, 0.0f };
    const auto player_collider = wuzy_hl_collider_create_capsule(half_up, player_radius);
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
            move_player(player_collider, player_trafo, player_velocity, move, jump, dt);
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
            player_mesh, glm::vec4(1.0f), texture, player_trafo, view_matrix, projection_matrix);

        const auto ray_start = camera_trafo.getPosition();
        const auto ray_dir = camera_trafo.getForward();
        wuzy_query_debug debug = {};
        wuzy_hl_ray_cast_result rc;
        const auto hits = wuzy_hl_ray_cast(&ray_start.x, &ray_dir.x, 0, &rc, 1);
        if (hits) {
            const auto hit_pos = glm::make_vec3(rc.hit.hit_position);
            debug_draw.diamond(glm::vec4(0.0f, 1.0f, 0.0f, 1.0f), hit_pos, 0.05f);
            debug_draw.arrow(glm::vec4(1.0f, 1.0f, 0.0f, 1.0f), hit_pos,
                hit_pos + glm::make_vec3(rc.hit.normal) * 0.2f);
        }

        window.swap();
    }

    return 0;
}
