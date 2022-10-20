#include <iostream>

#include "wuzy/wuzy.hpp"

using namespace wuzy;

int main()
{
    Collider a;
    // a.addShape(std::make_unique<Sphere>(1.0f), Mat4 {});
    a.addShape<Sphere>(Mat4 {}, 1.0f);

    Collider b;
    // b.addShape(std::make_unique<Sphere>(1.0f), Mat4 {});
    b.addShape<Sphere>(Mat4 {}, 1.0f);

    std::cout << testCollision(a, b) << std::endl;

    a.setTransform(Mat4::translate(Vec3 { 2.01f, 0.0f, 0.0f }));

    std::cout << testCollision(a, b) << std::endl;
}
