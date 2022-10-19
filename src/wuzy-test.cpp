#include <iostream>

#include "wuzy/wuzy.hpp"

using namespace wuzy;

int main()
{
    Collider a;
    a.addShape(std::make_unique<Sphere>(1.0f), Mat4 {});

    Collider b;
    b.addShape(std::make_unique<Sphere>(1.0f), Mat4 {});

    std::cout << testCollision(a, b) << std::endl;
}
