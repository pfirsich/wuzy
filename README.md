# wuzy
Collision Detection Library

Almost always when I make games I come up with some ad-hoc stuff for collision detection, which often works, but is always a huge pain and takes too much time in game jams.

I feel like either you bump a few spheres and AABBs into each other with your own code OR you have to go super big and use a full blown physics engine. This is supposed to fit into the gigantic gap inbetween.

This is supposed to be the least amount of code that lets you control a character in a level.

It will just do collision detection and resolution, no kinematics or dynamics. You can just create colliders and bump them into each other and get vectors that resolve the collision. It also does ray casting.

Most of the games I made were not made with any of the big engines, so I only used a proper physics engine a few times and I hated it quite a bit. It's very restrictive that most of the time you cannot set the position directly (which makes sense, but it is restrictive none the less) and I firmly believe that for most games the optimal physics have nothing to do with real-world kinematics and dynamics, so this library handles collision detection and resolution. No accelerations or forces, no constraints or anything of the sort. Maybe velocites if I ever introduce continuous collision detection.

Something like this: https://docs.godotengine.org/en/stable/classes/class_kinematicbody.html

This is not made for AAA games and millions of colliders. This is for games that hobby gamedevs might make in a gamejam or the average indie game you can get on Steam. It tries to be obvious and easy to integrate. It's supposed to provide the minimum amount of collision detection and resolution you need to make a game (that is not super focused on physics).

## Integration
wuzy provides a C header file (`include/wuzy/wuzy.h`), but is written in C++20. All dynamic allocations are obvious and can be controlled via a custom allocator. Only a few operations dynamically allocate at all (such a creating a new AABB Tree or preparing memory for queries) and it's possible to do all of that at initialization time.

If you don't want to use CMake, you can build `src/wuzy.cpp` yourself with `std=c++20`. It should compile with `-Wall -Wextra -pedantic -Werror -Wconversion` and `-fno-exceptions`/`-fno-rtti`, if you need them.

There is a C++ wrapper in `include/wuzy/wuzy.hpp` and a few examples in `examples/`.

## Architecture
wuzy uses [GJK](https://en.wikipedia.org/wiki/Gilbert%E2%80%93Johnson%E2%80%93Keerthi_distance_algorithm) to test whether shapes overlap and EPA (Expanding Polytope Algorithm) to generate contact information (collision normal and depth). To speed up queries there is also a AABB Tree.

I plan to add a high-level API which should should reduce the API surface to the essential and allow usage without understanding any of the involved algorithms at all.

## Usage Notes
If you need collisions with meshes, like levels, just create a separate collider for each triangle of the mesh. For static level geometry I would advised to create a separate AABB Tree.

# TODO:
* AABB Tree Rebalancing, bulk-insert of nodes that balances tree (for levels): https://box2d.org/files/ErinCatto_DynamicBVH_Full.pdf
* Tests!
* Capsule Shapes - The support function is easy, but ray casting is not.
* Some support for continuous collision detection - I really just need support for sweeping shapes to the support functions (somehow). The swept shapes are convex again (for linear movement at least), so there has to be a way. If you really need this and this hasn't been implemented yet, just do some raycasts and then multisample.
  - https://box2d.org/files/ErinCatto_ContinuousCollision_GDC2013.pdf
  - https://www.youtube.com/watch?v=7_nKOET6zwI

## Maybe Later
* Calculate closest distance with GJK result: (http://allenchou.net/2013/12/game-physics-collision-detection-gjk/ - "Extra" section)
* Enlarged AABBs - store enlarged AABBs in the AABB Tree and only update a leaf if they actual ("tight") AABB moves outside of the enlarged AABB, so moving objects don't require a tree update every frame.
* Hill Climbing for Extreme Vertices (Real-Time Collision Detection): http://allenchou.net/2014/02/game-physics-implementing-support-function-for-polyhedrons-using-half-edges/ - It makes a lot of sense algorithmically, but I think it's very cache-unfriendly and you need very large polyhedra for this to make sense (haven't tried it though).
* Heightmap Terrains
* [Minkowski Portal Refinement](https://en.wikipedia.org/wiki/Minkowski_Portal_Refinement) / [XenoCollide](http://xenocollide.snethen.com/)
* Better Collision Resolution: http://media.steampowered.com/apps/valve/2015/DirkGregorius_Contacts.pdf
* Contact Points: https://box2d.org/files/ErinCatto_GJK_GDC2010.pdf, GDC10_Coumans_Erwin_Contact.pdf - This is actually really hard, because you often need multiple contacts and GJK/EPA can only give you one, so you need to cache them across frames. Also contact points are essential for physics engines (you need to apply forces), but you don't really need them if you just bump a few shapes into each other and calculating them isn't free. I will think about this again, if I really need it, but that hasn't happened yet.

## Resources
* [Real-Time Collision Detection by Christer Ericson](http://realtimecollisiondetection.net/)
* http://allenchou.net/2013/12/game-physics-collision-detection-gjk/
* https://github.com/kevinmoran/GJK
* https://www.youtube.com/watch?v=Qupqu1xe7Io
* https://blog.winter.dev/2020/gjk-algorithm/
* http://allenchou.net/2013/12/game-physics-contact-generation-epa/
* https://blog.winter.dev/2020/epa-algorithm/
* http://allenchou.net/2014/02/game-physics-broadphase-dynamic-aabb-tree/
* https://box2d.org/files/ErinCatto_DynamicBVH_Full.pdf
