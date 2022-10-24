# wuzy
Collision Detection Library

Almost always when I make games I come up with some ad-hoc stuff, which often works, but is always a huge pain and takes too much time in game jams.

This library is meant to be used for games like that. That means it's not going to be fast and it's not going to do a lot of stuff.

It will just do collision detection and resolution, no kinematics or dynamics. You can just create colliders and bump them into each other and see what happens.

Most of the games I made were not made with any of the big engines, so I only used a proper physics engine a few times and I hated it quite a bit. It's very restrictive that most of the time you cannot set the position directly (which makes sense, but it is restrictive none the less) and I firmly believe that for most games the optimal physics have nothing to do with real-world kinematics and dynamics, so this library handles collision detection and resolution. No accelerations or forces, no contraints or anything of the sort. Maybe velocites if I ever introduce continuous collision detection.

This is not made for AAA games and millions of colliders. This is for games that hobby gamedevs might make in a gamejam or the average indie game you can get on Steam. It tries to be obvious, easy to integrate and use and not as fast a possible.

Something like this:
https://docs.godotengine.org/en/stable/classes/class_kinematicbody.html

# TODO:
* Fix EPA for deformed spheres (reaches max iterations sometimes and returns garbage)
* Acceleration Structure (AABB Tree?)
  http://allenchou.net/2014/02/game-physics-broadphase-dynamic-aabb-tree/
  http://allenchou.net/2014/02/game-physics-updating-aabbs-for-polyhedrons/
  https://box2d.org/files/ErinCatto_DynamicBVH_GDC2019.pdf
  https://box2d.org/files/ErinCatto_DynamicBVH_Full.pdf
* Collision Resolution
  http://media.steampowered.com/apps/valve/2015/DirkGregorius_Contacts.pdf
* Ray Casting
* Convex Decomposition
  https://medium.com/@val.v.gorbunov/a-search-for-better-convex-decomposition-671ea647cec
* Support Functions
  - Boxes
  - Capsules

## Maybe Later
* Contact Points
  https://box2d.org/files/ErinCatto_GJK_GDC2010.pdf
* Calculate closest distance with GJK result (http://allenchou.net/2013/12/game-physics-collision-detection-gjk/ - "Extra" section)
* Hill Climbing for Extreme Vertices (Real-Time Collision Detection)
  http://allenchou.net/2014/02/game-physics-implementing-support-function-for-polyhedrons-using-half-edges/
* Vertex Caching
* Minkowski Portal Refinement

## Resources
* http://allenchou.net/2013/12/game-physics-collision-detection-gjk/
* http://allenchou.net/2013/12/game-physics-contact-generation-epa/
* https://github.com/kevinmoran/GJK
* https://www.youtube.com/watch?v=Qupqu1xe7Io
* https://blog.winter.dev/2020/gjk-algorithm/
* https://www.youtube.com/watch?v=0XQ2FSz3EK8
