# wuzy
Collision Detection Library

Almost always when I make games I come up with some ad-hoc stuff, which often works, but is always a huge pain and takes too much time in game jams.

This library is meant to be used for games like that. That means it's not going to be fast and it's not going to do a lot of stuff.

It will just do collision detection and resolution, no kinematics or dynamics. You can just create colliders and bump them into each other and see what happens.

Most of the games I made were not made with any of the big engines, so I only used a proper physics engine a few times and I hated it quite a bit. It's very restrictive that most of the time you cannot set the position directly (which makes sense, but it is restrictive none the less) and I firmly believe that for most games the optimal physics have nothing to do with real-world kinematics and dynamics, so this library handles collision detection and resolution. No accelerations or forces, no contraints or anything of the sort. Maybe velocites if I ever introduce continuous collision detection.
