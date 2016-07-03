# VelocityRaptor
[![Build Status](https://upload.wikimedia.org/wikipedia/commons/f/f8/License_icon-mit-88x31-2.svg)]()

2D physics engine, written in highly portable C, with no dependencies other than the standard library and glfw3, glew and OpenGL for rendering. Designed to be fast, stable and accurate. Velocity Raptor is free to use in commercial and non-commercial applications.

It features Collision detection for convex polygons with the Separate Axis Theorum, and it can support concanve shapes, by using ear clipping to turn concave shapes into triangles. Collision's are resolved with impulses, and for bodies with two contacts, a mini-block solver will solve them together. The only joint currently available is the distance joint. This engine is still under development and not ready for use. 
