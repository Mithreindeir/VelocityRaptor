# VelocityRaptor

<a href=""><img src="https://upload.wikimedia.org/wikipedia/commons/thumb/f/f8/License_icon-mit-88x31-2.svg/2000px-License_icon-mit-88x31-2.svg.png"></a>

[![Build Status](https://travis-ci.org/Mithreindeir/VelocityRaptor.svg?branch=master)](https://travis-ci.org/Mithreindeir/VelocityRaptor)
(Travis isn't configured yet, thats why it says its failed).


2D physics engine, written in highly portable C, with no dependencies other than the standard library and glfw3, glew and OpenGL for rendering. It features Collision detection for convex polygons with the Separate Axis Theorum, and it can support concanve shapes, by using ear clipping to turn concave shapes into triangles. Collision's are resolved with impulses, and for bodies with two contacts, a mini-block solver will solve them together. The only joint currently available is the distance joint. This engine is still under development and not ready for use. 
