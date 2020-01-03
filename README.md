# VelocityRaptor
[![Build Status](https://upload.wikimedia.org/wikipedia/commons/f/f8/License_icon-mit-88x31-2.svg)]()
[![Build Status](https://travis-ci.org/Mithreindeir/VelocityRaptor.svg?branch=master)](https://travis-ci.org/Mithreindeir/VelocityRaptor)

A programming exercise to make a physics engine. 
The design and math is due mostly Erin Catto, and his "Fast and Simple Physics
using Sequential Impulses" and "Physics for Game Programmers Talk".
For Collision Detection, credit to Dyn4J for best Separate Axis Theorem explanation I've seen,
http://www.dyn4j.org/2010/01/sat/

It features Collision detection for convex polygons with the Separate Axis Theorum, and it can support concanve shapes, by using ear clipping to turn concave shapes into triangles. Collision's are resolved with impulses. Units are in pixels.

##Projects Using Velocity Raptor
[Crown Room](https://www.youtube.com/watch?v=mb8CKdHvIpo)

[Chicken Coup](https://www.youtube.com/watch?v=Fd2WpBGHzi4)

[Topdown Shooter](https://www.youtube.com/watch?v=gGHBFwacEIc)

##Velocity Raptor Demo videos
[Velocity Raptor Benchmark and Joints](https://www.youtube.com/watch?v=uhLYXmzzZp8&index=1&list=PLbl-UC7p03cuFCSpCfEVoQxMQfcG3zm9K)

[Velocity Raptor Fluids](https://www.youtube.com/watch?v=pnyTvRR69EI&index=2&list=PLbl-UC7p03cuFCSpCfEVoQxMQfcG3zm9K)

[Velocity Raptor concave and composite polyons](https://youtu.be/nplQNW8RBYk)

##Usage

To create a world:

    vrWorld* world = vrWorldInit(vrWorldAlloc());
    world->gravity = vrVect(0, 9.81);
    
To add a body:

    vrRigidBody* body = vrBodyInit(vrBodyAlloc());
    body->bodyMaterial.mass = 5;
    body->bodyMaterial.invMass = 1.0 / body->bodyMaterial.mass;
    body->bodyMaterial.momentInertia = vrMomentForBox(10, 10, body->bodyMaterial.mass);
    body->bodyMaterial.invMomentInertia = 1.0 / body->bodyMaterial.momentInertia;
    body->bodyMaterial.friction = 0.01;
    body->bodyMaterial.restitution = 0.2;
    vrWorldAddBody(world, body);
    
To add a box shape:

    vrShape* shape = vrShapeInit(vrShapeAlloc());
    shape = vrShapePolyInit(shape);
    shape->shape = vrPolyBoxInit(shape->shape, x, y, w, h);
    vrArrayPush(body->shape, shape);
    
To add a polygon shape:

    vrShape* shape = vrShapeInit(vrShapeAlloc());
    shape = vrShapePolyInit(shape);
    vrAddVertexToPolyShape(shape->shape, vrVect(0, 0));
    vrAddVertexToPolyShape(shape->shape, vrVect(10, 0));
    vrAddVertexToPolyShape(shape->shape, vrVect(10, 10));
    vrAddVertexToPolyShape(shape->shape, vrVect(0, 10));
    vrArrayPush(body->shape, shape);
    
To add a circle shape:

    vrShape* shape = vrShapeInit(vrShapeAlloc());
    shape = vrShapeCircleInit(shape);
    shape->shape = vrCircleInit(shape->shape);
    vrCircleShape* circle = shape->shape;
    circle->center = vrVect(0, 0);
    circle->radius = 25;
    vrArrayPush(body->shape, shape);

Every tick, call this to update the world:

    vrWorldStep(world);
    
To clean up, and free all memory:

    vrWorldDestroy(world);
    
To remove a body from the world and free its memory  before destruction of world

    vrWorldRemoveBody(world, body);
