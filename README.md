# VelocityRaptor
[![Build Status](https://upload.wikimedia.org/wikipedia/commons/f/f8/License_icon-mit-88x31-2.svg)]()
[![Build Status](https://travis-ci.org/Mithreindeir/VelocityRaptor.svg?branch=master)](https://travis-ci.org/Mithreindeir/VelocityRaptor)


2D physics engine, written in highly portable C, with no dependencies other than the standard library. Designed to be fast, stable and accurate. Velocity Raptor is free to use in commercial and non-commercial applications.

It features Collision detection for convex polygons with the Separate Axis Theorum, and it can support concanve shapes, by using ear clipping to turn concave shapes into triangles. Collision's are resolved with impulses, and for bodies with two contacts, a mini-block solver will solve them together. The engine is completely ready for use (but still in development), however it has yet to be documented so you may have to dig through the source code. Units are in pixels.
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
