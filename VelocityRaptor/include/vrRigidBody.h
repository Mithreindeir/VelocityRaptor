/*
* Copyright (c) 2016 Cormac Grindall (Mithreindeir)
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* vrFreely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef HEADER_VRRIGIDBODY
#define HEADER_VRRIGIDBODY

#include "vrShape.h"
#include "vrArray.h"
#include "vrColor.h"

///Holds data about the properties
/// Of the body
typedef struct vrMaterial
{
	///Bounciness 
	vrFloat restitution;
	///Friction
	vrFloat friction;

	///Mass of a object
	vrFloat mass;
	///Resistance to rotate
	vrFloat momentInertia;

	///1/mass
	vrFloat invMass;
	///1/momentInertia
	vrFloat invMomentInertia;

	///Linear velocity damping
	vrFloat linearDamping;
	///Angular velocity damping
	vrFloat angularDamping;
} vrMaterial;

///Holds data for collision masks
///And other collision relevant details
typedef struct vrCollisionGroup
{
	int maskBit;
	int categoryMask;
} vrCollisionGroup;

///The rigidbody structure
typedef struct vrRigidBody
{
	//Linear components
	///Position in world space
	vrVec2 position;
	///Linear velocity of the body
	vrVec2 velocity;
	///Force being applied to the body
	vrVec2 force;
	///Psuedo velocity for position solver
	vrVec2 vel_bias;

	//Angular components
	///Current orientation of body
	vrFloat orientation;
	///The angular velocity of the body
	vrFloat angularVelocity;
	///Torque being applied the body
	vrFloat torque;
	///Psuedo angular velocity for position solver
	vrFloat angv_bias;

	///Collision data
	vrCollisionGroup collisionData;
	///Properties of body
	vrMaterial bodyMaterial;
	///The physical representation of the body
	vrArray* shape;
	///Color
	vrColor color;
	///Center
	vrVec2 center;
	///OBB that contains all the shapes
	vrOrientedBoundingBox obb;
} vrRigidBody;

///Allocates a body
vrRigidBody* vrBodyAlloc();
///Initializes a body
vrRigidBody* vrBodyInit(vrRigidBody* body);
///Destroys a body
void vrBodyDestroy(vrRigidBody* body);

///Initializes and returns default values
vrMaterial vrMaterialInit();

///Integrates the forces being applied to a body
void vrBodyIntegrateForces(vrRigidBody* body, vrFloat dt);
///Integrates the velocity 
void vrBodyIntegrateVelocity(vrRigidBody* body, vrFloat dt);
///Updates the body's obb
void vrBodyUpdateOBB(vrRigidBody* body);
///Returns the moment of inertia for a box
vrFloat vrMomentForBox(vrFloat w, vrFloat h, vrFloat mass);
///Returns the moment of inertia for a polygon
vrFloat vrMomentForPoly(vrPolygonShape* shape, vrFloat mass);
///Returns the moment of inertia for a circle
vrFloat vrMomentForCircle(vrFloat radius, vrFloat mass);
///Returns the area of a polygon
vrFloat vrAreaForPoly(vrPolygonShape* shape);

#endif

