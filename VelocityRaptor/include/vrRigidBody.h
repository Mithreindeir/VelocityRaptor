/*
* Copyright (c) 2006-2009 Cormac Grindall (Mithreindeir)
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
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

//A rigidbodies material values
typedef struct vrMaterial
{
	vrFloat restitution;
	vrFloat friction;

	vrFloat mass;
	vrFloat momentInertia;

	vrFloat invMass;
	vrFloat invMomentInertia;
} vrMaterial;

//Holds data for collision marks
typedef struct vrCollisionGroup
{
	int maskBit;
	int categoryMask;
} vrCollisionGroup;

//The rigidbody structure
typedef struct vrRigidBody
{
	//Linear components
	vrVec2 position;
	vrVec2 velocity;
	vrVec2 force;
	vrVec2 vel_bias;

	//Angular components
	vrFloat orientation;
	vrFloat angularVelocity;
	vrFloat torque;
	vrFloat angv_bias;

	vrCollisionGroup collisionData;
	vrMaterial bodyMaterial;
	vrShape* shape;
} vrRigidBody;

/* Body control functions */
//Allocates a body
vrRigidBody* vrBodyAlloc();
//Initializes a body
vrRigidBody* vrBodyInit(vrRigidBody* body);
//Destroys a body
void vrBodyDestroy(vrRigidBody* body);
/* Body control functions */

//Initializes and returns a blank material
vrMaterial vrMaterialInit();

//Integration funtions
void vrBodyIntegrateForces(vrRigidBody* body, vrFloat dt);
void vrBodyIntegrateVelocity(vrRigidBody* body, vrFloat dt);

#endif
