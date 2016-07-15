/*
* Copyright (c) 2006-2009 Cormac Grindall (Mithreindeir)
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

/*
*@file vrWorld.h
*@author Mithreindeir
*@date 25 June 2016
*@brief World definition
*
*File containing definition of world
*and functions controlling the world
*/

#ifndef HEADER_VRWORLD
#define HEADER_VRWORLD

#include <time.h>
#include "vrRigidBody.h"
#include "vrBroadphase.h"
#include "vrArray.h"
#include "vrManifold.h"
#include "vrHashMap.h"
#include "vrJoint.h"

///Container of bodies that interact with each other
typedef struct vrWorld
{
	///Array holding the bodies
	vrArray* bodies;
	///Number of bodies in this world
	int num_bodies;

	///The last time vrWorldStep exited
	vrFloat lastTime;
	///Accumulator of time 
	vrFloat accumulator;
	///The timestep for the integrator
	vrFloat timeStep;

	///Number of global iterations
	///Of solving manifold for velocity
	int velIterations;
	///Number of global iterations
	///Of solving manifold for position
	int posIterations;
	///The gravity vector
	vrVec2 gravity;
	///Hash table holding the Contact manifolds
	vrHashTable* manifoldMap;
	///Array holding all the joints
	vrArray* joints;
	///Array holding manifolds to recycle
	vrArray* manifoldPool;
	vrManifold* manifolds;
	int num_manifolds;
	///Broadphase struct
	vrBroadphase* broadphase;
} vrWorld;

///Allocates memory for a world
vrWorld* vrWorldAlloc();
///Initializes the world
vrWorld* vrWorldInit(vrWorld* world);
///Frees memory used by the world
void vrWorldDestroy(vrWorld* world);
///Steps the world, resolving collisions
///And updating the bodies
void vrWorldStep(vrWorld* world);
///Adds a body to the world
void vrWorldAddBody(vrWorld* world, vrRigidBody * body);
///Function called from inside vrWorldStep
///Which detects collisions
void vrWorldQueryCollisions(vrWorld* world);
///Solves the contact manifolds, using the number
///Of velocity iterations
void vrWorldSolveVelocity(vrWorld* world, vrFloat dt);
///Solves the contact manifolds, using the number
///Of position iterations
void vrWorldSolvePosition(vrWorld* world, vrFloat dt);

#endif
