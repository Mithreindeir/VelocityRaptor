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
#ifndef HEADER_VRJOINT
#define HEADER_VRJOINT

#include "vrMath.h"
#include "vrRigidBody.h"

typedef struct vrJoint vrJoint;

typedef void(*vrJointPreSolveFunc)(vrJoint* joint, vrFloat dt);
typedef void(*vrJointSolveVelocityFunc)(vrJoint* joint);
typedef void(*vrJointPostSolveFunc)(vrJoint* joint);
typedef void(*vrJointSolvePositionFunc)(vrJoint* joint);
typedef void(*vrJointDrawFunc)(vrJoint* joint);

typedef void* vrJointData;

///Point relative to body's center, changes as body rotates
typedef struct vrLocalPoint
{
	///Initial body orientation
	vrFloat initialOrientation;
	///Point relative to body initially
	vrVec2 initialPoint;
	///Body which the point is on
	vrRigidBody* body;
} vrLocalPoint;

///Empty Joint constraint
struct vrJoint
{
	///One body in the joint
	vrRigidBody* A;
	///One body in the joint
	vrRigidBody* B;

	///Local point on body A
	vrLocalPoint anchorA;
	///Local point on body B
	vrLocalPoint anchorB;
	
	///Function to pre solve constraint
	vrJointPreSolveFunc preSolve;
	///Function to solve velocity constraint
	vrJointSolveVelocityFunc solveVelocity;
	///Function to pre solve position constraint, after velocity 
	vrJointPostSolveFunc postSolve;
	///Function to solve position constraint
	vrJointSolvePositionFunc solvePosition;
	///Void pointer to data for joint
	vrJointData jointData;
	///function to draw constraint
	vrJointDrawFunc drawJoint;
};

///Allocates memory for a joint
vrJoint* vrJointAlloc();
///Initializes joint
vrJoint* vrJointInit(vrJoint* joint);
///Destroys a joint
void vrJointDestroy(vrJoint* joint);
///Returns a local point
vrLocalPoint vrLocalPointInit(vrRigidBody* body, vrVec2 point);
///Updates the local point and returns it
vrVec2 vrGetLocalPoint(vrLocalPoint* localPoint);

#endif