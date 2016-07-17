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

#ifndef HEADER_VRREVOLUTEJOINT
#define HEADER_VRREVOLUTEJOINT

#include "vrJoint.h"

///Data structure for revolute joint
typedef struct vrRevoluteConstraint
{
	///Holds inverse mass matrix
	vrMat2 K;
	///Relative point on A
	vrVec2 ra;
	///Relative point on B
	vrVec2 rb;
	vrVec2 accum;
} vrRevoluteConstraint;

///Allocates memory for revolute joint data
vrRevoluteConstraint * vrRevoluteAlloc();
///Initializes revolute joint data
vrRevoluteConstraint * vrRevoluteInit(vrRevoluteConstraint * constraint);
///Initializes a revolute joint with a joint
vrJoint * vrRevoluteJointInit(vrJoint * joint, vrRigidBody * A, vrRigidBody * B, vrVec2 pointA, vrVec2 pointB);
///Frees memory
void vrRevoluteJointDestroy(vrJoint * joint);
///Velocity pre solve function for a revolute joint
void vrRevoluteJointPreSolve(vrJoint * joint, vrFloat dt);
///Velocity solve function for a revolute joint
void vrRevoluteJointSolveVelocity(vrJoint * joint);
///Position pre solve function for a revolute joint
void vrRevoluteJointPostSolve(vrJoint * joint, vrFloat dt);
///Position solve function for a revolute joint
void vrRevoluteJointSolvePosition(vrJoint * joint);
///Drawing function
void vrRevoluteJointDraw(vrJoint* joint);
#endif