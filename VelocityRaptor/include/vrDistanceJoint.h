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

#ifndef HEADER_VRSPRINGJOINT
#define HEADER_VRSPRINGJOINT
#include "vrJoint.h"
#include "vrMath.h"

///Data structure for distance joint
typedef struct vrDistanceConstraint
{
	///Holds inverse mass
	vrFloat invMass;
	///Distances stiffness
	vrFloat stiffness;
	///Accumulates impulse
	vrFloat accumImpulse;
	///Bias for position correction
	vrFloat bias;
	///Relative position on body A
	vrVec2 ra;
	///Relative position on body B
	vrVec2 rb;
	///Normal direction 
	vrVec2 dir;
	///Resting length
	vrFloat restLength;
	///Damping
	vrFloat damping;
} vrDistanceConstraint;

///Allocates memory for spring joint data
vrDistanceConstraint* vrDistanceAlloc();
///Initializes spring joint data
vrDistanceConstraint* vrDistanceInit(vrDistanceConstraint* joint);
///Initializes a spring joint with a joint
vrJoint* vrDistanceConstraintInit(vrJoint* joint, vrRigidBody* A, vrRigidBody* B, vrVec2 pointA, vrVec2 pointB);
///Frees memory
void vrDistanceConstraintDestroy(vrJoint* joint);
///Velocity pre solve function for a spring joint
void vrDistanceConstraintPreSolve(vrJoint* joint, vrFloat dt);
///Velocity solve function for a spring joint
void vrDistanceConstraintSolve(vrJoint joint);
///Velocity post solve function for a spring joint
void vrDistanceConstraintPostSolve(vrJoint* joint, vrFloat dt);
///Position solve function for a spring joint
void vrDistanceConstraintSolvePosition(vrJoint* joint);
///Drawing function
void vrDistanceConstraintDraw(vrJoint* joint);

#endif