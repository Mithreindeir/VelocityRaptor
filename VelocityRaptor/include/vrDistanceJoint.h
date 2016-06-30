/*
* Copyright (c) 2016 Cormac Grindall (Mithreindeir)
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

#ifndef HEADER_VRSPRINGJOINT
#define HEADER_VRSPRINGJOINT
#include "vrJoint.h"
#include "vrMath.h"

///Data structure for joint
typedef struct vrDistanceJoint
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

} vrDistanceJoint;

///Allocates memory for spring joint data
vrDistanceJoint* vrDistanceAlloc();
///Initializes spring joint data
vrDistanceJoint* vrDistanceInit(vrDistanceJoint* joint);
///Initializes a spring joint with a joint
vrJoint* vrDistanceJointInit(vrJoint* joint, vrRigidBody* A, vrRigidBody* B, vrVec2 pointA, vrVec2 pointB);
///Frees memory
void vrDistanceJointDestroy(vrJoint* joint);
///Velocity pre solve function for a spring joint
void vrDistanceJointPreSolve(vrJoint* joint, vrFloat dt);
///Velocity solve function for a spring joint
void vrDistanceJointSolve(vrJoint* joint);
#endif