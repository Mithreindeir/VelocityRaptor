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

#ifndef HEADER_VRMANIFOLD
#define HEADER_VRMANIFOLD

#include "vrRigidBody.h"
#include "velocityraptor.h"
#include "vrJoint.h"

#define MAX_IMPULSE 10000
#define VR_VELOCITYTHRESHOLD 2

///Contact point from collision detection,
///Holds point and solver data for the point
typedef struct vrContactPoint
{
	///To redo penetration
	vrVec2 best_Ref;
	///World coordinates of the contact point
	vrVec2 point;
	///Penetration of point
	vrFloat depth;

	///The accumulated normal impulse
	///For sequential solver
	vrFloat normalImpulseSum;
	///The accumulated tangent impulse
	///For sequential solver
	vrFloat tangentImpulseSum;
	///The accumulated bias impulse
	///For sequential solver
	vrFloat biasImpulseSum;
	///Distance from point to body a's center
	vrVec2 ra;
	///Distance from point to body b's center
	vrVec2 rb;
	///The inverse effection normal mass
	vrFloat effectiveMassN;
	///The inverse effection tangent mass
	vrFloat effectiveMassT;
	///Bias of contact
	vrFloat bias;
	///Velocity bias for restitution
	vrFloat velocityBias;
	///Relative position
	vrLocalPoint contactAnchor;
} vrContact;

///The solver data put in portable form
///For mini block solvers
typedef struct vrBlockSolverData
{
	///Effective mass for contacts
	vrVec2 effMass;
	///Effective mass in matrix form
	vrMat2 A; 
	///Contact Velocity (relative velocity dot normal)
	vrVec2 contactVel;
	///Initial guess
	vrVec2 initialGuess;
	///Clamping values
	vrVec2 lo, hi;
} vrBlockSolverData;

///Contact manifold, holds data about collisions,
///And impulses being applied to correct them
typedef struct vrManifold
{
	///Body involved in contact, which
	///Holds the reference shape
	vrRigidBody* A;
	///Body involved in contact, which
	///Holds the incident shape
	vrRigidBody* B;
	///Contacts on the shape
	///Only a max of 2 per shape
	vrContact contacts[2];
	///Number of contacts
	int contact_points;
	///Normal of edges colliding
	///Points from A to B
	vrVec2 normal;
	///Tangent direction
	vrVec2 tangent;
	///Restitution, or bounciness
	///Of collision
	vrFloat restitution;
	///Friction in the collision
	vrFloat friction;
	///Amount the bodies are penetrating
	vrFloat penetration;
	///For block contacts
	vrBlockSolverData solverData;
	///Direction of normal
	vrBOOL flip;
	///If the manifold has been resolved
	vrBOOL active;
	///If the manifold is new
	vrBOOL firstTime;
} vrManifold;

///Allocates memory for a manifold
vrManifold* vrManifoldAlloc();
///Initializes a manifold
vrManifold* vrManifoldInit(vrManifold* manifold);
///Destroys a manifold
void vrManifoldDestroy(vrManifold* manifold);

///Pre calculates all variables that don't need to be
///Re calculated each iteration for velocity
void vrManifoldPreStep(vrManifold* manifold, vrFloat dt);
///Pre calculates all variables that don't need to be
///Re calculated each iteration for position
void vrManifoldPostStep(vrManifold* manifold, vrFloat dt);
///Calculates relative velocity between points on two bodies
extern inline vrVec2 vrManifoldRelativeVelocity(vrRigidBody* a, vrRigidBody* b, vrVec2 ra, vrVec2 rb);
///Solves the local velocity
void vrManifoldSolveVelocity(vrManifold* manifold);
///Solves the local position
void vrManifoldSolvePosition(vrManifold* manifold, vrFloat dt);
///A mini block LPC solver if there is two contacts
vrVec2 vrManifoldGuassSeidel(vrBlockSolverData solverData);
///Clamps the normal impulse so the accumulated impulse is never negative
vrFloat vrManifoldClampNormalImpulse(vrManifold* manifold, vrFloat im, int index);
///A mini block LPC solver if there is two contacts
vrVec2 vrManifoldConjugateGradient(vrBlockSolverData solverData, vrFloat tolerance);
///Applies impulses from a point
void vrManifoldApplyImpulse(vrRigidBody* A, vrRigidBody* b, vrVec2 ra, vrVec2 rb, vrVec2 impulse);
///Applies bias impulses from a point (bias impulses don't create energy)
extern inline void vrManifoldApplyBiasImpulse(vrRigidBody* A, vrRigidBody* B, vrVec2 ra, vrVec2 rb, vrVec2 impulse);
///Updates contact points
void vrManifoldAddContactPoints(vrManifold* old_manifold, const vrManifold new_manifold);
///Sets bodies
void vrManifoldSetBodies(vrManifold* manifold, vrRigidBody* b1, vrRigidBody* b2);
///The relative velocity along the normal
extern inline vrFloat vrManifoldGetContactVel(vrManifold* manifold, int index);

#endif
