#ifndef HEADER_VRMANIFOLD
#define HEADER_VRMANIFOLD
#include "vrRigidBody.h"
#include "velocityraptor.h"

#define MAX_IMPULSE 10000
#define VR_VELOCITYTHRESHOLD 2

typedef struct vrContactPoint
{
	//World coordinates of the contact point
	vrVec2 point;
	//Penetration of point
	vrFloat depth;

	//The accumulated normal impulse 
	//For sequential solver
	vrFloat normalImpulseSum;
	//The accumulated tangent impulse 
	//For sequential solver
	vrFloat tangentImpulseSum;
	//The accumulated bias impulse 
	//For sequential solver
	vrFloat biasImpulseSum;
	//Distance from point to body a's center
	vrVec2 ra;
	//Distance from point to body b's center
	vrVec2 rb;
	//The inverse effection normal mass
	vrFloat effectiveMassN;
	//The inverse effection tangent mass
	vrFloat effectiveMassT;
	//Bias of contact
	vrFloat bias;
	//Velocity bias for restitution
	vrFloat velocityBias;
} vrContact;

typedef struct vrBlockSolverData
{
	//Effective mass for contacts
	vrVec2 effMass;//Effective mass in vector form
	vrMat2 A; //Effective mass in matrix form
	//Contact Velocity (relative velocity dot normal)
	vrVec2 contactVel;
	//Initial guess
	vrVec2 initialGuess;
	/* Clamping */
	vrVec2 lo, hi;
} vrBlockSolverData;


typedef struct vrManifold
{
	//Bodies involved in contact
	vrRigidBody* A;
	vrRigidBody* B;

	//Contact information
	vrContact* contacts;
	int contact_points;

	//Collision information
	vrVec2 normal;
	vrVec2 tangent;
	vrFloat restitution;
	vrFloat friction;
	vrFloat penetration;
	
	//For block contacts
	vrBlockSolverData solverData;

	//Maintenance variables
	vrBOOL flip;
	vrBOOL active;
} vrManifold;

vrManifold* vrManifoldAlloc();
vrManifold* vrManifoldInit(vrManifold* manifold);
void vrManifoldPreStep(vrManifold* manifold, vrFloat dt);
void vrManifoldPostStep(vrManifold* manifold, vrFloat dt);
vrVec2 vrManifoldRelativeVelocity(vrRigidBody* a, vrRigidBody* b, vrVec2 ra, vrVec2 rb);
void vrManifoldSolveVelocity(vrManifold* manifold);
void vrManifoldSolvePosition(vrManifold* manifold, vrFloat dt);
vrVec2 vrManifoldGuassSeidel(vrBlockSolverData solverData);
vrVec2 vrManifoldConjugateGradient(vrBlockSolverData solverData, vrFloat tolerance);
void vrManifoldApplyImpulse(vrRigidBody* A, vrRigidBody* b, vrVec2 ra, vrVec2 rb, vrVec2 impulse);
void vrManifoldApplyBiasImpulse(vrRigidBody* A, vrRigidBody* B, vrVec2 ra, vrVec2 rb, vrVec2 impulse);
void vrManifoldAddContactPoints(vrManifold* old_manifold, const vrManifold new_manifold);
void vrManifoldSetBodies(vrManifold* manifold, vrRigidBody* b1, vrRigidBody* b2);
vrFloat vrManifoldGetContactVel(vrManifold* manifold, int index);

#endif