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