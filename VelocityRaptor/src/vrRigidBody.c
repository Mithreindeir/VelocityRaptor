#include "../include/vrRigidBody.h"
#include "../include/velocityraptor.h"
#include "../include/vrMath.h"


vrRigidBody * vrBodyInit(vrRigidBody* body)
{
	body->angularVelocity = 0;
	body->velocity = vrVect(0, 0);
	body->angv_bias = 0;
	body->vel_bias = vrVect(0, 0);
	body->position = vrVect(0, 0);
	body->orientation = 0.0;
	body->force = vrVect(0, 0);
	body->torque = 0.0;

	body->bodyMaterial = vrMaterialInit();
	return body;
}

vrRigidBody * vrBodyAlloc()
{
	return vrAlloc(sizeof(vrRigidBody));
}

vrMaterial vrMaterialInit()
{
	vrMaterial material;
	material.friction = 0.3;
	material.invMass = 0.1;
	material.invMomentInertia = 0.001;
	material.mass = 10.0;
	material.momentInertia = 10.0;
	material.restitution = 0.0;
	return material;
}

void vrBodyDestroy(vrRigidBody * body)
{
	vrShapeDestroy(body->shape);
	vrFree(body);
}

void vrBodyIntegrateForces(vrRigidBody * body, vrFloat dt)
{
	body->velocity = vrAdd(body->velocity, vrScale(body->force,  dt*body->bodyMaterial.invMass));
	body->angularVelocity += body->torque * dt;

	body->force = vrVect(0, 0);
	body->torque = 0;
}

void vrBodyIntegrateVelocity(vrRigidBody * body, vrFloat dt)
{
	body->position = vrAdd(body->position, vrScale(vrAdd(body->velocity, body->vel_bias), dt));
	body->shape->move(body->shape->shape, vrScale(vrAdd(body->velocity, body->vel_bias), dt));

	body->orientation += ( body->angv_bias + body->angularVelocity )* dt;
	body->shape->rotate(body->shape->shape, (body->angv_bias + body->angularVelocity)* dt);

	body->angv_bias = 0;
	body->vel_bias = vrVect(0, 0);
}

