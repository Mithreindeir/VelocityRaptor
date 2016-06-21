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
	material.mass = 100.0;
	material.invMass = 1.0 / material.mass;
	material.momentInertia = 10000.0;
	material.invMomentInertia = 1.0 / material.momentInertia;

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
	//Integrate velocity
	body->position = vrAdd(body->position, vrScale(vrAdd(body->velocity, body->vel_bias), dt));
	body->shape->move(body->shape->shape, vrScale(vrAdd(body->velocity, body->vel_bias), dt));

	body->orientation += ( body->angv_bias + body->angularVelocity )* dt;
	body->shape->rotate(body->shape->shape, (body->angv_bias + body->angularVelocity)* dt);

	//Update Oriented Bounding box
	body->shape->obb = body->shape->updateOBB(body->shape->shape);

	body->angv_bias = 0;
	body->vel_bias = vrVect(0, 0);
}
