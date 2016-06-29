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

#include "../include/vrRigidBody.h"


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
	body->color = vrColorCreate(fmod(rand(), 0.8) + 0.2, fmod(rand(), 0.8) + 0.2, fmod(rand(), 0.8) + 0.2);
	body->shape = vrArrayInit(vrArrayAlloc(), sizeof(vrShape*));
	return body;
}

vrRigidBody * vrBodyAlloc()
{
	return vrAlloc(sizeof(vrRigidBody));
}

vrMaterial vrMaterialInit()
{
	vrMaterial material;
	material.friction = 0.1;
	material.mass = 1.0;
	material.invMass = 1.0 / material.mass;
	material.momentInertia = 30.0;
	material.invMomentInertia = 1.0 / material.momentInertia;
	material.angularDamping = 0;
	material.linearDamping = 0;

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
	//Apply damping
	body->velocity = vrScale(body->velocity, VR_POW(1.0 - body->bodyMaterial.linearDamping, dt));
	body->angularVelocity *= VR_POW(1.0 - body->bodyMaterial.angularDamping, dt);

	//Apply force
	body->velocity = vrAdd(body->velocity, vrScale(body->force,  dt*body->bodyMaterial.invMass));
	body->angularVelocity += body->torque * dt;

	body->force = vrVect(0, 0);
	body->torque = 0;
}

void vrBodyIntegrateVelocity(vrRigidBody * body, vrFloat dt)
{

	//Integrate velocity
	body->position = vrAdd(body->position, vrScale(vrAdd(body->velocity, body->vel_bias), dt));

	for (int i = 0; i < body->shape->sizeof_active; i++)
	{
		vrShape* shape = body->shape->data[i];
		shape->move(shape->shape, vrScale(vrAdd(body->velocity, body->vel_bias), dt));
	}

	body->orientation += ( body->angv_bias + body->angularVelocity )* dt;
	//Get center
	vrVec2 center = vrVect(0, 0);
	for (int i = 0; i < body->shape->sizeof_active; i++)
	{
		vrShape* shape = body->shape->data[i];
		center = vrAdd(center, shape->getCenter(shape->shape));
	}
	center = vrScale(center, 1.0 / body->shape->sizeof_active);
	body->center = center;
	//Rotate
	for (int i = 0; i < body->shape->sizeof_active; i++)
	{
		vrShape* shape = body->shape->data[i];
		shape->rotate(shape->shape, (body->angv_bias + body->angularVelocity)* dt, center);
	}
	//Update Oriented Bounding box
	for (int i = 0; i < body->shape->sizeof_active; i++)
	{
		vrShape* shape = body->shape->data[i];
		shape->obb = shape->updateOBB(shape->shape);
	}

	body->angv_bias = 0;
	body->vel_bias = vrVect(0, 0);
}

//Agorithms from https://en.wikipedia.org/wiki/List_of_moments_of_inertia
vrFloat vrMomentForBox(vrFloat w, vrFloat h, vrFloat mass)
{
	//I = m/12(h^2 + w^2)
	return (mass/12.0)*(w*w + h*h);
}

vrFloat vrMomentForPoly(vrPolygonShape * shape, vrFloat mass)
{
	//If 3 vertices, use triangle formula
	//I = m/6(P*P + P*Q + Q*Q)
	if (shape->num_vertices == 3)
	{
		vrVec2 P, Q;
		//First point is at origin
		vrNode* n = shape->vertices->head;
		vrVec2 o = ((vrVertex*)n->data)->vertex;
		n = n->next;
		P = vrSub(((vrVertex*)n->data)->vertex, o);
		n = n->next;
		Q = vrSub(((vrVertex*)n->data)->vertex, o);
		return (mass / 6.0)*(vrDot(P, P) + vrDot(P, Q) + vrDot(Q, Q));
	}
	//Otherwise use polygon moment formula
	/*
			/ En ||Pn+1 x Pn || ((Pn dot Pn) + (Pn dot Pn+1) + (Pn+1 dot Pn+1) \
	I = m/6 | -----------------------------------------------------------------|
			\						En||Pn+1 x Pn||							   /
	*/
	vrNode* n = shape->vertices->head;
	vrFloat sumT = 0;
	vrFloat sumB = 0;
	vrVec2 center = vrPolyGetCenter(shape);

	while (n)
	{
		vrVec2 P = ((vrVertex*)n->data)->vertex;
		P = vrSub(P,center);
		vrVec2 N = vrVect(0, 0);
		if (n->next && n->next->data)
		{
			N = ((vrVertex*)n->next->data)->vertex;
			N = vrSub(N, center);
		}

		vrFloat cross = (vrCross(N, P));
		sumT += cross * (vrDot(P, P) + vrDot(P, N) + vrDot(N, N));
		sumB += cross;
		n = n->next;
		//printf("P (%f, %f) and N (%f and %f) and C (%f, %f)\n", P.x, P.y, N.x, N.y, center.x, center.y);

	}

	return (mass/6.0)*(sumT/sumB);
}

vrFloat vrMomentForCircle(vrFloat radius, vrFloat mass)
{
	return (mass*radius*radius)/2.0;
}
