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

#include "../include/vrParticleSystem.h"
#include "../include/velocityraptor.h"
#define EPSILON 1.192092896e-07f

vrParticleSystem * vrParticleSystemAlloc()
{
	return vrAlloc(sizeof(vrParticleSystem));
}

vrParticleSystem * vrParticleSystemInit(vrParticleSystem * psys)
{
	psys->resting_d = 1;
	psys->k_stiff = 3.0;
	psys->k_stiffN = 10.00;
	psys->viscosity = 0.00;
	psys->k_spring = 3;
	psys->restLen = 1.0;
	psys->particles = vrArrayInit(vrArrayAlloc(), sizeof(vrParticle*));
	psys->gravity = vrVect(0, 9.81);
	srand(time(NULL));
	int amount = 0;
	vrFloat pw = VR_SQRT(amount);
	vrFloat dist = 0.1;

	vrVec2 opos = vrVect(lower_bound , lower_bound);
	vrVec2 pos = opos;
	
	for (int i = 0; i < pw*2; i++)
	{
		if (i % 2 == 0)
		{
			pos.x += dist/2.0;
		}
		if (i % 2 == 0)
		{
			pw--;
		}
		for (int j = 0; j < pw/2; j++)
		{
			
			//pos.x = lower_bound + (float)(rand()) / ((RAND_MAX / (upper_bound - lower_bound)));
			//pos.y = lower_bound + (float)(rand()) / ((RAND_MAX / (upper_bound - lower_bound)));
			vrParticle* p = vrParticleInit(vrParticleAlloc(), pos);
			vrArrayPush(psys->particles, p);
			pos.x = pos.x + dist;

		}
		if (i % 2 == 0)
		{
			pw++;
		}
		pos.y = pos.y + dist;
		pos.x = opos.x;

	}
	
	psys->pBody = vrBodyInit(vrBodyAlloc());
	psys->pBody->shape = vrShapeInit(vrShapeAlloc());
	psys->pBody->shape = vrShapeCircleInit(psys->pBody->shape);

	return psys;
}
vrFloat ct = 0;
void vrParticleSystemStep(vrParticleSystem * system, vrFloat dt)
{

	for (int i = 0; i < system->particles->sizeof_active; i++)
	{
		vrParticle* p = system->particles->data[i];
		p->vel = vrScale(vrSub(p->pos, p->oldp), 1.0 / dt);
	}

	for (int i = 0; i < system->particles->sizeof_active; i++)
	{
		vrParticle* p = system->particles->data[i];
		p->vel = vrAdd(p->vel, vrScale(system->gravity, dt));
	}

	for (int i = 0; i < system->particles->sizeof_active; i++)
	{
		vrParticle* p = system->particles->data[i];
		for (int j = 0; j < system->particles->sizeof_active; j++)
		{
			if (j > i) continue;

			vrParticle* p2 = system->particles->data[j];

			//Find distance
			vrVec2 diff = vrSub(p2->pos, p->pos);
			vrFloat dist = vrLengthSqr(diff);
			if (dist > p->r*p->r)
				continue;
			dist = VR_SQRT(dist);
			vrFloat q = dist / p->r;
			if (q < 1 && dist != 0)
			{
				vrVec2 norm = vrScale(diff, 1.0 / dist);
				vrFloat u = vrDot(vrSub(p->vel, p2->vel), norm);
				if (u > 0)
				{
					vrFloat Q = 0;
					vrFloat B = 1.0;
					vrVec2 I = vrScale(norm, dt * (1 - q)*(B*u + Q*(u*u)));
					//vrVec2Log(I);
					I = vrScale(I, 1.0 / 2.0);
					p->vel = vrSub(p->vel, I);
					p2->vel = vrAdd(p2->vel, I);

				}
			}
		}
	}
	for (int i = 0; i < system->particles->sizeof_active; i++)
	{
		vrParticle* p = system->particles->data[i];

		p->oldp = p->pos;
		p->pos = vrAdd(p->pos, vrScale(p->vel, dt));
	}
	vrFloat max_d = -10000;
	for (int i = 0; i < system->particles->sizeof_active; i++)
	{
		vrParticle* p = system->particles->data[i];
		p->d = 0.0;
		p->dNear = 0.0;
		//Calculate densities
		for (int j = 0; j < system->particles->sizeof_active; j++)
		{
			vrParticle* p2 = system->particles->data[j];

			//Find distance
			vrVec2 diff = vrSub(p->pos, p2->pos);
			vrFloat dist = vrLengthSqr(diff);
			if (dist > p->r*p->r)
				continue;
			dist = VR_SQRT(dist);
			vrFloat q = dist / p->r;
			if (q < 1)
			{
				p->d += VR_POW(1 - q, 2);
				p->dNear += VR_POW(1 - q, 3);
			}
			if (p->d > max_d) max_d = p->d;
		}
		//p->color.r = p->d/max_d;

		//Calculate pressure
		p->p = system->k_stiff * (p->d - system->resting_d);
		p->pNear = system->k_stiffN*p->dNear;
		vrVec2 dx = vrVect(0, 0);
		//Calculate forces
		//First add gravity
		for (int j = 0; j < system->particles->sizeof_active; j++)
		{
			//Remove duplicates
			vrParticle* p2 = system->particles->data[j];

			//Find distance
			vrVec2 diff = vrSub(p2->pos, p->pos);
			vrFloat dist = vrLengthSqr(diff);
			if (dist > p->r*p->r)
				continue;
			dist = VR_SQRT(dist);
			vrFloat q = (1 - dist / p->r);
			if (dist <= p->r)
			{
				vrFloat press = p->p + p2->p;
				vrFloat pressN = p->pNear + p2->pNear;
				vrFloat displace = (press*(q) + pressN*(q*q))*(dt*dt);
				if (dist != 0)
				{
					diff = vrScale(diff, 1.0 / dist);
					displace /= -2;
					dx = vrAdd(dx, vrScale(diff, displace));
					p2->pos = vrSub(p2->pos, vrScale(diff, displace));
				}

			}
		}
		p->pos = vrAdd(p->pos, dx);
	}
	for (int i = 0; i < system->particles->sizeof_active; i++)
	{
		vrParticle* p = system->particles->data[i];
		//p->vel = vrVect(0, 0);
	}
	vrParticleSystemBoundaries(system);
}

void vrParticleSystemBoundaries(vrParticleSystem * system)
{
	for (int i = 0; i < system->particles->sizeof_active; i++)
	{
		vrParticle* p = system->particles->data[i];

		if (p->pos.x <= lower_bound)
		{
			p->pos.x = lower_bound + EPSILON;
			p->vel.x = -0.2*p->vel.x;

		}
		else if (p->pos.x >= upper_bound)
		{
			p->pos.x = upper_bound - EPSILON;
			p->vel.x = -0.2*p->vel.x;

		}
		if (p->pos.y <= lower_bound)
		{
			p->pos.y = lower_bound + EPSILON;
			p->vel.y = -0.2*p->vel.y;
		}
		else if (p->pos.y >= upper_bound)
		{
			p->pos.y = upper_bound - EPSILON;
			p->vel.y = -0.2*p->vel.y;
		}
	}
}

void vrParticleSystemCollide(vrParticleSystem * system, vrRigidBody * body, vrFloat scale, vrFloat dt)
{
	vrFloat r = 0;
	vrFloat m = 0;
	vrFloat displaced = 0;
	vrFloat im = body->bodyMaterial.invMass;
	body->bodyMaterial.invMass = 1.0 / 1000*im;
	for (int i = 0; i < system->particles->sizeof_active; i++)
	{
		vrParticle* p = system->particles->data[i];
		vrOrientedBoundingBox b;
		b.position = vrSub(vrScale(p->pos, scale), vrVect(p->r * scale, p->r* scale));
		b.size = vrVect(p->r * 2 * scale, p->r * 2 * scale);
		r = (p->r * scale);

		if (vrOBBOverlaps(b, body->shape->obb))
		{

			system->pBody->velocity = vrScale(p->vel, scale);
			system->pBody->bodyMaterial.invMomentInertia = 0;
			system->pBody->bodyMaterial.restitution = 0;
			system->pBody->bodyMaterial.friction = 0;
			system->pBody->bodyMaterial.invMass = 1.0/0.6;
			m = system->pBody->bodyMaterial.mass;

			((vrCircleShape*)system->pBody->shape->shape)->center = vrScale(p->pos, scale);
			((vrCircleShape*)system->pBody->shape->shape)->radius = p->r * scale;

			if (body->shape->shapeType == VR_POLYGON)
			{
				vrManifold* manifold = vrManifoldInit(vrManifoldAlloc());
				vrCirclePoly(manifold, *((vrCircleShape*)system->pBody->shape->shape), *((vrPolygonShape*)body->shape->shape));
				vrManifoldSetBodies(manifold, system->pBody, body);
				vrManifoldPreStep(manifold, dt);
				vrManifoldSolveVelocity(manifold);
				vrManifoldPostStep(manifold, dt);
				vrManifoldSolvePosition(manifold, dt);
				p->vel = vrScale(system->pBody->velocity, 1.0 / scale);
				vrVec2 d = vrScale(system->pBody->velocity, 1.0 / scale);
				displaced += vrLength(d) / r;
				p->pos = vrAdd(p->pos, vrScale(system->pBody->vel_bias, 1.0/scale * dt));
				d = vrScale(system->pBody->vel_bias, 1.0 / scale * dt);
				displaced += vrLength(d) / r;
				system->pBody->vel_bias = vrVect(0, 0);

				vrManifoldDestroy(manifold);
			}
			else if (body->shape->shapeType == VR_CIRCLE)
			{
				vrManifold* manifold = vrManifoldInit(vrManifoldAlloc());
				vrCircleCircle(manifold, *((vrCircleShape*)system->pBody->shape->shape), *((vrCircleShape*)body->shape->shape));
				vrManifoldSetBodies(manifold, system->pBody, body);
				vrManifoldPreStep(manifold, dt);
				vrManifoldSolveVelocity(manifold);
				vrManifoldPostStep(manifold, dt);
				vrManifoldSolvePosition(manifold, dt);
				p->vel = vrScale(system->pBody->velocity, 1.0 / scale);
				p->pos = vrAdd(p->pos, vrScale(system->pBody->vel_bias, 1.0 / scale * dt));
				system->pBody->vel_bias = vrVect(0, 0);
				vrManifoldDestroy(manifold);

			}
		}
	}
	body->bodyMaterial.invMass = im;

	return;
	if (body->bodyMaterial.invMass == 0) return;
	if (displaced < EPSILON) return;
	vrVec2 gravity = vrVect(0, 9.81);
	vrVec2 weight = vrScale(gravity, body->bodyMaterial.mass);
	vrFloat area = VR_PI*(r*r);
	vrFloat density = m / area;
	vrVec2 buoyancy = vrScale(vrVect(-gravity.x, -gravity.y), density * displaced);
	vrVec2 total = vrAdd(weight, buoyancy);
	if (vrLength(total) < 1) return;
	body->velocity = vrAdd(body->velocity, vrScale(total, body->bodyMaterial.invMass));
}
