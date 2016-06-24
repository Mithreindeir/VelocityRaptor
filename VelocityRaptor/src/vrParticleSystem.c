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

#include "../include/vrParticleSystem.h"
#include "../include/velocityraptor.h"
#define EPSILON 1.192092896e-07f
vrParticleSystem * vrParticleSystemAlloc()
{
	return vrAlloc(sizeof(vrParticleSystem));
}

vrParticleSystem * vrParticleSystemInit(vrParticleSystem * psys)
{
	psys->resting_d = 80;
	psys->k_stiff = 0.1;
	psys->viscosity = 0.00;
	psys->epsilon = 0.00001f;
	psys->gamma = 2.0;
	psys->particles = vrArrayInit(vrArrayAlloc(), sizeof(vrParticle*));
	psys->gravity = vrVect(0, 9.81);
	srand(time(NULL));
	int amount = 100;
	vrFloat pw = VR_SQRT(amount);
	vrFloat dist = 0.1;

	vrVec2 opos = vrVect(lower_bound , lower_bound);
	vrVec2 pos = opos;
	for (int i = 0; i < pw; i++)
	{
		if (i % 2 == 0)
		{
			pos.x += dist/2.0;
		}
		if (i % 2 == 0)
		{
			pw--;
		}
		for (int j = 0; j < pw; j++)
		{
			pos.x = lower_bound + (float)(rand()) / ((RAND_MAX / (upper_bound - lower_bound)));
			pos.y = lower_bound + (float)(rand()) / ((RAND_MAX / (upper_bound - lower_bound)));
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
	return psys;
}

void vrParticleSystemStep(vrParticleSystem * system, vrFloat dt)
{

	for (int i = 0; i < system->particles->sizeof_active; i++)
	{
		vrParticle* p = system->particles->data[i];
		p->d = 0.0;
		//Calculate densities
		for (int j = 0; j < system->particles->sizeof_active; j++)
		{
			//if (j > i) continue;

			vrParticle* p2 = system->particles->data[j];

			//Find distance
			vrVec2 diff = vrSub(p->pos, p2->pos);
			vrFloat dist = vrLength(diff);
			if (dist <= p->r)
			{
				p->d += p2->m * W(dist, p->r);		
			}
		}
		//Calculate pressure
		p->p = system->k_stiff * (VR_POW(p->d / system->resting_d, system->gamma) - 1.0);
	}
	for (int i = 0; i < system->particles->sizeof_active; i++)
	{
		vrParticle* p = system->particles->data[i];
		//Calculate forces
		//First add gravity
		p->force = vrAdd(p->force, system->gravity);
		for (int j = 0; j < system->particles->sizeof_active; j++)
		{
			//Remove duplicates
			if (j > i) continue;
			vrParticle* p2 = system->particles->data[j];

			//Find distance
			vrVec2 diff = vrSub(p->pos, p2->pos);
			vrFloat dist = vrLength(diff);
			if (dist <= p->r)
			{
				//Calculate force due to pressure
				vrVec2 gradient = GradW(diff, p->r);
				// pressure = -(mj * (pi + pj) / ( 2.0 * dj) * W(r - rb, h))
				// f = particles[nIdx].Mass * ((particles[nIdx].Velocity - particles[i].Velocity) / particles[nIdx].Density) * WViscosityLap(ref dist) * Constants.VISC0SITY;

				vrVec2 pressure = vrScale(gradient, p2->m * (p->p + p2->p) / (2.0 * p2->d));
				vrVec2 viscosity = vrScale((vrScale(vrScale(vrSub(p2->vel, p->vel), 1.0 / p2->d), LaplacianW(dist, p->r))), p2->m * system->viscosity);

				if (p2->d >= EPSILON)
				{

					p2->force = vrAdd(p2->force, pressure);
					p->force = vrSub(p->force, pressure);
					p2->force = vrSub(p2->force, viscosity);
					p->force = vrAdd(p->force, viscosity);

				}


			}
		}
	}
	for (int i = 0; i < system->particles->sizeof_active; i++)
	{
		vrParticle* p = system->particles->data[i];
		p->acc = vrScale(p->force, 1.0 / p->m);

		vrParticleIntegrate(p, dt);
	}

	vrParticleSystemBoundaries(system);
	/*
	for (int i = 0; i < system->particles->sizeof_active; i++)
	{
		vrParticle* p = system->particles->data[i];
		vrFloat min_dist = p->r*1.5;

		//Correct particles if they are too close
		for (int j = 0; j < system->particles->sizeof_active; j++)
		{
			if (j > i) continue;
			vrParticle* p2 = system->particles->data[j];

			//Find distance
			vrVec2 diff = vrSub(p->pos, p2->pos);
			vrFloat dist = vrLength(diff);
			//if (dist < p->r)
			{

				if (dist < min_dist)
				{
					if (dist < system->epsilon)
						dist = system->epsilon;
					vrVec2 move = vrScale(diff, 0.5f * (dist - min_dist) / dist);
					move = vrVect(-move.x, -move.y);
					p2->pos = vrSub(p2->pos, move);
					p2->oldp = vrSub(p2->oldp, move);
					p->pos = vrAdd(p->pos, move);
					p->oldp = vrAdd(p->oldp, move);
				}
			}
		}

	}
	*/
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



vrFloat W(vrFloat x, vrFloat h)
{

	vrFloat lenSq = x*x;
	if (lenSq > h*h)
	{
		return 0.0f;
	}
	if (lenSq < EPSILON)
	{
		lenSq = EPSILON;
	}
		
	return (315.0 / (64.0 * VR_PI * VR_POWF(h, 9)))*VR_POWF((h*h - lenSq), 3);
}

vrVec2 GradW(vrVec2 d, vrFloat h)
{

	vrFloat kr6 = VR_POW(h, 6);
	vrFloat m_factor = (vrFloat)(15.0 / (VR_PI * kr6));

	float lenSq = vrLengthSqr(d);
	if (lenSq >h*h)
	{
		return vrVect(0.0f, 0.0f);
	}
	if (lenSq < EPSILON)
	{
		lenSq = EPSILON;
	}
	float len = VR_SQRT(lenSq);
	vrFloat f = -m_factor * 3.0 * (h - len) * (h - len) / len;
	vrVec2 result = vrScale(d, f);
	return result;

}

vrFloat LaplacianW(vrFloat x, vrFloat h)
{
	vrFloat k3 = (h*h*h);
	vrFloat m_factor = (float)(15.0 / (2.0f * VR_PI * k3));
	if (x*x > h*h)
		return 0.0;
	if (x*x <= 0.0)
		return 0.0;
	return m_factor * (6.0 / k3) * (h - x);
}