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

vrParticleSystem * vrParticleSystemAlloc()
{
	return vrAlloc(sizeof(vrParticleSystem));
}

vrParticleSystem * vrParticleSystemInit(vrParticleSystem * psys)
{
	psys->resting_d = 5;
	psys->k_stiff = 1.0;
	psys->viscosity = 0.002;
	psys->epsilon = 0.00001f;
	psys->gamma = 3.0;
	psys->particles = vrArrayInit(vrArrayAlloc(), sizeof(vrParticle*));
	psys->gravity = vrVect(0, 1.81);
	srand(time(NULL));
	int amount = 100;
	vrFloat pw = VR_SQRT(amount);
	vrFloat dist = 0.075*3;

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
	return psys;
}

void vrParticleSystemStep(vrParticleSystem * system, vrFloat dt)
{

	for (int i = 0; i < system->particles->sizeof_active; i++)
	{
		vrParticle* p = system->particles->data[i];
		//Calculate densities
		for (int j = 0; j < system->particles->sizeof_active; j++)
		{
			if (j > i) continue;

			vrParticle* p2 = system->particles->data[j];

			//Find distance
			vrVec2 diff = vrSub(p->pos, p2->pos);
			vrFloat dist = vrLength(diff);
			if (dist < p->r * 6)
			{
				p->d += p2->m * W(dist, p->r);
			}
		}
		//Calculate pressure
		p->p = system->k_stiff * (p->d - system->resting_d);
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
			if (dist < p->r*6)
			{
				//Calculate force due to pressure
				vrVec2 gradient = GradW(diff, p->r);
				// pressure = -(mj * (pi + pj) / ( 2.0 * dj) * W(r - rb, h))
				if (p2->d <= system->epsilon)
					p2->d = system->epsilon;

				vrVec2 pressure = vrScale(gradient, p2->m * (p->p + p2->p) / (2.0 * p2->d));

				p->force = vrSub(p->force, pressure);	
				p2->force = vrAdd(p2->force, pressure);
			}
		}
		//printf("%f and %f this\n", p->force.x, p->force.y);
	}
	for (int i = 0; i < system->particles->sizeof_active; i++)
	{
		vrParticle* p = system->particles->data[i];
		p->acc = vrScale(p->force, 1.0 / p->m);

		vrParticleIntegrate(p, dt);
	}

	vrParticleSystemBoundaries(system);
	for (int i = 0; i < system->particles->sizeof_active; i++)
	{
		vrParticle* p = system->particles->data[i];
		vrFloat min_dist = p->r * 2;

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
					if (dist > system->epsilon)
					{
						vrVec2 move = vrScale(diff, 0.5f * (dist - min_dist) / dist);
						move = vrVect(-move.x, -move.y);
						p2->pos = vrSub(p2->pos, move);
						p2->oldp = vrSub(p2->oldp, move);
						p->pos = vrAdd(p->pos, move);
						p->oldp = vrAdd(p->oldp, move);

					}
					else
					{

						vrFloat ndiff = 0.5 * min_dist;
						p2->pos.y -= ndiff;
						p2->oldp.y -= ndiff;
						p->pos.y += ndiff;
						p->oldp.y += ndiff;

					}
				}
			}
		}

	}
}

void vrParticleSystemBoundaries(vrParticleSystem * system)
{
	for (int i = 0; i < system->particles->sizeof_active; i++)
	{
		vrParticle* p = system->particles->data[i];

		if (p->pos.x < lower_bound)
		{
			p->pos.x = lower_bound;
		}
		else if (p->pos.x > upper_bound)
		{
			p->pos.x = upper_bound;
		}
		if (p->pos.y < lower_bound)
		{
			p->pos.y = lower_bound;
		}
		else if (p->pos.y > upper_bound)
		{
			p->pos.y = upper_bound;
		}
	}
}



vrFloat W(vrFloat x, vrFloat h)
{
	if (x*x <= 0 || x*x > h*h) return 0.0;
	return (315.0 / (64.0 * VR_PI * VR_POWF(h, 9)))*VR_POWF((h*h - x*x), 3);
}

vrVec2 GradW(vrVec2 d, vrFloat h)
{
	double kernelRad6 = VR_POWF((double)h, 6.0);
	vrFloat m_factor = (float)(15.0 / (VR_PI * kernelRad6));
	vrFloat dlen = vrLength(d);
	vrFloat dlens = dlen * dlen;
	if (dlens > h*h) return vrVect(0, 0);
	if (dlens <= 0.0)
		return vrVect(0, 0);

	vrVec2 r =  vrScale(d, -m_factor * 3.0f * (h - dlen) * (h - dlen) / dlen);
	return r;
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