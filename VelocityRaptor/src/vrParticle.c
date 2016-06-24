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

#include "../include/vrParticle.h"

vrParticle * vrParticleAlloc()
{
	return vrAlloc(sizeof(vrParticle));
}

vrParticle * vrParticleInit(vrParticle * particle, vrVec2 p)
{
	particle->pos = p;
	particle->oldp = p;
	particle->vel = vrVect(0, 0);
	particle->acc = vrVect(0, 0);
	particle->force = vrVect(0, 0);

	particle->d = 100;
	particle->p = 0;
	particle->r = 0.190625;
	particle->m = 3.8;
	particle->damping = 0.01;

	return particle;
}

void vrParticleIntegrate(vrParticle * particle, vrFloat dt)
{
	//particle->pos = vrAdd(particle->pos, vrAdd(vrSub(particle->pos, particle->oldp), vrScale(particle->acc, dt*dt)));
	//xi+1 = xi + (xi - xi-1) + a * dt * dt

	vrVec2 t;
	vrVec2 oldPos = particle->pos;
	particle->acc = vrScale(particle->acc, dt*dt);
	t = vrSub(particle->pos, particle->oldp);
	t = vrScale(t, (1.0 - particle->damping));
	t = vrAdd(t, particle->acc);
	particle->pos = vrAdd(particle->pos, t);
	particle->oldp = oldPos;
	t = vrSub(particle->pos, particle->oldp);
	particle->vel = vrScale(t, 1.0 / dt);

	particle->force = vrVect(0, 0);
}
