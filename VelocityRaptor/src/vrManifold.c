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

#include "..\include\vrManifold.h"
#define GLEW_STATIC
#include <glew.h>
#include <glfw3.h>

vrManifold* vrManifoldAlloc()
{
	return vrAlloc(sizeof(vrManifold));
}

vrManifold * vrManifoldInit(vrManifold * manifold)
{
	manifold->A = NULL;
	manifold->B = NULL;
	manifold->active = vrFALSE;
	manifold->firstTime = vrTRUE;
	manifold->normal = vrVect(0, 0);
	manifold->penetration = 0;
	manifold->contact_points = 0;
	manifold->flip = vrFALSE;
	manifold->contacts[0].point = vrVect(0, 0);
	manifold->contacts[0].depth = 0;
	manifold->contacts[0].normalImpulseSum = 0;
	manifold->contacts[0].tangentImpulseSum = 0;
	manifold->contacts[0].bias = 0;
	manifold->contacts[0].biasImpulseSum = 0;
	manifold->contacts[0].effectiveMassN = 0;
	manifold->contacts[0].effectiveMassT = 0;
	manifold->contacts[0].ra = vrVect(0, 0);
	manifold->contacts[0].rb = vrVect(0, 0);

	manifold->contacts[1].point = vrVect(0, 0);
	manifold->contacts[1].depth = 0;
	manifold->contacts[1].normalImpulseSum = 0;
	manifold->contacts[1].tangentImpulseSum = 0;
	manifold->contacts[1].bias = 0;
	manifold->contacts[1].biasImpulseSum = 0;
	manifold->contacts[1].effectiveMassN = 0;
	manifold->contacts[1].effectiveMassT = 0;
	manifold->contacts[1].ra = vrVect(0, 0);
	manifold->contacts[1].rb = vrVect(0, 0);
	return manifold;
}

void vrManifoldDestroy(vrManifold* manifold)
{
	vrFree(manifold);
}

void vrManifoldPreStep(vrManifold * manifold, vrFloat dt)
{
	vrFloat k_bias = 0.034;
	vrFloat allowedPenetration = 0.05;
	manifold->restitution = VR_MAX(manifold->A->bodyMaterial.restitution, manifold->B->bodyMaterial.restitution);
	manifold->friction = sqrt(manifold->A->bodyMaterial.friction* manifold->A->bodyMaterial.friction + manifold->B->bodyMaterial.friction*manifold->B->bodyMaterial.friction);
	for (int i = 0; i < manifold->contact_points; i++)
	{
		manifold->contacts->biasImpulseSum = 0;
		vrVec2 aCenter = manifold->A->center;
		vrVec2 bCenter = manifold->B->center;

		vrVec2 ra = vrSub(manifold->contacts[i].point, aCenter);
		vrVec2 rb = vrSub(manifold->contacts[i].point, bCenter);

		manifold->contacts[i].velocityBias = 0;

		vrFloat rv = vrDot(manifold->normal, vrManifoldRelativeVelocity(manifold->A, manifold->B, ra, rb));
		if (rv < -VR_VELOCITYTHRESHOLD)
		{
			manifold->contacts[i].velocityBias = rv * manifold->restitution;
		}

		manifold->contacts[i].ra = ra;
		manifold->contacts[i].rb = rb;

		vrFloat ran = vrCross(ra, manifold->normal);
		vrFloat rbn = vrCross(rb, manifold->normal);

		manifold->contacts[i].effectiveMassN = manifold->A->bodyMaterial.invMass + manifold->B->bodyMaterial.invMass + (ran*ran) * manifold->A->bodyMaterial.invMomentInertia + (rbn*rbn) * manifold->B->bodyMaterial.invMomentInertia;
		manifold->contacts[i].effectiveMassN = 1.0 / manifold->contacts[i].effectiveMassN;

		manifold->tangent = vrCrossScalar(1.0, manifold->normal);

		vrFloat rat = vrCross(ra, manifold->tangent);
		vrFloat rbt = vrCross(rb, manifold->tangent);
		manifold->contacts[i].effectiveMassT = manifold->A->bodyMaterial.invMass + manifold->B->bodyMaterial.invMass + (rat*rat) * manifold->A->bodyMaterial.invMomentInertia + (rbt*rbt) * manifold->B->bodyMaterial.invMomentInertia;
		manifold->contacts[i].effectiveMassT = 1.0 / manifold->contacts[i].effectiveMassT;

		manifold->contacts[i].bias = 0;
		manifold->contacts[i].bias = -k_bias * VR_MIN((vrFloat)0.0, manifold->contacts[i].depth + allowedPenetration) / dt;

		vrVec2 impulse = vrAdd(vrScale(manifold->normal, manifold->contacts[i].normalImpulseSum), vrScale(manifold->tangent, manifold->contacts[i].tangentImpulseSum));

		vrManifoldApplyImpulse(manifold->A, manifold->B, ra, rb, impulse);
	}

	if (manifold->contact_points == 2)
	{

		manifold->solverData.effMass = vrVect(-1.0 / manifold->contacts[0].effectiveMassN, -1.0 / manifold->contacts[1].effectiveMassN);
		manifold->solverData.hi = vrVect(MAX_IMPULSE, MAX_IMPULSE);
		manifold->solverData.initialGuess = vrVect(0, 0);
		manifold->solverData.A = vrMat(vrVect(manifold->solverData.effMass.x, 0), vrVect(0, manifold->solverData.effMass.y));

	}
}

void vrManifoldPostStep(vrManifold * manifold, vrFloat dt)
{
	vrFloat k_bias = 0.35;
	vrFloat allowedPenetration = 0.1;
	for (int i = 0; i < manifold->contact_points; i++)
	{
		
		//Recompute points
		vrVec2 rb = vrGetLocalPoint(&manifold->contacts[i].contactAnchor);
		vrVec2 p = vrAdd(manifold->B->center, rb);
		vrVec2 aCenter = manifold->A->center;
		
		vrVec2 ra = vrSub(p, aCenter);
		vrFloat ran = vrCross(ra, manifold->normal);
		vrFloat rbn = vrCross(rb, manifold->normal);
		//Recompute eff mass
		manifold->contacts[i].effectiveMassN = manifold->A->bodyMaterial.invMass + manifold->B->bodyMaterial.invMass + (ran*ran) * manifold->A->bodyMaterial.invMomentInertia + (rbn*rbn) * manifold->B->bodyMaterial.invMomentInertia;
		manifold->contacts[i].effectiveMassN = 1.0 / manifold->contacts[i].effectiveMassN;
		//Recompute error and bias
		vrFloat oldp = vrDot(manifold->normal, manifold->contacts[i].point);
		vrFloat new_depth = VR_MIN(0.0, manifold->contacts[i].depth + vrDot(manifold->normal, p) - oldp);
		manifold->contacts[i].bias = -k_bias * VR_MIN((vrFloat)0.0, new_depth + allowedPenetration) / dt;
		manifold->contacts[i].ra = ra;
		manifold->contacts[i].rb = rb;

	}
	if (manifold->contact_points == 2)
	{
		manifold->solverData.effMass = vrVect(-1.0/manifold->contacts[0].effectiveMassN, -1.0/manifold->contacts[1].effectiveMassN);

		manifold->solverData.A = vrMat(vrVect(manifold->solverData.effMass.x, 0), vrVect(0, manifold->solverData.effMass.y));
	}
}

inline vrVec2 vrManifoldRelativeVelocity(vrRigidBody * a, vrRigidBody * b, vrVec2 ra, vrVec2 rb)
{
	return vrSub(vrAdd(b->velocity, vrCrossScalar(b->angularVelocity, rb)), vrAdd(a->velocity, vrCrossScalar(a->angularVelocity, ra)));
}

void vrManifoldSolveVelocity(vrManifold * manifold)
{
	int cp = manifold->contact_points;
	
	for (int i = 0; i < cp; i++)
	{
		vrFloat tangentVel = vrDot(manifold->tangent, vrManifoldRelativeVelocity(manifold->A, manifold->B, manifold->contacts[i].ra, manifold->contacts[i].rb));
		vrFloat j = (-tangentVel) * manifold->contacts[i].effectiveMassT;
		vrFloat maxA = manifold->contacts[i].normalImpulseSum * manifold->friction;

		vrFloat oldAccum = manifold->contacts[i].tangentImpulseSum;
		manifold->contacts[i].tangentImpulseSum = vrClamp(oldAccum + j, -maxA, maxA);
		j = manifold->contacts[i].tangentImpulseSum - oldAccum;

		vrVec2 im = vrScale(manifold->tangent, j);

		vrBodyApplyImpulse(manifold->A, vrVect(-im.x, -im.y), manifold->contacts[i].ra);
		vrBodyApplyImpulse(manifold->B, im, manifold->contacts[i].rb);
	}

	if (cp == 1)
	{
		for (int i = 0; i < cp; i++)
		{
			vrFloat contactVel = vrManifoldGetContactVel(manifold, i);
			vrFloat j = (manifold->contacts[i].bias + -contactVel - manifold->contacts[i].velocityBias) * manifold->contacts[i].effectiveMassN;

			vrFloat oldAccum = manifold->contacts[i].normalImpulseSum;
			manifold->contacts[i].normalImpulseSum = VR_MAX(j + oldAccum, 0.0);
			j = manifold->contacts[i].normalImpulseSum - oldAccum;
			vrVec2 im = vrScale(manifold->normal, j);

			vrBodyApplyImpulse(manifold->A, vrVect(-im.x, -im.y), manifold->contacts[i].ra);
			vrBodyApplyImpulse(manifold->B, im, manifold->contacts[i].rb);
		}
	}
	else
	{
		manifold->solverData.contactVel = vrVect(-manifold->contacts[0].bias +  vrManifoldGetContactVel(manifold, 0) + manifold->contacts[0].velocityBias, -manifold->contacts[1].bias + vrManifoldGetContactVel(manifold, 1) + manifold->contacts[1].velocityBias );

		vrVec2 t = vrManifoldConjugateGradient(manifold->solverData, 1e-10);

		vrFloat oldAccum = manifold->contacts[0].normalImpulseSum;
		manifold->contacts[0].normalImpulseSum = VR_MAX(t.x + oldAccum, 0.0);
		t.x = manifold->contacts[0].normalImpulseSum - oldAccum;
		oldAccum = manifold->contacts[1].normalImpulseSum;
		manifold->contacts[1].normalImpulseSum = VR_MAX(t.y + oldAccum, 0.0);
		t.y = manifold->contacts[1].normalImpulseSum - oldAccum;

		vrVec2 i, i2;
		i = vrScale(manifold->normal, t.x);
		i2 = vrScale(manifold->normal, t.y);
		vrBodyApplyImpulse(manifold->A, vrVect(-i.x, -i.y), manifold->contacts[0].ra);
		vrBodyApplyImpulse(manifold->A, vrVect(-i2.x, -i2.y), manifold->contacts[1].ra);

		vrBodyApplyImpulse(manifold->B, i, manifold->contacts[0].rb);
		vrBodyApplyImpulse(manifold->B, i2, manifold->contacts[1].rb);
	}

}

void vrManifoldSolvePosition(vrManifold * manifold, vrFloat dt)
{
	if (manifold->contact_points == 1)
	{
		for (int i = 0; i < manifold->contact_points; i++)
		{
			vrFloat bn = (manifold->contacts[i].bias - vrDot(vrSub(vrAdd(manifold->B->vel_bias, vrCrossScalar(manifold->B->angv_bias, manifold->contacts[i].rb)), vrAdd(manifold->A->vel_bias, vrCrossScalar(manifold->A->angv_bias, manifold->contacts[i].ra))), manifold->normal)) * manifold->contacts[i].effectiveMassN;
			vrFloat oldbn = manifold->contacts[i].biasImpulseSum;
			manifold->contacts[i].biasImpulseSum = VR_MAX(oldbn + bn, 0);
			bn = manifold->contacts[i].biasImpulseSum - oldbn;

			vrManifoldApplyBiasImpulse(manifold->A, manifold->B, manifold->contacts[i].ra, manifold->contacts[i].rb, vrScale(manifold->normal, bn));
		}
	}
	else
	{
		vrBlockSolverData solverData = manifold->solverData;
		solverData.contactVel.x = -(manifold->contacts[0].bias - vrDot(vrSub(vrAdd(manifold->B->vel_bias, vrCrossScalar(manifold->B->angv_bias, manifold->contacts[0].rb)), vrAdd(manifold->A->vel_bias, vrCrossScalar(manifold->A->angv_bias, manifold->contacts[0].ra))), manifold->normal));
		solverData.contactVel.y = -(manifold->contacts[1].bias - vrDot(vrSub(vrAdd(manifold->B->vel_bias, vrCrossScalar(manifold->B->angv_bias, manifold->contacts[1].rb)), vrAdd(manifold->A->vel_bias, vrCrossScalar(manifold->A->angv_bias, manifold->contacts[1].ra))), manifold->normal));
		vrVec2 t = vrManifoldConjugateGradient(solverData, 1e-4);

		vrFloat oldbn = manifold->contacts[0].biasImpulseSum;
		manifold->contacts[0].biasImpulseSum = VR_MAX(oldbn + t.x, 0);
		t.x = (manifold->contacts[0].biasImpulseSum - oldbn);
		oldbn = manifold->contacts[1].biasImpulseSum;
		manifold->contacts[1].biasImpulseSum = VR_MAX(oldbn + t.y, 0);
		t.y = (manifold->contacts[1].biasImpulseSum - oldbn);

		vrManifoldApplyBiasImpulse(manifold->A, manifold->B, manifold->contacts[0].ra, manifold->contacts[0].rb, vrScale(manifold->normal, t.x));
		vrManifoldApplyBiasImpulse(manifold->A, manifold->B, manifold->contacts[1].ra, manifold->contacts[1].rb, vrScale(manifold->normal, t.y));
	}

}

vrVec2 vrManifoldGuassSeidel(vrBlockSolverData solverData)
{
	int iter = 100;
	vrVec2 x = vrVect(0, 0);
	float delta = 0;
	//Relaxation
	vrFloat w = 0.78;
	//Guass-Seidel with SOR
	//I simplified it because I knew the size
	//Of the vectors and matrix
	//Also, for now only one iteration
	for (int k = 0; k < iter; k++)
	{
		//w = 1 / (k+1);
		delta = 0.0f;
		delta += solverData.effMass.x * x.x;
		delta = (solverData.contactVel.x - delta) / solverData.effMass.x;
		x.x += w * (delta - x.x);

		if (x.x < solverData.lo.x)
		{
			x.x = solverData.lo.x;
		}
		if (x.x > solverData.hi.x)
		{
			x.x = solverData.hi.x;
		}

		delta = 0.0f;
		delta += solverData.effMass.y * x.y;
		delta = (solverData.contactVel.y - delta) / solverData.effMass.y;
		x.y += w * (delta - x.y);

		if (x.y < solverData.lo.y)
		{
			x.y = solverData.lo.y;
		}
		if (x.y > solverData.hi.y)
		{
			x.y = solverData.hi.y;
		}

	}
	return x;
}

vrVec2 vrManifoldConjugateGradient(vrBlockSolverData solverData, vrFloat tolerance)
{
	/* Conjugate Gradient solver
	* Based off of psuedo code in
	* http://www.cs.usfca.edu/~peter/cs625/prog2.pdf
	*/
	vrMat2 A = solverData.A;
	vrVec2 b = solverData.contactVel;
	vrVec2 x = vrVect(0, 0);

	int k = 0;
	int max_iterations = 100;
	vrVec2 r = b;
	vrVec2 rp = b;
	vrVec2 rpp = b;
	vrVec2 p = vrVect(0, 0);
	vrVec2 pp = vrVect(0, 0);
	vrVec2 xp = vrVect(0, 0);
	vrVec2 oldScaled = vrVect(0, 0);
	vrVec2 s = vrVect(0, 0);
	vrFloat alpha = 0;
	while ((k < max_iterations) && (vrDot(r, r) > tolerance))
	{
		rpp = rp;
		rp = r;
		pp = p;
		xp = x;
		k++;
		if (k == 1)
		{
			p = rp;
			pp = p;
		}
		else
		{
			vrFloat k = vrDot(rp, rp) / vrDot(rpp, rpp);
			vrVec2 oldScaled = vrScale(pp, k);
			p = vrAdd(r, oldScaled);
		}
		s = vrMat2Mult(A, p);

		vrFloat d1 = vrDot(rp, rp);
		vrFloat d2 = vrDot(p, s);
		alpha = d1 / d2;

		oldScaled = vrScale(p, alpha);
		x = vrAdd(xp, oldScaled);

		oldScaled = vrScale(s, alpha);
		r = vrSub(rp, oldScaled);
	}

	return x;

}

void vrManifoldApplyImpulse(vrRigidBody * A, vrRigidBody * B, vrVec2 ra, vrVec2 rb, vrVec2 impulse)
{
	
	A->velocity = vrSub(A->velocity, vrScale(impulse, A->bodyMaterial.invMass));
	A->angularVelocity -= A->bodyMaterial.invMomentInertia * vrCross(ra, impulse);

	B->velocity = vrAdd(B->velocity, vrScale(impulse, B->bodyMaterial.invMass));
	B->angularVelocity += B->bodyMaterial.invMomentInertia * vrCross(rb, impulse);
}

void vrManifoldApplyBiasImpulse(vrRigidBody * A, vrRigidBody * B, vrVec2 ra, vrVec2 rb, vrVec2 impulse)
{
	A->vel_bias = vrSub(A->vel_bias, vrScale(impulse, A->bodyMaterial.invMass));
	A->angv_bias -= A->bodyMaterial.invMomentInertia * vrCross(ra, impulse);

	B->vel_bias = vrAdd(B->vel_bias, vrScale(impulse, B->bodyMaterial.invMass));
	B->angv_bias += B->bodyMaterial.invMomentInertia * vrCross(rb, impulse);
}

void vrManifoldAddContactPoints(vrManifold* old_manifold, const vrManifold new_manifold)
{
	old_manifold->active = new_manifold.active;
	vrContact merged[2];
	for (int i = 0; i < new_manifold.contact_points; i++)
	{
		merged[i] = new_manifold.contacts[i];

		if (i < old_manifold->contact_points)
		{
			merged[i].normalImpulseSum = old_manifold->contacts[i].normalImpulseSum;
			merged[i].tangentImpulseSum = old_manifold->contacts[i].tangentImpulseSum;
		}
	}
	for (int i = 0; i < new_manifold.contact_points; i++)
		old_manifold->contacts[i] = merged[i];

	old_manifold->A = new_manifold.A;
	old_manifold->B = new_manifold.B;
	old_manifold->normal = new_manifold.normal;
	old_manifold->contact_points = new_manifold.contact_points;
	old_manifold->flip = new_manifold.flip;
	old_manifold->penetration = new_manifold.penetration;
}

void vrManifoldSetBodies(vrManifold * manifold, vrRigidBody * b1, vrRigidBody * b2)
{
	if (manifold->flip)
	{
		manifold->A = b1;
		manifold->B = b2;
	}
	else
	{
		manifold->A = b2;
		manifold->B = b1;
	}
	///B is the body with the incident face
	///So both points are on body B
	for (int i = 0; i < manifold->contact_points; i++)
	{
		vrContact* c = manifold->contacts + i;
		c->contactAnchor = vrLocalPointInit(manifold->B, c->point);
	}
}

inline vrFloat vrManifoldGetContactVel(vrManifold * manifold, int index)
{
	return vrDot(vrManifoldRelativeVelocity(manifold->A, manifold->B, manifold->contacts[index].ra, manifold->contacts[index].rb), manifold->normal);
}
