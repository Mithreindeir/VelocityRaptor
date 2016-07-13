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
#include "..\include\vrDistanceJoint.h"

vrDistanceJoint * vrDistanceAlloc()
{
	return vrAlloc(sizeof(vrDistanceJoint));
}

vrDistanceJoint * vrDistanceInit(vrDistanceJoint * joint)
{
	joint->accumImpulse = 0;
	joint->stiffness = 0.1;
	joint->bias = 0;
	joint->dir = vrVect(0, 0);
	joint->invMass = 0;
	joint->ra = vrVect(0, 0);
	joint->rb = vrVect(0, 0);
	return joint;
}

vrJoint * vrDistanceJointInit(vrJoint * joint, vrRigidBody* A, vrRigidBody* B, vrVec2 pointA, vrVec2 pointB)
{
	joint = vrJointInit(joint);
	joint->A = A;
	joint->B = B;
	joint->anchorA = vrLocalPointInit(A, pointA);
	joint->anchorB = vrLocalPointInit(B, pointB);
	joint->preSolve = &vrDistanceJointPreSolve;
	joint->solveVelocity = &vrDistanceJointSolve;
	joint->postSolve = &vrDistanceJointPostSolve;
	joint->solvePosition = &vrDistanceJointSolvePosition;

	joint->jointData = vrDistanceInit(vrDistanceAlloc());
	vrDistanceJoint* sj = joint->jointData;
	sj->bias = 0;
	sj->ra = vrGetLocalPoint(&joint->anchorA);
	sj->rb = vrGetLocalPoint(&joint->anchorB);
	sj->damping = 0.4;

	vrVec2 ra = vrGetLocalPoint(&joint->anchorA);
	vrVec2 rb = vrGetLocalPoint(&joint->anchorB);
	vrVec2 ca = joint->A->center;
	vrVec2 cb = joint->B->center;

	vrVec2 n = vrSub(pointB, pointA);
	vrFloat len = vrLength(n);

	sj->restLength = 120;

	return joint;
}

void vrDistanceJointDestroy(vrJoint * joint)
{
	vrFree(joint->jointData);
	vrFree(joint);
}

void vrDistanceJointPreSolve(vrJoint * joint, vrFloat dt)
{
	vrDistanceJoint* sj = joint->jointData;

	sj->ra = vrGetLocalPoint(&joint->anchorA);
	sj->rb = vrGetLocalPoint(&joint->anchorB);

	vrVec2 ra = sj->ra;
	vrVec2 rb = sj->rb;
	vrVec2 ca = joint->A->center;
	vrVec2 cb = joint->B->center;

	vrVec2 n = vrSub(vrAdd(cb, rb), vrAdd(ca, ra));
	vrFloat len = vrLength(n);
	if (len == 0)
		n = vrVect(0, 0);
	else
		n = vrScale(n, 1.0 / len);

	sj->dir = n;
	sj->bias = (len - sj->restLength) / dt;

	vrRigidBody* A = joint->A;
	vrRigidBody* B = joint->B;

	vrFloat ran = vrCross(ra, n);
	vrFloat rbn = vrCross(rb, n);
	sj->invMass = A->bodyMaterial.invMass + B->bodyMaterial.invMass + (ran*ran) * A->bodyMaterial.invMomentInertia + (rbn*rbn) * B->bodyMaterial.invMomentInertia;

	vrVec2 impulse = vrScale(n, sj->accumImpulse);

	A->velocity = vrAdd(A->velocity, vrScale(impulse, A->bodyMaterial.invMass));
	A->angularVelocity -= A->bodyMaterial.invMomentInertia * vrCross(impulse, ra);

	B->velocity = vrSub(B->velocity, vrScale(impulse, B->bodyMaterial.invMass));
	B->angularVelocity += B->bodyMaterial.invMomentInertia * vrCross(impulse, rb);
	
	impulse = vrScale(n, sj->bias/dt);

	A->velocity = vrAdd(A->velocity, vrScale(impulse, A->bodyMaterial.invMass));
	A->angularVelocity -= A->bodyMaterial.invMomentInertia * vrCross(impulse, ra);

	B->velocity = vrSub(B->velocity, vrScale(impulse, B->bodyMaterial.invMass));
	B->angularVelocity += B->bodyMaterial.invMomentInertia * vrCross(impulse, rb);

}

void vrDistanceJointPostSolve(vrJoint * joint, vrFloat dt)
{
}

void vrDistanceJointSolvePosition(vrJoint joint)
{
}

void vrDistanceJointSolve(vrJoint * joint)
{
	vrDistanceJoint* sj = joint->jointData;

	vrVec2 n = sj->dir;
	vrVec2 ra = sj->ra;
	vrVec2 rb = sj->rb;
	vrRigidBody* A = joint->A;
	vrRigidBody* B = joint->B;
	vrVec2 rv = vrSub(vrAdd(B->velocity, vrCrossScalar(B->angularVelocity, rb)), vrAdd(A->velocity, vrCrossScalar(A->angularVelocity, ra)));

	vrFloat j = vrDot(rv, n);
	if (sj->invMass != 0) j /= sj->invMass;
	vrVec2 impulse = vrScale(n, j);

	A->velocity = vrAdd(A->velocity, vrScale(impulse, A->bodyMaterial.invMass));
	A->angularVelocity -= A->bodyMaterial.invMomentInertia * vrCross(impulse, ra);

	B->velocity = vrSub(B->velocity, vrScale(impulse, B->bodyMaterial.invMass));
	B->angularVelocity += B->bodyMaterial.invMomentInertia * vrCross(impulse, rb);
	sj->accumImpulse += j;
}
