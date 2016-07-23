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

#include "../include/vrRevoluteJoint.h"
#include "../include/vrManifold.h"

vrRevoluteConstraint * vrRevoluteAlloc()
{
	return vrAlloc(sizeof(vrRevoluteConstraint));
}

vrRevoluteConstraint * vrRevoluteInit(vrRevoluteConstraint * constraint)
{
	constraint->K = vrMat(vrVect(0, 0), vrVect(0, 0));
	constraint->ra = vrVect(0, 0);
	constraint->rb = vrVect(0, 0);
	return constraint;
}

vrJoint * vrRevoluteJointInit(vrJoint * joint, vrRigidBody * A, vrRigidBody * B, vrVec2 pointA, vrVec2 pointB)
{
	joint = vrJointInit(joint);
	joint->A = A;
	joint->B = B;
	joint->anchorA = vrLocalPointInit(A, pointA);
	joint->anchorB = vrLocalPointInit(B, pointB);
	joint->preSolve = &vrRevoluteJointPreSolve;
	joint->solveVelocity = &vrRevoluteJointSolveVelocity;
	joint->postSolve = &vrRevoluteJointPostSolve;
	joint->solvePosition = &vrRevoluteJointSolvePosition;
	joint->jointData = vrRevoluteInit(vrRevoluteAlloc());
	joint->drawJoint = &vrRevoluteJointDraw;
	return joint;
}

void vrRevoluteJointDestroy(vrJoint * joint)
{
	vrFree(joint->jointData);
	vrFree(joint);
}

void vrRevoluteJointPreSolve(vrJoint * joint, vrFloat dt)
{
	vrRevoluteConstraint* rc = joint->jointData;
	vrRigidBody* A = joint->A;
	vrRigidBody* B = joint->B;
	rc->ra = vrGetLocalPoint(&joint->anchorA);
	rc->rb = vrGetLocalPoint(&joint->anchorB);

	vrVec2 ra = rc->ra;
	vrVec2 rb = rc->rb;
	vrVec2 ca = joint->A->center;
	vrVec2 cb = joint->B->center;

	//Compute K matrix
	rc->K.m.x = A->bodyMaterial.invMass + B->bodyMaterial.invMass;
	rc->K.m.x += A->bodyMaterial.invMomentInertia * ra.y * ra.y;
	rc->K.m.x += B->bodyMaterial.invMomentInertia * rb.y * rb.y;

	rc->K.m.y = -A->bodyMaterial.invMomentInertia * ra.y * ra.x;
	rc->K.m.y += -B->bodyMaterial.invMomentInertia * rb.y * rb.x;
	
	rc->K.n.y = A->bodyMaterial.invMass + B->bodyMaterial.invMass;
	rc->K.n.y += A->bodyMaterial.invMomentInertia * ra.x * ra.x;
	rc->K.n.y += B->bodyMaterial.invMomentInertia * rb.x * rb.x;

	rc->K.n.x = -A->bodyMaterial.invMomentInertia * ra.x * ra.y;
	rc->K.n.x += -B->bodyMaterial.invMomentInertia * rb.x * rb.y;

	vrVec2 impulse = rc->accum;
	A->velocity = vrSub(A->velocity, vrScale(impulse, A->bodyMaterial.invMass));
	A->angularVelocity -= A->bodyMaterial.invMomentInertia * vrCross(ra, impulse);

	B->velocity = vrAdd(B->velocity, vrScale(impulse, B->bodyMaterial.invMass));
	B->angularVelocity += B->bodyMaterial.invMomentInertia * vrCross(rb, impulse);
}

void vrRevoluteJointPostSolve(vrJoint * joint, vrFloat dt)
{
}

void vrRevoluteJointSolvePosition(vrJoint * joint)
{
	vrRevoluteConstraint* rc = joint->jointData;
	vrRigidBody* A = joint->A;
	vrRigidBody* B = joint->B;
	rc->ra = vrGetLocalPoint(&joint->anchorA);
	rc->rb = vrGetLocalPoint(&joint->anchorB);

	vrVec2 ra = rc->ra;
	vrVec2 rb = rc->rb;
	vrVec2 ca = joint->A->center;
	vrVec2 cb = joint->B->center;
	ca = vrAdd(ca, A->vel_bias);
	cb = vrAdd(cb, B->vel_bias);

	vrFloat nca = VR_COSINE(A->angv_bias);
	vrFloat nsa = VR_SINE(A->angv_bias);
	ra = vrVect(ra.x * nca - ra.y * nsa, ra.x * nsa + ra.y * nca);

	nca = VR_COSINE(B->angv_bias);
	nsa = VR_SINE(B->angv_bias);
	rb = vrVect(rb.x * nca - rb.y * nsa, rb.x * nsa + rb.y * nca);

	rc->K.m.x = A->bodyMaterial.invMass + B->bodyMaterial.invMass;
	rc->K.m.x += A->bodyMaterial.invMomentInertia * ra.y * ra.y;
	rc->K.m.x += B->bodyMaterial.invMomentInertia * rb.y * rb.y;

	rc->K.m.y = -A->bodyMaterial.invMomentInertia * ra.y * ra.x;
	rc->K.m.y += -B->bodyMaterial.invMomentInertia * rb.y * rb.x;

	rc->K.n.y = A->bodyMaterial.invMass + B->bodyMaterial.invMass;
	rc->K.n.y += A->bodyMaterial.invMomentInertia * ra.x * ra.x;
	rc->K.n.y += B->bodyMaterial.invMomentInertia * rb.x * rb.x;

	rc->K.n.x = -A->bodyMaterial.invMomentInertia * ra.x * ra.y;
	rc->K.n.x += -B->bodyMaterial.invMomentInertia * rb.x * rb.y;

	vrVec2 C = vrSub(vrAdd(cb, rb), vrAdd(ca, ra));

	vrVec2 x = vrMat2Mult(vrMat2Invert(rc->K), C);

	vrVec2 impulse = vrVect(-x.x, -x.y);

	A->vel_bias = vrSub(A->vel_bias, vrScale(impulse, A->bodyMaterial.invMass));
	A->angv_bias -= A->bodyMaterial.invMomentInertia * vrCross(ra, impulse);

	B->vel_bias = vrAdd(B->vel_bias, vrScale(impulse, B->bodyMaterial.invMass));
	B->angv_bias += B->bodyMaterial.invMomentInertia * vrCross(rb, impulse);
}

void vrRevoluteJointDraw(vrJoint * joint)
{
	vrRevoluteConstraint* rc = joint->jointData;
	rc->ra = vrGetLocalPoint(&joint->anchorA);
	rc->rb = vrGetLocalPoint(&joint->anchorB);

	vrVec2 ra = rc->ra;
	vrVec2 rb = rc->rb;
	vrVec2 ca = joint->A->center;
	vrVec2 cb = joint->B->center;
	ca = vrAdd(ca, ra);
	cb = vrAdd(cb, rb);
	vrVec2 p = vrScale(vrAdd(ca, cb), 1.0 / 2.0);
}

void vrRevoluteJointSolveVelocity(vrJoint * joint)
{
	vrRevoluteConstraint* rc = joint->jointData;
	vrRigidBody* A = joint->A;
	vrRigidBody* B = joint->B;

	vrVec2 ra = rc->ra;
	vrVec2 rb = rc->rb;
	vrVec2 rv = vrSub(vrAdd(B->velocity, vrCrossScalar(B->angularVelocity, rb)), vrAdd(A->velocity, vrCrossScalar(A->angularVelocity, ra)));
	//Explicitly solve in Ax = b form
	vrBlockSolverData bk;
	bk.A = rc->K;
	bk.contactVel = vrVect(-rv.x, -rv.y);
	vrVec2 x = vrManifoldConjugateGradient(bk, 1e-4);
	vrVec2 impulse = x;
	rc->accum = vrAdd(rc->accum, x);

	A->velocity = vrSub(A->velocity, vrScale(impulse, A->bodyMaterial.invMass));
	A->angularVelocity -= A->bodyMaterial.invMomentInertia * vrCross(ra, impulse);

	B->velocity = vrAdd(B->velocity, vrScale(impulse, B->bodyMaterial.invMass));
	B->angularVelocity += B->bodyMaterial.invMomentInertia * vrCross(rb, impulse);
}
