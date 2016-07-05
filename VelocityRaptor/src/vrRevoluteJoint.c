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
	joint->solveVelocity = &vrRevoluteJointSolve;
	joint->jointData = vrRevoluteInit(vrRevoluteAlloc());
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

}

void vrRevoluteJointSolve(vrJoint * joint)
{
	//Explicitly solve in Ax = b form

}
