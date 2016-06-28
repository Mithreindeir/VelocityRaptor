#include "..\include\vrSpringJoint.h"

vrJoint * vrSpringJointInit(vrJoint * joint, vrRigidBody* A, vrRigidBody* B, vrVec2 pointA, vrVec2 pointB)
{
	joint = vrJointInit(joint);
	joint->A = A;
	joint->B = B;
	joint->anchorA = vrLocalPointInit(A, pointA);
	joint->anchorB = vrLocalPointInit(B, pointB);

	return joint;
}

void vrSpringJointSolve(vrJoint * joint)
{
	vrSpringJoint* sj = joint->jointData;
	vrVec2 ra = vrGetLocalPoint(&joint->anchorA);
	vrVec2 rb = vrGetLocalPoint(&joint->anchorB);
	vrVec2 ca = joint->A->shape->getCenter(joint->A->shape->shape);
	vrVec2 cb = joint->B->shape->getCenter(joint->B->shape->shape);


	vrVec2 n = vrSub(vrAdd(ca, ra), vrAdd(cb, rb));
	vrFloat len = vrLength(n);
	if (len == 0.0)
		n = vrVect(0, 0);
	else n = vrScale(n, 1.0 / len);

	vrFloat ran = vrCross(ra, n);
	vrFloat rbn = vrCross(rb, n);

	vrFloat inv_mass = joint->A->bodyMaterial.invMass + joint->B->bodyMaterial.invMass + (ran*ran) * joint->A->bodyMaterial.invMomentInertia + (rbn*rbn) * joint->B->bodyMaterial.invMomentInertia;
	sj->invMass = inv_mass;
	vrVec2 accum = sj->accumImpulse;


}
