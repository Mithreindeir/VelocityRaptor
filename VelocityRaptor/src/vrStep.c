#include "../include/vrStep.h"

vrStep * vrStepAlloc()
{
	return vrAlloc(sizeof(vrStep));
}

vrStep * vrStepInit(vrStep * step)
{
	step->userStep = NULL;
	return step;
}

void vrStepBodies(vrStep * step, vrRigidBody * bodies, int num_bodies)
{
	if (step->userStep) step->userStep(bodies, num_bodies);

	for (int i = 0; i < num_bodies; i++)
	{ 
		
	}
}
