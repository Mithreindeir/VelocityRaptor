#ifndef HEADER_VRSTEP
#define HEADER_VRSTEP

#include "vrRigidBody.h"

typedef void*(*vrStepFunc)(vrRigidBody* bodies, int num_bodies);

typedef struct vrStep
{
	vrStepFunc userStep;
} vrStep;

vrStep* vrStepAlloc();
vrStep* vrStepInit(vrStep* step);
void vrStepBodies(vrStep* step, vrRigidBody* bodies, int num_bodies);

#endif