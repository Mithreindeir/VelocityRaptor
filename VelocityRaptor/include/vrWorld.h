#ifndef HEADER_VRWORLD
#define HEADER_VRWORLD

#include <time.h>
#include "velocityraptor.h"
#include "vrRigidBody.h"
#include "vrMemoryPool.h"
#include "vrLinkedList.h"
#include "vrAlignedArray.h"

typedef struct vrWorld
{
	vrMemoryPool* memoryPool;
	vrLinkedList* memoryList;
	vrAlignedArray* bodies;
	int num_bodies;

	/* For Stepper */
	vrFloat lastTime;
	vrFloat accumulator;
	vrFloat timeStep;

	/* Body controls */
	int velIterations;
	int posIterations;
	vrVec2 gravity;
} vrWorld;

vrWorld* vrWorldAlloc();
vrWorld* vrWorldInit(vrWorld* world);
void vrWorldDestroy(vrWorld* world);
void vrWorldStep(vrWorld* world);
void vrWorldAddBody(vrWorld* world, vrRigidBody * body);


#endif