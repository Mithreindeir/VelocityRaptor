#ifndef HEADER_VRBROADPHASE
#define HEADER_VRBROADPHASE

#include "vrRigidBody.h"
#include "vrArray.h"

typedef struct vrCollisionPair
{
	int body_indexA;
	int body_indexB;
	int shape_indexA;
	int shape_indexB;
} vrCollisionPair;

typedef struct vrBroadphase vrBroadphase;
typedef vrCollisionPair*(*vrBroadphaseFunc)(vrBroadphase* bp, vrArray* body_arr, int* num_pairs);

struct vrBroadphase
{
	void* broadphase_dat;
	vrBroadphaseFunc getColliding;
};

vrBroadphase* vrBroadphaseAlloc();
vrBroadphase* vrBroadphaseInit(vrBroadphase* broadphase);
vrCollisionPair* vrDefaultBroadphaseFunc(vrBroadphase* bp, vrArray* body_arr);

#endif