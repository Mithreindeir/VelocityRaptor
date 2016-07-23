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
vrCollisionPair * vrDefaultBroadphaseFunc(vrBroadphase * bp, vrArray * body_arr, int* num_pairs);

#endif