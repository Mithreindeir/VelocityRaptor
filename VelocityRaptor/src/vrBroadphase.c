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

#include "../include/vrBroadphase.h"

vrBroadphase * vrBroadphaseAlloc()
{
	return vrAlloc(sizeof(vrBroadphase));
}

vrBroadphase * vrBroadphaseInit(vrBroadphase * broadphase)
{
	broadphase->broadphase_dat = NULL;
	broadphase->getColliding = &vrDefaultBroadphaseFunc;
	return broadphase;
}

vrCollisionPair * vrDefaultBroadphaseFunc(vrBroadphase * bp, vrArray * body_arr, int* num_pairs)
{
	vrCollisionPair* collidingPairs = NULL;
	int pairs = 0;

	for (int i = 0; i < body_arr->sizeof_active; i++)
	{
		vrRigidBody* body = body_arr->data[i];
		for (int j = 0; j < body_arr->sizeof_active; j++)
		{
			vrRigidBody* body2 = body_arr->data[j];
			if (body->bodyMaterial.invMass == 0 && body2->bodyMaterial.invMass == 0) continue;
			if (i == j) continue;
			//Culls duplicate pairs
			//By only colliding when i > j
			if (i < j) continue;

			if (!vrOBBOverlaps(body->obb, body2->obb)) continue;

			
			for (int k = 0; k < body->shape->sizeof_active; k++)
			{
				vrShape* shape = body->shape->data[k];

				for (int l = 0; l < body2->shape->sizeof_active; l++)
				{
					vrShape* shape2 = body2->shape->data[l];
					vrBOOL overlap = 1;
					if (body->shape->sizeof_active > 1 || body2->shape->sizeof_active > 1)
						overlap = vrOBBOverlaps(shape->obb, shape2->obb);

					if (overlap)
					{
						vrCollisionPair cp;
						cp.body_indexA = i;
						cp.body_indexB = j;
						cp.shape_indexA = k;
						cp.shape_indexB = l;
						if (pairs == 0)
						{
							collidingPairs = vrAlloc(sizeof(vrCollisionPair));
							collidingPairs[0] = cp;
							pairs++;
						}
						else
						{
							pairs += 1;
							collidingPairs = vrRealloc(collidingPairs, sizeof(vrCollisionPair) * pairs);
							collidingPairs[pairs - 1] = cp;
						}
					}
				}
			}
		}
	}
	*num_pairs = pairs;
	return collidingPairs;
}
