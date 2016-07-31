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

#include "../include/vrWorld.h"
#include "../include/vrCollision.h"
#include "../include/vr.h"
#include "../include/vrDistanceJoint.h"

vrWorld * vrWorldAlloc()
{
	return vrAlloc(sizeof(vrWorld));
}

vrWorld * vrWorldInit(vrWorld * world)
{
	world->bodies = vrArrayInit(vrArrayAlloc(), sizeof(vrRigidBody*));
	world->accumulator = 0;
	world->num_manifolds = 0;
	world->lastTime = 0;
	world->timeStep = (1.0f / 60.0f);
	world->gravity = vrVect(0, 981);
	world->velIterations = 20;
	world->posIterations = 15;
	world->manifoldMap = vrHashTableInit(vrHashTableAlloc(), 1000);
	world->manifoldMap->deleteFunc = &vrManifoldDestroy;
	world->num_bodies = 0;
	world->joints = vrArrayInit(vrArrayAlloc(), sizeof(vrJoint*));
	world->manifoldPool = vrArrayInit(vrArrayAlloc(), sizeof(vrManifold*));
	world->broadphase = vrBroadphaseInit(vrBroadphaseAlloc());
	for (int i = 0; i < 500; i++)
	{
		vrArrayPush(world->manifoldPool, vrAlloc(sizeof(vrManifold)));
	}
	return world;
}

void vrWorldDestroy(vrWorld * world)
{
	for (int i = 0; i < world->bodies->sizeof_active; i++)
	{
		vrBodyDestroy(world->bodies->data[i]);
	}
	for (int i = 0; i < world->joints->sizeof_active; i++)
	{

	}
	vrFree(world);
}
vrBOOL b = vrFALSE;
void vrWorldStep(vrWorld * world)
{
	vrFloat currentTime = clock();
	vrFloat frameTime = (currentTime - world->lastTime) / CLOCKS_PER_SEC;

	//Stops spiral of death
	if (frameTime > 0.2) frameTime = 0.2;
	world->accumulator += frameTime;
	int avg_checks = 0;
	while (world->accumulator > world->timeStep)
	{

		/* Step Started */
		//Integrate forces
		for (int i = 0; i < world->bodies->sizeof_active; i++)
		{
			vrRigidBody* body = ((vrRigidBody*)world->bodies->data[i]);

			//Apply gravity
			body->force = vrAdd(body->force, world->gravity);
			vrBodyIntegrateForces(body, world->timeStep);
		}

		//Get collisions 
		vrWorldQueryCollisions(world);

		//Solve velocities and positions
		vrWorldSolveVelocity(world, world->timeStep);
		vrWorldSolvePosition(world, world->timeStep);


		//Integrate velocity
		for (int i = 0; i < world->bodies->sizeof_active; i++)
		{
			vrBodyIntegrateVelocity(((vrRigidBody*)world->bodies->data[i]), world->timeStep);
		}
		/* Step Finished */
		world->accumulator = world->accumulator - world->timeStep;
	}
	
	for (int i = 0; i < world->joints->sizeof_active; i++)
	{
		vrJoint* joint = world->joints->data[i];
		if (joint->drawJoint)
			joint->drawJoint(joint);
	}
	
	world->lastTime = currentTime;
}

void vrWorldAddBody(vrWorld* world, vrRigidBody * body)
{
	for (int i = 0; i < body->shape->sizeof_active; i++)
	{
		vrShape* shape = body->shape->data[i];
		shape->updateOBB(shape->shape);
		if (shape->shapeType == VR_POLYGON) vrUpdatePolyAxes(shape->shape);
	}
	vrArrayPush(world->bodies, body);
	world->num_bodies++;
}

void vrWorldRemoveBody(vrWorld * world, vrRigidBody * body)
{
	for (int i = 0; i < world->bodies->sizeof_active; i++)
	{
		vrRigidBody* b = world->bodies->data[i];
		if (b == body)
		{
			vrBodyDestroy(body);
			vrArrayErase(world->bodies, i);
		}
	}
}

void vrWorldQueryCollisions(vrWorld * world)
{
	for (int i = 0; i < world->manifoldMap->buckets->sizeof_active; i++)
	{
		if (world->manifoldMap->buckets->data[i])
		{

			vrHashEntry* head = world->manifoldMap->buckets->data[i];
			while (head)
			{
				if (head && head->data)
				{
					vrManifold* manifold = head->data;
					manifold->active = vrFALSE;
				}
				head = head->next;
			}
		}
	}


	//Get collisions and solve
	//O^2 Broadphase for now
	vrFloat dt = world->timeStep;
	int collision_checks = 0;
	int collisions = 0;
	int double_checks = 0;

	int num_colliding = 0;
	//Broadphase
	vrCollisionPair* cp = world->broadphase->getColliding(world->broadphase, world->bodies, &num_colliding);
	//Narrowphase
	for (int i = 0; i < num_colliding; i++)
	{
		vrRigidBody* body = world->bodies->data[cp[i].body_indexA];
		vrRigidBody* body2 = world->bodies->data[cp[i].body_indexB];

		vrShape* shape = body->shape->data[cp[i].shape_indexA];
		vrShape* shape2 = body2->shape->data[cp[i].shape_indexB];
		unsigned int key = COMBINE_INTS(shape, shape2);

		vrManifold* manifold = NULL;

		if (world->manifoldPool->sizeof_active > 0)
		{
			manifold = vrManifoldInit(world->manifoldPool->data[world->manifoldPool->sizeof_active - 1]);
			vrArrayPop(world->manifoldPool);
		}
		else
		{
			manifold = vrManifoldInit(vrManifoldAlloc());
		}
		manifold->active = vrTRUE;
		if (shape->shapeType == VR_POLYGON && shape2->shapeType == VR_POLYGON)
			vrPolyPoly(manifold, *((vrPolygonShape*)shape->shape), *((vrPolygonShape*)shape2->shape));
		else if (shape->shapeType == VR_POLYGON && shape2->shapeType == VR_CIRCLE)
			vrPolyCircle(manifold, *((vrPolygonShape*)shape->shape), *((vrCircleShape*)shape2->shape));
		else if (shape->shapeType == VR_CIRCLE && shape2->shapeType == VR_POLYGON)
			vrCirclePoly(manifold, *((vrCircleShape*)shape->shape), *((vrPolygonShape*)shape2->shape));
		else if (shape->shapeType == VR_CIRCLE && shape2->shapeType == VR_CIRCLE)
			vrCircleCircle(manifold, *((vrCircleShape*)shape->shape), *((vrCircleShape*)shape2->shape));
		
		if (manifold && manifold->contact_points > 0)
		{

			collisions++;
			vrManifoldSetBodies(manifold, body, body2);

			vrManifold* m = vrHashTableLookup(world->manifoldMap, key);

			if (m)
			{
				m->active = vrTRUE;
				vrManifoldAddContactPoints(m, *manifold);
				m->key = key;
				if (manifold) vrArrayPush(world->manifoldPool, manifold);
				vrBOOL contains = vrFALSE;
				for (int i = 0; i < body->manifolds->sizeof_active; i++)
				{
					vrManifold* m = body->manifolds->data[i];
					if (m->key == key)
						contains = vrTRUE;
				}
				if (!contains)
					vrArrayPush(body->manifolds, m);

				contains = vrFALSE;
				for (int i = 0; i < body2->manifolds->sizeof_active; i++)
				{
					vrManifold* m = body2->manifolds->data[i];
					if (m->key == key)
						contains = vrTRUE;
				}
				if (!contains)
					vrArrayPush(body2->manifolds, m);

			}
			else
			{
				manifold->key = key;
				vrHashTableInsert(world->manifoldMap, manifold, key);
				vrBOOL contains = vrFALSE;
				for (int i = 0; i < body->manifolds->sizeof_active; i++)
				{
					vrManifold* m = body->manifolds->data[i];
					if (m->key == key)
						contains = vrTRUE;
				}
				if (!contains)
					vrArrayPush(body->manifolds, manifold);

				contains = vrFALSE;
				for (int i = 0; i < body2->manifolds->sizeof_active; i++)
				{
					vrManifold* m = body2->manifolds->data[i];
					if (m->key == key)
						contains = vrTRUE;
				}
				if (!contains)
					vrArrayPush(body2->manifolds, manifold);

			}
		}
		else if (manifold)
		{
			vrArrayPush(world->manifoldPool, manifold);
		}

	}
	if(cp) vrFree(cp);
	for (int i = 0; i < world->manifoldMap->buckets->sizeof_active; i++)
	{
		if (world->manifoldMap->buckets->data[i])
		{

			vrHashEntry* head = world->manifoldMap->buckets->data[i];
			vrHashEntry* prev = NULL;
			while (head)
			{
				if (head && head->data)
				{
					vrManifold* manifold = head->data;
					if (!manifold->active)
					{
						vrBOOL contains = vrFALSE;
						for (int i = 0; i < manifold->A->manifolds->sizeof_active; i++)
						{
							vrManifold* m = manifold->A->manifolds->data[i];
							if (m == manifold)
								vrArrayErase(manifold->A->manifolds, i);
						}
						for (int i = 0; i < manifold->B->manifolds->sizeof_active; i++)
						{
							vrManifold* m = manifold->B->manifolds->data[i];
							if (m == manifold)
								vrArrayErase(manifold->B->manifolds, i);
						}
						vrArrayPush(world->manifoldPool, manifold);
						vrHashTableRemove(world->manifoldMap, head->key);
					}
				}
				head = head->next;
			}
		}
	}

}

void vrWorldSolvePosition(vrWorld * world, vrFloat dt)
{
	for (int i = 0; i < world->joints->sizeof_active; i++)
	{
		vrJoint* joint = world->joints->data[i];
		if (joint->postSolve)
			joint->postSolve(joint);
	}
	for (int i = 0; i < world->manifoldMap->buckets->sizeof_active; i++)
	{
		if (world->manifoldMap->buckets->data[i])
		{

			vrHashEntry* m = world->manifoldMap->buckets->data[i];
			vrHashEntry* prev = NULL;
			while (m)
			{
				vrManifold* manifold = m->data;
				vrManifoldPostStep(manifold, dt);
				manifold->firstTime = vrFALSE;
				m = m->next;
			}
		}
	}


	for (int j = 0; j < world->posIterations; j++)
	{
		for (int i = 0; i < world->manifoldMap->buckets->sizeof_active; i++)
		{
			if (world->manifoldMap->buckets->data[i])
			{

				vrHashEntry* m = world->manifoldMap->buckets->data[i];
				vrHashEntry* prev = NULL;
				while (m)
				{
					vrManifold* manifold = m->data;
					vrManifoldSolvePosition(manifold, dt);
					manifold->firstTime = vrFALSE;
					m = m->next;
				}
			}
		}
		for (int j = 0; j < world->posIterations; j++)
		{
			for (int i = 0; i < world->joints->sizeof_active; i++)
			{
				vrJoint* joint = world->joints->data[i];
				if (joint->solvePosition)
					joint->solvePosition(joint);
			}
		}
	}
}

void vrWorldSolveVelocity(vrWorld * world, vrFloat dt)
{

	int num_m = 0;
	for (int i = 0; i < world->manifoldMap->buckets->sizeof_active; i++)
	{
		if (world->manifoldMap->buckets->data[i])
		{

			vrHashEntry* m = world->manifoldMap->buckets->data[i];
			vrHashEntry* prev = NULL;
			while (m)
			{
				num_m++;
				m = m->next;
			}
		}
	}
	if (world->num_manifolds == 0)
		world->manifolds = vrAlloc(sizeof(vrManifold) * num_m);
	else
	{
		world->manifolds = vrRealloc(world->manifolds, sizeof(vrManifold) * num_m);
	}
	world->num_manifolds = num_m;
	int iter = 0;
	for (int i = 0; i < world->manifoldMap->buckets->sizeof_active; i++)
	{
		if (world->manifoldMap->buckets->data[i])
		{
			vrHashEntry* m = world->manifoldMap->buckets->data[i];
			vrHashEntry* prev = NULL;
			while (m)
			{
				world->manifolds[iter] = *((vrManifold*)m->data);
				iter++;
				m = m->next;
			}
		}
	}
	if (DEBUG_DRAW_CONTACTS)
	{
	}
	for (int j = 0; j < num_m; j++)
	{
		vrManifoldPreStep(&world->manifolds[j], dt);
		world->manifolds[j].firstTime = vrFALSE;
	}
	for (int i = 0; i < world->joints->sizeof_active; i++)
	{
		vrJoint* joint = world->joints->data[i];
		if (joint->preSolve)
			joint->preSolve(joint, world->timeStep);
	}

	for (int j = 0; j < world->velIterations; j++)
	{

		for (int i = 0; i < num_m; i++)
		{
			vrManifoldSolveVelocity(&world->manifolds[i]);
		}
		for (int i = 0; i < world->joints->sizeof_active; i++)
		{
			vrJoint* joint = world->joints->data[i];

			if (joint->solveVelocity)
				joint->solveVelocity(joint);

		}
	}
	iter = 0;
	for (int i = 0; i < world->manifoldMap->buckets->sizeof_active; i++)
	{
		if (world->manifoldMap->buckets->data[i])
		{
			vrHashEntry* m = world->manifoldMap->buckets->data[i];
			vrHashEntry* prev = NULL;
			while (m)
			{
				*((vrManifold*)m->data) = world->manifolds[iter];
				iter++;
				m = m->next;
			}
		}
	}
}

