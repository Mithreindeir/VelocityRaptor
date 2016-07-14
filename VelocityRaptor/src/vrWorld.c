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
#include "../include/velocityraptor.h"
#define GLEW_STATIC
#include <glew.h>
#include <glfw3.h>
#include "../include/vrDistanceJoint.h"

vrWorld * vrWorldAlloc()
{
	return vrAlloc(sizeof(vrWorld));
}

vrWorld * vrWorldInit(vrWorld * world)
{
	world->bodies = vrArrayInit(vrArrayAlloc(), sizeof(vrRigidBody*));
	world->accumulator = 0;
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
		vrWorldSolvePosition(world, world->timeStep);

		//Solve velocities and positions
		vrWorldSolveVelocity(world, world->timeStep);
		for (int j = 0; j < world->velIterations; j++)
		{
			for (int i = 0; i < world->joints->sizeof_active; i++)
			{
				vrJoint* joint = world->joints->data[i];
				if (joint->preSolve)
					joint->preSolve(joint, world->timeStep);
				if (joint->solveVelocity)
					joint->solveVelocity(joint);
				if (joint->postSolve)
					joint->postSolve(joint, world->timeStep);
				if (joint->solvePosition)
					joint->solvePosition(joint);
			}
		}
		
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
		
		vrVec2 pa, pb;
		pa = ((vrDistanceJoint*)joint->jointData)->ra;
		pb = ((vrDistanceJoint*)joint->jointData)->rb;
		pa = vrAdd(pa, joint->A->center);
		pb = vrAdd(pb, joint->B->center);

		glBegin(GL_LINES);
		glVertex2f(pa.x, pa.y);
		glVertex2f(pb.x, pb.y);
		glEnd();
		
	}
	world->lastTime = currentTime;
}

void vrWorldAddBody(vrWorld* world, vrRigidBody * body)
{
	for (int i = 0; i < body->shape->sizeof_active; i++)
	{
		vrShape* shape = body->shape->data[i];
		shape->updateOBB(shape->shape);
		if(shape->shapeType == VR_POLYGON) vrUpdatePolyAxes(shape->shape);
	}
	vrArrayPush(world->bodies, body);
	world->num_bodies++;
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
	for (int i = 0; i < world->bodies->sizeof_active; i++)
	{
		vrRigidBody* body = world->bodies->data[i];
		for (int j = 0; j < world->bodies->sizeof_active; j++)
		{
			vrRigidBody* body2 = world->bodies->data[j];
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
					unsigned int key = COMBINE_INTS(shape, shape2);
					vrBOOL overlap = 1;
					if(body->shape->sizeof_active > 1 || body2->shape->sizeof_active > 1)
						overlap = vrOBBOverlaps(shape->obb, shape2->obb);

					vrManifold* manifold = NULL;
					if (overlap)
					{

						collision_checks++;
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
					}
					if (manifold && manifold->contact_points > 0)
					{

						collisions++;
						vrManifoldSetBodies(manifold, body, body2);

						vrManifold* m = vrHashTableLookup(world->manifoldMap, key);

						if (m)
						{
							m->active = vrTRUE;
							vrManifoldAddContactPoints(m, *manifold);
							if (manifold) vrArrayPush(world->manifoldPool, manifold);

						}
						else
						{
							vrHashTableInsert(world->manifoldMap, manifold, key);
						}
					}
					else if (manifold)
					{
						vrArrayPush(world->manifoldPool, manifold);
					}
				}
			}
		}
	}
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
	if(world->num_manifolds == 0)
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
		glPointSize(8.0);
		glColor3f(1, 0, 0);
		glBegin(GL_POINTS);
		for (int i = 0; i < num_m; i++)
		{
			vrManifold manifold = world->manifolds[i];
			for (int i = 0; i < manifold.contact_points; i++)
			{

				glVertex2f(manifold.contacts[i].point.x, manifold.contacts[i].point.y);
			}
		}
		glEnd();
	}
	for (int j = 0; j < num_m; j++)
	{
		vrManifoldPreStep(&world->manifolds[j], dt);
		world->manifolds[j].firstTime = vrFALSE;
	}
	for (int j = 0; j < world->velIterations; j++)
	{

		for (int i = 0; i < num_m; i++)
		{
			vrManifoldSolveVelocity(&world->manifolds[i]);
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

