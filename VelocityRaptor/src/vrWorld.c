/*
* Copyright (c) 2016 Cormac Grindall (Mithreindeir)
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
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
	world->velIterations = 10;
	world->posIterations = 5;
	world->manifoldMap = vrHashTableInit(vrHashTableAlloc(), 1000);
	world->manifoldMap->deleteFunc = &vrManifoldDestroy;
	world->manifoldKeys = vrArrayInit(vrArrayAlloc(), sizeof(unsigned int));
	world->num_bodies = 0;
	world->joints = vrArrayInit(vrArrayAlloc(), sizeof(vrJoint*));
	
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
		vrWorldSolve(world, world->timeStep);
		for (int j = 0; j < world->velIterations; j++)
		{
			for (int i = 0; i < world->joints->sizeof_active; i++)
			{
				vrJoint* joint = world->joints->data[i];
				if (joint->preSolve)
					joint->preSolve(joint, world->timeStep);
				if (joint->solveVelocity)
					joint->solveVelocity(joint);
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
	if (body->bodyMaterial.invMass == 0)
	{
		for (int i = 0; i < body->shape->sizeof_active; i++)
		{
			vrShape* shape = body->shape->data[i];
			shape->updateOBB(shape->shape);
		}
	}
	vrArrayPush(world->bodies, body);
	world->num_bodies++;
}

void vrWorldQueryCollisions(vrWorld * world)
{
	//Get collisions and solve
	//O^2 Broadphase for now
	vrFloat dt = world->timeStep;
	int collision_checks = 0;
	int collisions = 0;
	int double_checks = 0;
	for (int i = 0; i < world->bodies->sizeof_active; i++)
	{
		vrRigidBody* body = world->bodies->data[i];
		for (int k = 0; k < body->shape->sizeof_active; k++)
		{
			vrShape* shape = body->shape->data[k];
			for (int j = 0; j < world->bodies->sizeof_active; j++)
			{
				if (i == j) continue;
				//Culls duplicate pairs
				//By only colliding when i > j
				if (i < j) continue;

				vrRigidBody* body2 = world->bodies->data[j];
				for (int l = 0; l < body2->shape->sizeof_active; l++)
				{
					vrShape* shape2 = body2->shape->data[l];
					unsigned int key = COMBINE_INTS(shape, shape2);

					vrBOOL overlap = vrOBBOverlaps(shape->obb, shape2->obb);
					overlap = overlap && vrOBBOverlaps(body->obb, body2->obb);
					vrHashEntry* manifold = NULL;
					if (overlap)
					{
						manifold = vrAlloc(sizeof(vrHashEntry));

						manifold->key = key;
						collision_checks++;

						manifold->data = vrManifoldInit(vrManifoldAlloc());

						if (shape->shapeType == VR_POLYGON && shape2->shapeType == VR_POLYGON)
							vrPolyPoly(manifold->data, *((vrPolygonShape*)shape->shape), *((vrPolygonShape*)shape2->shape));
						else if (shape->shapeType == VR_POLYGON && shape2->shapeType == VR_CIRCLE)
							vrPolyCircle(manifold->data, *((vrPolygonShape*)shape->shape), *((vrCircleShape*)shape2->shape));
						else if (shape->shapeType == VR_CIRCLE && shape2->shapeType == VR_POLYGON)
							vrCirclePoly(manifold->data, *((vrCircleShape*)shape->shape), *((vrPolygonShape*)shape2->shape));
						else if (shape->shapeType == VR_CIRCLE && shape2->shapeType == VR_CIRCLE)
							vrCircleCircle(manifold->data, *((vrCircleShape*)shape->shape), *((vrCircleShape*)shape2->shape));
					}
					if (manifold && ((vrManifold*)manifold->data)->contact_points > 0)
					{

						collisions++;
						vrManifoldSetBodies(manifold->data, body, body2);

						vrHashEntry* m = vrHashTableLookup(world->manifoldMap, key);

						if (m)
						{
							vrManifoldAddContactPoints(((vrManifold*)m->data), *((vrManifold*)manifold->data));
							if (manifold->data) vrManifoldDestroy(manifold->data);
							if (manifold) vrFree(manifold);
						}
						else
						{
							vrHashTableInsert(world->manifoldMap, manifold, key);
							vrArrayPush(world->manifoldKeys, key);
						}
						glPointSize(8.0);
						glColor3f(1, 0, 0);
						vrManifold* t = vrHashTableLookup(world->manifoldMap, key)->data;
						for (int i = 0; i < t->contact_points; i++)
						{
							glColor3f(0, 0, 0);
							glPointSize(10);
							glBegin(GL_POINTS);
							glVertex2f(t->contacts[i].point.x,t->contacts[i].point.y);
							glEnd();
							glColor3f(1, 0, 0);
							glPointSize(5);
							glBegin(GL_POINTS);
							glVertex2f(t->contacts[i].point.x, t->contacts[i].point.y);
							glEnd();
						}
					}
					else
					{
						vrHashEntry* m = vrHashTableLookup(world->manifoldMap, key);
						if (m)
						{
							vrHashTableRemove(world->manifoldMap, key);

							//Linear search for key
							for (int i = 0; i < world->manifoldKeys->sizeof_array; i++)
							{
								if (world->manifoldKeys->data[i] == key)
								{
									vrArrayErase(world->manifoldKeys, i);
								}
							}
						}
						if (overlap)
						{
							if (manifold->data) vrManifoldDestroy(manifold->data);
							if (manifold) vrFree(manifold);
						}
					}
				}
			}
		}
	}
	//printf("%d collision checks and %d actual collisions and %d bodies \n ", collision_checks, collisions, world->num_bodies);
}

void vrWorldSolve(vrWorld * world, vrFloat dt)
{
	for (int i = 0; i < world->manifoldMap->buckets->sizeof_active; i++)
	{
		if (world->manifoldMap->buckets->data[i])
		{

			vrLinkedList* list = world->manifoldMap->buckets->data[i];
			vrNode* node = list->head;
			while (node)
			{
				vrHashEntry* m = ((vrHashEntry*)node->data);
				vrManifold* manifold = m->data;
				vrManifoldPreStep(manifold, dt);
				manifold->firstTime = vrFALSE;
				node = node->next;
			}
		}
	}

	for (int j = 0; j < world->velIterations; j++)
	{
		for (int i = 0; i < world->manifoldMap->buckets->sizeof_active; i++)
		{
			if (world->manifoldMap->buckets->data[i])
			{

				vrLinkedList* list = world->manifoldMap->buckets->data[i];
				vrNode* node = list->head;
				while (node)
				{
					vrHashEntry* m = ((vrHashEntry*)node->data);
					vrManifold* manifold = m->data;
					vrManifoldSolveVelocity(manifold);
					manifold->firstTime = vrFALSE;
					node = node->next;
				}
			}
		}
	}

	for (int i = 0; i < world->manifoldMap->buckets->sizeof_active; i++)
	{
		if (world->manifoldMap->buckets->data[i])
		{

			vrLinkedList* list = world->manifoldMap->buckets->data[i];
			vrNode* node = list->head;
			while (node)
			{
				vrHashEntry* m = ((vrHashEntry*)node->data);
				vrManifold* manifold = m->data;
				vrManifoldPostStep(manifold, dt);
				manifold->firstTime = vrFALSE;
				node = node->next;
			}
		}
	}

	for (int j = 0; j < world->posIterations; j++)
	{
		for (int i = 0; i < world->manifoldMap->buckets->sizeof_active; i++)
		{
			if (world->manifoldMap->buckets->data[i])
			{

				vrLinkedList* list = world->manifoldMap->buckets->data[i];
				vrNode* node = list->head;
				while (node)
				{
					vrHashEntry* m = ((vrHashEntry*)node->data);
					vrManifold* manifold = m->data;
					vrManifoldSolvePosition(manifold, dt);
					manifold->firstTime = vrFALSE;
					node = node->next;
				}
			}
		}

	}
	
}

