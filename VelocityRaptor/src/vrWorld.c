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
	world->timeStep = (1.0f / 180.0f);
	world->gravity = vrVect(0, 981);
	world->velIterations = 15;
	world->posIterations = 10;
	world->manifoldMap = vrHashTableInit(vrHashTableAlloc(), 1000);
	world->manifoldMap->deleteFunc = &vrManifoldDestroy;
	world->manifoldKeys = vrArrayInit(vrArrayAlloc(), sizeof(unsigned int));
	world->num_bodies = 0;

	return world;
}

void vrWorldDestroy(vrWorld * world)
{
	for (int i = 0; i < world->bodies->sizeof_active; i++)
	{
		vrBodyDestroy(world->bodies->data[i]);
	}
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
		if (!b && world->num_bodies > 5)
		{
			vrRigidBody* A = world->bodies->data[4];
			vrRigidBody* B = world->bodies->data[5];
			
						vrVec2 ra = ((vrVertex*)((vrNode*)(((vrPolygonShape*)A->shape->shape)->vertices->head))->data)->vertex;
			vrVec2 rb = ((vrVertex*)((vrNode*)(((vrPolygonShape*)B->shape->shape)->vertices->head))->data)->vertex;

			world->joint = vrDistanceJointInit(vrJointAlloc(), A, B, ra, rb);
			
			//world->joint = vrDistanceJointInit(vrJointAlloc(), A, B, A->shape->getCenter(A->shape->shape), B->shape->getCenter(B->shape->shape));
			b = vrTRUE;
		}
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
		if (b)
		{
			if (world->joint->preSolve)
				world->joint->preSolve(world->joint, world->timeStep);
			if (world->joint->solveVelocity)
				world->joint->solveVelocity(world->joint);

		}
		//Integrate velocity
		for (int i = 0; i < world->bodies->sizeof_active; i++)
		{
			vrBodyIntegrateVelocity(((vrRigidBody*)world->bodies->data[i]), world->timeStep);
		}
		/* Step Finished */
		world->accumulator = world->accumulator - world->timeStep;
	}
	if (b)
	{
		vrVec2 pa, pb;
		pa = ((vrDistanceJoint*)world->joint->jointData)->ra;
		pb = ((vrDistanceJoint*)world->joint->jointData)->rb;
		pa = vrAdd(pa, world->joint->A->shape->getCenter(world->joint->A->shape->shape));
		pb = vrAdd(pb, world->joint->B->shape->getCenter(world->joint->B->shape->shape));

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
		body->shape->updateOBB(body->shape->shape);
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
		for (int j = 0; j < world->bodies->sizeof_active; j++)
		{
			if (i == j) continue;
			//Culls duplicate pairs
			//By only colliding when i > j
			if (i < j) continue; 
			vrRigidBody* body = world->bodies->data[i];
			vrRigidBody* body2 = world->bodies->data[j];

			unsigned int key = COMBINE_INTS(body, body2);

			vrBOOL overlap = vrOBBOverlaps(body->shape->obb, body2->shape->obb);

			vrHashEntry* manifold = NULL;
			if (overlap)
			{
				manifold = vrAlloc(sizeof(vrHashEntry));

				manifold->key = key;
				collision_checks++;

				manifold->data = vrManifoldInit(vrManifoldAlloc());

				if (body->shape->shapeType == VR_POLYGON && body2->shape->shapeType == VR_POLYGON)
					vrPolyPoly(manifold->data, *((vrPolygonShape*)body->shape->shape), *((vrPolygonShape*)body2->shape->shape));
				else if (body->shape->shapeType == VR_POLYGON && body2->shape->shapeType == VR_CIRCLE)
					vrPolyCircle(manifold->data, *((vrPolygonShape*)body->shape->shape), *((vrCircleShape*)body2->shape->shape));
				else if (body->shape->shapeType == VR_CIRCLE && body2->shape->shapeType == VR_POLYGON)
					vrCirclePoly(manifold->data, *((vrCircleShape*)body->shape->shape), *((vrPolygonShape*)body2->shape->shape));
				else if (body->shape->shapeType == VR_CIRCLE && body2->shape->shapeType == VR_CIRCLE)
					vrCircleCircle(manifold->data, *((vrCircleShape*)body->shape->shape), *((vrCircleShape*)body2->shape->shape));
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
				glBegin(GL_POINTS);
				vrManifold* t = vrHashTableLookup(world->manifoldMap, key)->data;
				for (int i = 0; i < t->contact_points; i++)
				{
					glVertex2f(t->contacts[i].point.x, t->contacts[i].point.y);
				}
				glEnd();
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
	//printf("%d collision checks and %d actual collisions and %d bodies \n ", collision_checks, collisions, world->num_bodies);
}

void vrWorldSolve(vrWorld * world, vrFloat dt)
{
	for (int i = 0; i < world->manifoldKeys->sizeof_array; i++)
	{
		vrHashEntry* m = vrHashTableLookup(world->manifoldMap, world->manifoldKeys->data[i]);
		if (!m) continue;
		vrManifold* manifold = m->data;
		vrManifoldPreStep(manifold, dt);
		manifold->firstTime = vrFALSE;

	}
	for (int i = 0; i < world->velIterations; i++)
	{
		for (int i = 0; i < world->manifoldKeys->sizeof_array; i++)
		{
			vrHashEntry* m = vrHashTableLookup(world->manifoldMap, world->manifoldKeys->data[i]);
			if (!m) continue;
			vrManifold* manifold = m->data;
			vrManifoldSolveVelocity(manifold);
			manifold->firstTime = vrFALSE;

		}
	}
	
	for (int i = 0; i < world->manifoldKeys->sizeof_array; i++)
	{
		vrHashEntry* m = vrHashTableLookup(world->manifoldMap, world->manifoldKeys->data[i]);
		if (!m) continue;
		vrManifold* manifold = m->data;
		vrManifoldPostStep(manifold, dt);
		manifold->firstTime = vrFALSE;

	}
	for (int i = 0; i < world->posIterations; i++)
	{
		for (int i = 0; i < world->manifoldKeys->sizeof_array; i++)
		{
			vrHashEntry* m = vrHashTableLookup(world->manifoldMap, world->manifoldKeys->data[i]);
			if (!m) continue;
			vrManifold* manifold = m->data;
			vrManifoldSolvePosition(manifold, dt);
			manifold->firstTime = vrFALSE;

		}
	}
	
}

