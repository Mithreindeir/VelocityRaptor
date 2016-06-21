/*
* Copyright (c) 2006-2009 Cormac Grindall (Mithreindeir)
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
#define GLEW_STATIC
#include <glew.h>
#include <glfw3.h>

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
	world->gravity = vrVect(0, 9810);
	world->velIterations = 30;
	world->posIterations = 20;
	world->manifoldMap = vrHashTableInit(vrHashTableAlloc(), 1000);
	world->manifoldMap->deleteFunc = &vrManifoldDestroy;
	world->manifoldKeys = vrArrayInit(vrArrayAlloc(), sizeof(unsigned int));

	return world;
}

void vrWorldDestroy(vrWorld * world)
{
}

void vrWorldStep(vrWorld * world)
{
	vrFloat currentTime = clock();
	vrFloat frameTime = (currentTime - world->lastTime) / CLOCKS_PER_SEC;

	//Stops spiral of death
	if (frameTime > 0.2) frameTime = 0.2;
	world->accumulator += frameTime;

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
		
		//Integrate velocity
		for (int i = 0; i < world->bodies->sizeof_active; i++)
		{
			vrBodyIntegrateVelocity(((vrRigidBody*)world->bodies->data[i]), world->timeStep);
		}
		/* Step Finished */
		world->accumulator = world->accumulator - world->timeStep;
	}

	world->lastTime = currentTime;
}

void vrWorldAddBody(vrWorld* world, vrRigidBody * body)
{
	vrArrayPush(world->bodies, body);
}

void vrWorldQueryCollisions(vrWorld * world)
{
	//Get collisions and solve
	//O^2 Broadphase for now
	vrFloat dt = world->timeStep;
	for (int i = 0; i < world->bodies->sizeof_active; i++)
	{
		for (int j = 0; j < world->bodies->sizeof_active; j++)
		{
			if (i == j) continue;
			vrRigidBody* body = world->bodies->data[i];
			vrRigidBody* body2 = world->bodies->data[j];
			unsigned int key = COMBINE_PTR(body, body2);

			vrBOOL overlap = vrOBBOverlaps(body->shape->obb, body2->shape->obb);
			overlap = 1;
			vrHashEntry* manifold = NULL;
			if (overlap)
			{
				manifold = vrAlloc(sizeof(vrHashEntry));

				manifold->key = key;


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

				vrManifoldSetBodies(manifold->data, body, body2);
				vrHashEntry* m = vrHashTableLookup(world->manifoldMap, key);

				glPointSize(8);
				glColor3f(1, 0, 0);
				glBegin(GL_POINTS);

				for (int i = 0; i <((vrManifold*)manifold->data)->contact_points; i++)
				{
					glVertex2f(((vrManifold*)manifold->data)->contacts[i].point.x, ((vrManifold*)manifold->data)->contacts[i].point.y);

				}
				glEnd();
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

void vrWorldSolve(vrWorld * world, vrFloat dt)
{
	vrArray* manifolds = vrArrayInit(vrArrayAlloc(), sizeof(vrManifold*));
	for (int i = 0; i < world->manifoldKeys->sizeof_array; i++)
	{
		vrHashEntry* m = vrHashTableLookup(world->manifoldMap, world->manifoldKeys->data[i]);
		if (m)
		{
			vrArrayPush(manifolds, m->data);
		}
	}
	for (int i = 0; i < manifolds->sizeof_active; i++)
	{
		vrManifoldPreStep(manifolds->data[i], dt);
	}

	for (int i = 0; i < world->velIterations; i++)
	{
		for (int i = 0; i < manifolds->sizeof_active; i++)
		{
			vrManifoldSolveVelocity(manifolds->data[i]);
		}
	}
	for (int i = 0; i < manifolds->sizeof_active; i++)
	{
		vrManifoldPostStep(manifolds->data[i], dt);
	}
	for (int i = 0; i < world->posIterations; i++)
	{
		for (int i = 0; i < manifolds->sizeof_active; i++)
		{
			vrManifoldSolvePosition(manifolds->data[i], dt);
		}
	}

	vrFree(manifolds->data);
	vrFree(manifolds);
}
