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
	world->gravity = vrVect(0, 981);
	world->velIterations = 8;
	world->posIterations = 2;
	world->manifoldMap = vrHashTableInit(vrHashTableAlloc(), 200);
	world->manifoldMap->deleteFunc = &vrManifoldDestroy;

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
			//body->force = vrAdd(body->force, world->gravity);
			vrBodyIntegrateForces(body, world->timeStep);
		}

		//Get collisions and solve
		//O^2 Broadphase for now

		//Integrate velocity
		vrFloat dt = world->timeStep;
		for (int i = 0; i < world->bodies->sizeof_active; i++)
		{
			for (int j = 0; j < world->bodies->sizeof_active; j++)
			{

				if (i == j) continue;
				vrRigidBody* body = world->bodies->data[i];
				vrRigidBody* body2 = world->bodies->data[j];
				unsigned int key = COMBINE_PTR((unsigned int)body, (unsigned int)body2);

				vrHashEntry* manifold = vrAlloc(sizeof(vrHashEntry));

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

				if (((vrManifold*)manifold->data)->contact_points > 0)
				{

					vrManifoldSetBodies(manifold->data, body, body2);
					vrHashTableInsert(world->manifoldMap, manifold, key);



					glPointSize(8);
					glColor3f(1, 0, 0);
					glBegin(GL_POINTS);

					for (int i = 0; i <((vrManifold*)manifold->data)->contact_points; i++)
					{
						glVertex2f(((vrManifold*)manifold->data)->contacts[i].point.x, ((vrManifold*)manifold->data)->contacts[i].point.y);

					}
					glEnd();
				}
				else
				{
					vrHashEntry* m = vrHashTableLookup(world->manifoldMap, key);
					if (m)
					{
						if(m->data) vrManifoldDestroy(m->data);
						vrHashTableRemove(world->manifoldMap, key);

					}
					if (manifold->data) vrManifoldDestroy(manifold->data);
					if (manifold) vrFree(manifold);
				}
			}
		}
		
		for (int i = 0; i < world->bodies->sizeof_active; i++)
		{
			for (int j = 0; j < world->bodies->sizeof_active; j++)
			{
				vrRigidBody* body = world->bodies->data[i];
				vrRigidBody* body2 = world->bodies->data[j];
				unsigned int key = COMBINE_PTR((unsigned int)body, (unsigned int)body2);

				vrHashEntry* m = vrHashTableLookup(world->manifoldMap, key);
				if (m)
				{
					vrManifold* manifold = m->data;

					vrManifoldPreStep(manifold, frameTime);
					for (int i = 0; i < 80; i++)
						vrManifoldSolveVelocity(manifold);
					vrManifoldPostStep(manifold, frameTime);

					for (int i = 0; i < 20; i++)
						vrManifoldSolvePosition(manifold, frameTime);

					glPointSize(8);
					glColor3f(1, 0, 0);
					glBegin(GL_POINTS);

					for (int i = 0; i < manifold->contact_points; i++)
					{
						glVertex2f(manifold->contacts[i].point.x, manifold->contacts[i].point.y);

					}
					glEnd();

				}
			}
		}
		
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
