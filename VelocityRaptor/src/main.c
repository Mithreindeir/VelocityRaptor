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
#include "../include/vrRigidBody.h"
#include "../include/velocityraptor.h"
#include "../include/vrCollision.h"
#include "../include/vrMemoryPool.h"
#include "../include/vrAlignedArray.h"
#include "../include/vrWorld.h"
#include "../include/vrHashMap.h"
#include <stdio.h>
#include <conio.h>
#define GLEW_STATIC
#include <glew.h>
#include <glfw3.h>
void vrDebugDrawPolygon(vrPolygonShape* shape);
void vrDebugDrawCircle(vrCircleShape* circle);
void vrManifoldDestroy(vrManifold* manifold);
vrMemoryPool* memPool;
vrBOOL keys[1024];
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode);
vrAlignedArray* arr;
vrWorld* world;

int
main(void)
{
	GLFWwindow* window;
	glfwInit();

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);
	window = glfwCreateWindow(800, 800, "Velocity Raptor: v0.0.3 C Revision", NULL, NULL);
	//glViewport(0, 0, 80, 80);
	glfwMakeContextCurrent(window);


	glewExperimental = GL_TRUE;
	glewInit();

	vrFloat dt = 1 / 60.0f;
	vrFloat lastTime = 0.0;
	vrFloat currentTime = 0.0;
	vrFloat accumulator = 0.0;
	glfwSwapInterval(1);
	glfwSetKeyCallback(window, key_callback);
	vrFloat avgFT = 0;
	int frames = 0;
	world = vrWorldInit(vrWorldAlloc());


	vrRigidBody* body = vrBodyInit(vrBodyAlloc());
	body->shape = vrShapeInit(vrShapeAlloc());
	body->shape = vrShapePolyInit(body->shape);
	body->shape->shape = vrPolyBoxInit(body->shape->shape, 400, 50, 50, 5);
	body->bodyMaterial.restitution = 0.0;
	//body->shape = vrShapeCircleInit(body->shape);
	//((vrCircleShape*)body->shape->shape)->center = vrVect(400, 400);
	//((vrCircleShape*)body->shape->shape)->radius = 50;

	vrRigidBody* body2 = vrBodyInit(vrBodyAlloc());

	body2->shape = vrShapeInit(vrShapeAlloc());
	//body2->shape = vrShapeCircleInit(body2->shape);
	body2->bodyMaterial.invMass = 0;

	body2->bodyMaterial.invMomentInertia = 0;
//	((vrCircleShape*)body2->shape->shape)->center = vrVect(400, 600);
//	((vrCircleShape*)body2->shape->shape)->radius = 50;

	body2->shape = vrShapePolyInit(body2->shape);

	body2->shape->shape = vrPolyBoxInit(body2->shape->shape, 400, 650, 200, 50);
	body2->bodyMaterial.invMass = 0;
	body2->bodyMaterial.invMomentInertia = 0;
	vrManifold* m = vrManifoldInit(vrManifoldAlloc());

	//vrPolyPoly(m, *((vrPolygonShape*)body->shape->shape), *((vrPolygonShape*)body2->shape->shape));
	vrWorldAddBody(world, body);
	vrWorldAddBody(world, body2);


	vrLinkedList* manifolds = vrLinkedListInit(vrLinkedListAlloc());
	manifolds->deleteFunc = vrManifoldDestroy;

	manifolds->head = NULL;


	vrHashTable* map = vrHashTableInit(vrHashTableAlloc());
	vrHashEntry* entry = vrAlloc(sizeof(vrHashEntry));
	entry->data = 15;
	entry->key = "BANANA";
	entry->next = NULL;
	vrHashTableInsert(map, entry, entry->key);

	vrHashEntry* test = vrHashTableLookup(map, "BANANA");
	printf("%d\n", (int)test->data);
	while (!glfwWindowShouldClose(window))
	{

		lastTime = currentTime;
		currentTime = glfwGetTime();
		vrFloat deltaTime = currentTime - lastTime;
		vrFloat frameTime = currentTime - lastTime;
		lastTime = currentTime;
		accumulator += frameTime;
		if (accumulator > 0.2) accumulator = 0.2;

		int display_w, display_h;
		glfwGetFramebufferSize(window, &display_w, &display_h);
		glViewport(0, 0, display_w, display_h);
		glClearColor(1, 1, 1, 1);
		glClear(GL_COLOR_BUFFER_BIT);
		//vrWorldStep(world);

		while (accumulator >= dt)
		{
			body->force.y += 980;
			body->torque += .1;
			//body->force.x += 10;

			vrBodyIntegrateForces(body, frameTime);
			vrBodyIntegrateForces(body2, frameTime);

			vrManifold* man = vrManifoldInit(vrManifoldAlloc());

			if(body->shape->shapeType == VR_POLYGON && body2->shape->shapeType == VR_POLYGON)
				vrPolyPoly(man, *((vrPolygonShape*)body->shape->shape), *((vrPolygonShape*)body2->shape->shape));
			else if (body->shape->shapeType == VR_POLYGON && body2->shape->shapeType == VR_CIRCLE)
				vrPolyCircle(man, *((vrPolygonShape*)body->shape->shape), *((vrCircleShape*)body2->shape->shape));
			else if (body->shape->shapeType == VR_CIRCLE && body2->shape->shapeType == VR_POLYGON)
				vrCirclePoly(man, *((vrCircleShape*)body->shape->shape), *((vrPolygonShape*)body2->shape->shape));
			else if (body->shape->shapeType == VR_CIRCLE && body2->shape->shapeType == VR_CIRCLE)
				vrCircleCircle(man, *((vrCircleShape*)body->shape->shape), *((vrCircleShape*)body2->shape->shape));


			if (man->contact_points > 0)
			{

				vrManifoldSetBodies(man, body, body2);

				vrNode* manifold = vrAlloc(sizeof(vrNode));

				manifold->data = vrAlloc(sizeof(vrManifold));
				manifold->data = man;


				if (manifolds->head == NULL)
					vrLinkedListAddBack(manifolds, manifold);
				else
				{
					vrManifoldAddContactPoints(((vrManifold*)manifolds->head->data), *man);
					vrFree(man->contacts);
					vrFree(man);

				}
			}
			else
			{
				vrLinkedListClear(manifolds);
				vrFree(man->contacts);
				vrFree(man);
			}


			vrNode* node = manifolds->head;
			while (node)
			{
				vrManifold* manifold = ((vrManifold*)manifolds->head->data);


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
				node = node->next;
			}


			vrBodyIntegrateVelocity(body, frameTime);
			vrBodyIntegrateVelocity(body2, frameTime);
			accumulator -= dt;

		}

		glColor3f(0, 0, 0);

		glLineWidth(4);
		if (body->shape->shapeType == VR_POLYGON)
		{
			vrDebugDrawPolygon(body->shape->shape);
		}
		else
		{
			vrVec2 center = ((vrCircleShape*)body->shape->shape)->center;
			vrFloat orientation = body->orientation;
			vrFloat radius = ((vrCircleShape*)body->shape->shape)->radius;
			glBegin(GL_LINES);
			glVertex2f(center.x, center.y);
			glVertex2f(center.x + cos(orientation)*radius, center.y + sin(orientation)*radius);
			glEnd();
			vrDebugDrawCircle(body->shape->shape);

		}
		if (body2->shape->shapeType == VR_POLYGON)
		{
			vrDebugDrawPolygon(body2->shape->shape);
		}
		else
		{
			vrVec2 center = ((vrCircleShape*)body2->shape->shape)->center;
			vrFloat orientation = body2->orientation;
			vrFloat radius = ((vrCircleShape*)body2->shape->shape)->radius;
			glBegin(GL_LINES);
			glVertex2f(center.x, center.y);
			glVertex2f(center.x + cos(orientation)*radius, center.y + sin(orientation)*radius);
			glEnd();
			vrDebugDrawCircle(body2->shape->shape);
		}
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(0, 800, 800, 0 + 1.f, 1.f, -1.f);
		glfwSwapBuffers(window);
		glfwPollEvents();

	}

	vrBodyDestroy(body);
	return 1;
}

void vrDebugDrawCircle(vrCircleShape* circle)
{
	int vrMax_segs = circle->radius / 3;
	glBegin(GL_LINE_LOOP);
	for (int i = 0; i < vrMax_segs; i++)
	{
		vrFloat theta = 2.0f * 3.1415926f * (vrFloat)i / vrMax_segs;

		vrFloat x = circle->radius * cos(theta);
		vrFloat y = circle->radius * sin(theta);
		glVertex2f(x + circle->center.x, y + circle->center.y );
	}
	glEnd();
}

void vrDebugDrawPolygon(vrPolygonShape* shape)
{

	vrNode* v = shape->vertices->head;
	glBegin(GL_LINE_LOOP);
	glColor3f(0, 0, 0);
	while (v)
	{
		glVertex2f(((vrVertex*)v->data)->vertex.x, ((vrVertex*)v->data)->vertex.y);
		v = v->next;
	}
	glEnd();
}

void vrManifoldDestroy(vrManifold* manifold)
{
	vrFree(manifold->contacts);
	vrFree(manifold);
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode)
{
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GL_TRUE);
	if (action == GLFW_PRESS)
		keys[key] = vrTRUE;

	if (action == GLFW_RELEASE)
		keys[key] = vrFALSE;
}
