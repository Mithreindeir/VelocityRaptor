#include "../include/vrRigidBody.h"
#include "../include/velocityraptor.h"
#include "../include/vrCollision.h"
#include "../include/vrMemoryPool.h"
#include "../include/vrArray.h"
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
vrArray* arr;
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
	body->shape->shape = vrPolyBoxInit(body->shape->shape, 400, 425, 50, 50);
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
	/*
	vrHashTable* map = vrHashTableInit(vrHashTableAlloc(), 200);
	map->deleteFunc = vrFree;
	vrHashEntry* entry = vrAlloc(sizeof(vrHashEntry));
	entry->data = vrAlloc(sizeof(int));
	*((int*)entry->data) = 5;
	entry->key = 29348;
	vrHashTableInsert(map, entry, entry->key);
	vrHashEntry* t = vrHashTableLookup(map, 29348);
	printf("%d and \n", *((int*)t->data));
	vrHashTableRemove(map, 29348);
	t = vrHashTableLookup(map, 29348);
	printf("%d and \n", *((int*)t->data));
	*/
	vrFloat timer = glfwGetTime();
	while (!glfwWindowShouldClose(window))
	{
		if ((glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_1) == GLFW_PRESS) && ((timer + 0.4) < glfwGetTime()))
		{
			double x, y;
			glfwGetCursorPos(window, &x, &y);
			vrRigidBody* body3 = vrBodyInit(vrBodyAlloc());
			body3->shape = vrShapeInit(vrShapeAlloc());
			body3->shape = vrShapePolyInit(body3->shape);
			body3->shape->shape = vrPolyBoxInit(body3->shape->shape, x, y, 15, 15);
			body3->bodyMaterial.restitution = 0.0;
			vrWorldAddBody(world, body3);
			timer = glfwGetTime();

		}
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
		vrFloat f = glfwGetTime();
		vrWorldStep(world);
		f = glfwGetTime() - f;
	//	printf("Framerate: %f\n", 1 / f);

		for (int i = 0; i < world->bodies->sizeof_active; i++)
		{
			vrRigidBody* vbody = world->bodies->data[i];
			glColor3f(0, 0, 0);

			glLineWidth(4);
			if (vbody->shape->shapeType == VR_POLYGON)
			{
				vrDebugDrawPolygon(vbody->shape->shape);
			}
			else
			{
				vrVec2 center = ((vrCircleShape*)vbody->shape->shape)->center;
				vrFloat orientation = vbody->orientation;
				vrFloat radius = ((vrCircleShape*)vbody->shape->shape)->radius;
				glBegin(GL_LINES);
				glVertex2f(center.x, center.y);
				glVertex2f(center.x + cos(orientation)*radius, center.y + sin(orientation)*radius);
				glEnd();
				vrDebugDrawCircle(vbody->shape->shape);

			}
			/*
			vrOrientedBoundingBox obb = vbody->shape->obb;
			glBegin(GL_LINE_LOOP);
			glVertex2f(obb.position.x, obb.position.y);
			glVertex2f(obb.position.x + obb.size.x, obb.position.y);
			glVertex2f(obb.position.x + obb.size.x, obb.position.y + obb.size.y);
			glVertex2f(obb.position.x, obb.position.y + obb.size.y);
			glEnd();
			*/
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


void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode)
{
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GL_TRUE);
	if (action == GLFW_PRESS)
		keys[key] = vrTRUE;

	if (action == GLFW_RELEASE)
		keys[key] = vrFALSE;
}