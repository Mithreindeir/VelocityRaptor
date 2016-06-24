#include "../include/vrRigidBody.h"
#include "../include/velocityraptor.h"
#include "../include/vrCollision.h"
#include "../include/vrMemoryPool.h"
#include "../include/vrArray.h"
#include "../include/vrWorld.h"
#include "../include/vrHashMap.h"
#include "../include/vrParticleSystem.h"
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
vrParticleSystem* psys;

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
	//body->shape = vrShapePolyInit(body->shape);
	//body->shape->shape = vrPolyBoxInit(body->shape->shape, 400, 425, 50, 50);
	body->bodyMaterial.restitution = 0.0;

	body->shape = vrShapeCircleInit(body->shape);
	((vrCircleShape*)body->shape->shape)->center = vrVect(400, 400);
	((vrCircleShape*)body->shape->shape)->radius = 50;

	if (1)
	{
		vrRigidBody* body2 = vrBodyInit(vrBodyAlloc());

		body2->shape = vrShapeInit(vrShapeAlloc());
		//body2->shape = vrShapeCircleInit(body2->shape);
		body2->bodyMaterial.invMass = 0;

		body2->bodyMaterial.invMomentInertia = 0;
		//	((vrCircleShape*)body2->shape->shape)->center = vrVect(400, 600);
		//	((vrCircleShape*)body2->shape->shape)->radius = 50;

		body2->shape = vrShapePolyInit(body2->shape);

		body2->shape->shape = vrPolyBoxInit(body2->shape->shape, -100, 0, 100, 8000);
		body2->bodyMaterial.invMass = 0;
		body2->bodyMaterial.invMomentInertia = 0;
		vrWorldAddBody(world, body2);
	}
	vrRigidBody* bodyt = vrBodyInit(vrBodyAlloc());

	bodyt->shape = vrShapeInit(vrShapeAlloc());
	//body2->shape = vrShapeCircleInit(body2->shape);
	bodyt->bodyMaterial.invMass = 0;

	bodyt->bodyMaterial.invMomentInertia = 0;
	//	((vrCircleShape*)body2->shape->shape)->center = vrVect(400, 600);
	//	((vrCircleShape*)body2->shape->shape)->radius = 50;

	bodyt->shape = vrShapePolyInit(bodyt->shape);

	bodyt->shape->shape = vrPolyBoxInit(bodyt->shape->shape, 0, 700, 800, 100);
	bodyt->bodyMaterial.invMass = 0;
	bodyt->bodyMaterial.invMomentInertia = 0;
	vrWorldAddBody(world, bodyt);

	if (1)
	{
		vrRigidBody* body2 = vrBodyInit(vrBodyAlloc());

		body2->shape = vrShapeInit(vrShapeAlloc());
		//body2->shape = vrShapeCircleInit(body2->shape);
		body2->bodyMaterial.invMass = 0;

		body2->bodyMaterial.invMomentInertia = 0;
		//	((vrCircleShape*)body2->shape->shape)->center = vrVect(400, 600);
		//	((vrCircleShape*)body2->shape->shape)->radius = 50;

		body2->shape = vrShapePolyInit(body2->shape);

		body2->shape->shape = vrPolyBoxInit(body2->shape->shape, 800, 0, 100, 8000);
		body2->bodyMaterial.invMass = 0;
		body2->bodyMaterial.invMomentInertia = 0;
		vrWorldAddBody(world, body2);
	}
	vrManifold* m = vrManifoldInit(vrManifoldAlloc());
	
	//vrPolyPoly(m, *((vrPolygonShape*)body->shape->shape), *((vrPolygonShape*)body2->shape->shape));
	//vrWorldAddBody(world, body);



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
	/*
	for (int i = 14; i >= 0; i--) {
		for (int j = 0; j <= i; j++) {
			vrRigidBody* body3 = vrBodyInit(vrBodyAlloc());
			body3->shape = vrShapeInit(vrShapeAlloc());
			body3->shape = vrShapePolyInit(body3->shape);
			vrVec2 pos = vrVect(400 + j * 32 - i * 16, (700 - 448) - 32 + i * 32);
			body3->shape->shape = vrPolyBoxInit(body3->shape->shape, pos.x, pos.y, 30, 30);
			body3->bodyMaterial.restitution = 0.0;
			vrWorldAddBody(world, body3);

		}
	}
	*/
	/*
	vrVec2 p = vrVect(150, 100);
	int size = 25;
	vrVec2 verts[5];
	for (int i = 0; i<5; i++) {
		vrFloat angle = -2.0f*VR_PI*i / ((vrFloat)5);
		verts[i] = vrVect(size * cos(angle), size * sin(angle));
	}
	for (int j = 0; j < 8; j++)
	{
		for (int i = 0; i < 10; i++)
		{
			
			vrRigidBody* body3 = vrBodyInit(vrBodyAlloc());
			body3->shape = vrShapeInit(vrShapeAlloc());
			body3->shape = vrShapePolyInit(body3->shape);

			vrAddVertexToPolyShape(body3->shape->shape, vrAdd(verts[0], p));
			vrAddVertexToPolyShape(body3->shape->shape, vrAdd(verts[1], p));
			vrAddVertexToPolyShape(body3->shape->shape, vrAdd(verts[2], p));
			vrAddVertexToPolyShape(body3->shape->shape, vrAdd(verts[3], p));
			vrAddVertexToPolyShape(body3->shape->shape, vrAdd(verts[4], p));
			body3->bodyMaterial.restitution = 0.0;
			body3->bodyMaterial.invMomentInertia = 11.0 / vrMomentForPoly(body3->shape->shape, body3->bodyMaterial.mass);
			vrWorldAddBody(world, body3);
			//printf("Moment = %f \n", vrMomentForPoly(body3->shape->shape, body3->bodyMaterial.mass));

			p.x += 50;
		}
		p.y -= 150;
		p.x = 150;
	}
	p.y = 200;
	p.x = 100;
	for (int j = 0; j < 8; j++)
	{
		for (int i = 0; i < 7; i++)
		{
			vrRigidBody* body3 = vrBodyInit(vrBodyAlloc());
			body3->shape = vrShapeInit(vrShapeAlloc());
			body3->shape = vrShapePolyInit(body3->shape);
			vrAddVertexToPolyShape(body3->shape->shape, vrAdd(vrVect(0, -size), p));
			vrAddVertexToPolyShape(body3->shape->shape, vrAdd(vrVect(size, size), p));
			vrAddVertexToPolyShape(body3->shape->shape, vrAdd(vrVect(-size, size), p));

			body3->bodyMaterial.restitution = 0.0;
			body3->bodyMaterial.invMass = 0;
			body3->bodyMaterial.invMomentInertia = 0;
			vrWorldAddBody(world, body3);
			p.x += 100;
		}
		p.y += 150;
		p.x = 100;
	}
	*/
	psys = vrParticleSystemInit(vrParticleSystemAlloc());
	
	vrFloat timer = glfwGetTime();
	while (!glfwWindowShouldClose(window))
	{
		/*
		if ((glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_1) == GLFW_PRESS) && ((timer + 0.4) < glfwGetTime()))
		{
			double x, y;
			glfwGetCursorPos(window, &x, &y);
			vrRigidBody* body3 = vrBodyInit(vrBodyAlloc());
			body3->shape = vrShapeInit(vrShapeAlloc());
			body3->shape = vrShapePolyInit(body3->shape);
			body3->shape->shape = vrPolyBoxInit(body3->shape->shape, x, y, 60, 60);
			body3->bodyMaterial.restitution = 0.0;
			body3->bodyMaterial.invMomentInertia = 1.0 / vrMomentForBox(60, 60, 1);
			vrWorldAddBody(world, body3);
			timer = glfwGetTime();

		}
		*/
		if ((glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_1) == GLFW_PRESS) && ((timer + 0.4) < glfwGetTime()))
		{
			if (psys->gravity.x == 0)
			{
				psys->gravity.x = 9.81;
				psys->gravity.y = 0;
			}
			else
			{
				psys->gravity.x = 0;
				psys->gravity.y = 9.81;

			}
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
		//vrWorldStep(world);
		f = glfwGetTime() - f;
		//printf("Framerate: %f\n", 1 / f);
		vrFloat ts = 0.01;
		while (accumulator > ts)
		{
			vrParticleSystemStep(psys, ts);
			accumulator -= ts;
		}

		glPointSize(16);
		glLineWidth(12);
		glColor3f(1, 0, 0);

		for (int i = 0; i < psys->particles->sizeof_active; i++)
		{
			vrParticle* p = psys->particles->data[i];
			int vrMax_segs = 30;
			glBegin(GL_LINE_LOOP);
			for (int i = 0; i < vrMax_segs; i++)
			{
				vrFloat theta = 2.0f * 3.1415926f * (vrFloat)i / vrMax_segs;

				vrFloat x = (p->r*133.333) * cos(theta);
				vrFloat y = (p->r *133.333) * sin(theta);
				glVertex2f(x + p->pos.x *133.333, y + p->pos.y *133.333);
			}
			glEnd();
		}
		/*

		for (int i = 0; i < world->bodies->sizeof_active; i++)
		{
			vrRigidBody* b = world->bodies->data[i];
			vrVec2 center = b->shape->getCenter(b->shape->shape);
			if (center.y > 900)
			{
				vrFloat move = -200 - center.y;
				vrVec2 mov = vrVect(0, move);
				b->shape->move(b->shape->shape, mov);
				b->position = vrAdd(b->position, mov);
			}

		}
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
			
			vrOrientedBoundingBox obb = vbody->shape->obb;
			glBegin(GL_LINE_LOOP);
			glVertex2f(obb.position.x, obb.position.y);
			glVertex2f(obb.position.x + obb.size.x, obb.position.y);
			glVertex2f(obb.position.x + obb.size.x, obb.position.y + obb.size.y);
			glVertex2f(obb.position.x, obb.position.y + obb.size.y);
			glEnd();
			
		}
	*/
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