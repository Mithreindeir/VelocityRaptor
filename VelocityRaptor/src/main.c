#include "../include/vrRigidBody.h"
#include "../include/velocityraptor.h"
#include "../include/vrCollision.h"
#include "../include/vrMemoryPool.h"
#include "../include/vrArray.h"
#include "../include/vrWorld.h"
#include "../include/vrHashMap.h"
#include "../include/vrParticleSystem.h"
#include "../include/vrTriangulation.h"
#include "../include/vrShapeCreate.h"
#include "../include/vrDistanceJoint.h"
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
#define DEBUG 0

int
main(void)
{

	GLFWwindow* window;
	glfwInit();
		
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);
	window = glfwCreateWindow(800, 800, "Velocity Raptor", NULL, NULL);
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



	if (1)
	{
		vrRigidBody* body2 = vrBodyInit(vrBodyAlloc());

		//body2->shape = vrShapeCircleInit(body2->shape);
		body2->bodyMaterial.invMass = 0;

		body2->bodyMaterial.invMomentInertia = 0;
		//	((vrCircleShape*)body2->shape->shape)->center = vrVect(400, 600);
		//	((vrCircleShape*)body2->shape->shape)->radius = 50;
		vrShape* s = vrShapeInit(vrShapeAlloc());

		s = vrShapePolyInit(s);

		s->shape = vrPolyBoxInit(s->shape, -100, 0, 100, 8000);
		vrArrayPush(body2->shape, s);
		body2->bodyMaterial.invMass = 0;
		body2->bodyMaterial.invMomentInertia = 0;
		vrWorldAddBody(world, body2);
	}

	if (1)
	{
		vrRigidBody* body2 = vrBodyInit(vrBodyAlloc());

		body2->bodyMaterial.invMass = 0;

		body2->bodyMaterial.invMomentInertia = 0;

		vrShape* s = vrShapeInit(vrShapeAlloc());

		s = vrShapePolyInit(s);

		s->shape = vrPolyBoxInit(s->shape, 800, 0, 100, 8000);
		vrArrayPush(body2->shape, s);
		body2->bodyMaterial.invMass = 0;
		body2->bodyMaterial.invMomentInertia = 0;
		vrWorldAddBody(world, body2);
	}
	
	vrRigidBody* bodyt = vrBodyInit(vrBodyAlloc());
	bodyt->bodyMaterial.invMass = 0;
	bodyt->bodyMaterial.invMomentInertia = 0;
	/*
	
	vrShape* sh = vrShapeInit(vrShapeAlloc());
	sh = vrShapePolyInit(sh);
	sh->shape = vrPolyBoxInit(sh->shape, 0, 700, 800, 100);
	vrArrayPush(bodyt->shape, sh);	
	*/
	vrShapeMold mold = vrShapeMoldInit();
	vrShapeMoldBind(&mold);
	vrPolyBegin(VR_CONVEX_POLYGON);
	vrAddVertex(vrVect(0, 700));
	vrAddVertex(vrVect(800, 700));
	vrAddVertex(vrVect(800, 800));
	vrAddVertex(vrVect(0, 800));
	vrPolyEnd();
	vrArray* arr = vrShapeMoldGetShape(&mold);
	vrArrayCopy(bodyt->shape, arr);
	vrArrayDestroy(arr);
	//bodyt->shape = vrShapeMoldGetShape(&mold);
	bodyt->bodyMaterial.invMass = 0;
	bodyt->bodyMaterial.invMomentInertia = 0;
	vrWorldAddBody(world, bodyt);
	psys = vrParticleSystemInit(vrParticleSystemAlloc());
	
	vrFloat timer = glfwGetTime();
	vrVec2* polygon = NULL;
	int polygonSize = 0;
	int b = 0;
	while (!glfwWindowShouldClose(window))
	{		
		if ((glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_1) == GLFW_PRESS) && ((timer + 0.4) < glfwGetTime()))
		{
			
			double x, y;
			glfwGetCursorPos(window, &x, &y);
			//vrVec2 pos = vrVect(x / 133.333, y / 133.333);
		    //vrParticle* p = vrParticleInit(vrParticleAlloc(), pos);
			//vrArrayPush(psys->particles, p);
			
			vrRigidBody* body3 = vrBodyInit(vrBodyAlloc());

			vrShape* s = vrShapeInit(vrShapeAlloc());
			s = vrShapeCircleInit(s);
			((vrCircleShape*)s->shape)->radius = 50;
			((vrCircleShape*)s->shape)->center = vrVect(x, y);
			//s = vrShapePolyInit(s);

			//s->shape = vrPolyBoxInit(s->shape, x, y, 60, 60);
			vrArrayPush(body3->shape, s);

			body3->bodyMaterial.restitution = 0.0;
			body3->bodyMaterial.mass = 5;
			body3->bodyMaterial.invMass = 1.0/ body3->bodyMaterial.mass;
			body3->bodyMaterial.invMomentInertia = 1.0 / vrMomentForCircle(50, body3->bodyMaterial.mass);
			vrWorldAddBody(world, body3);
			
			if (b == 0)
			{
				body3->bodyMaterial.invMass = 0;
				body3->bodyMaterial.invMomentInertia = 0;
			}
			else
			{
				vrRigidBody* A = body3;
				vrRigidBody* B = world->bodies->data[world->bodies->sizeof_active - 2];

				vrJoint* joint = vrDistanceJointInit(vrJointAlloc(), A, B, A->center, B->center);
				joint->anchorA.initialOrientation = 0;
				joint->anchorA.initialPoint = vrVect(0, 50);
				joint->anchorB.initialOrientation = 0;
				joint->anchorB.initialPoint = vrVect(0, -50);
				vrArrayPush(world->joints, joint);
			}
			b++;
			timer = glfwGetTime();
		}
		if ((glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_2) == GLFW_PRESS) && ((timer + 0.4) < glfwGetTime()))
		{
			double x, y;
			glfwGetCursorPos(window, &x, &y);
			polygonSize += 1;
			if (polygonSize > 1)
				polygon = realloc(polygon, polygonSize*sizeof(vrVec2));
			else
				polygon = vrAlloc(sizeof(vrVec2));
			polygon[polygonSize - 1] = vrVect(x, y);

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
		vrFloat ts = 1.0 / 60.0;
		
		while (accumulator > ts)
		{
			for (int i = 0; i < world->bodies->sizeof_active; i++)
				vrParticleSystemCollide(psys, world->bodies->data[i], 133.333, ts);
			vrParticleSystemStep(psys, ts);
			accumulator -= ts;
		}
		f = glfwGetTime() - f;

		printf("Framerate: %f\n", 1 / f);

		glPointSize(16);
		glLineWidth(12);

		for (int i = 0; i < psys->particles->sizeof_active; i++)
		{
			vrParticle* p = psys->particles->data[i];
			int vrMax_segs = 30;
			glColor3f(p->color.r, p->color.g, p->color.b);

			glBegin(GL_POLYGON);
			for (int i = 0; i < vrMax_segs; i++)
			{
				vrFloat theta = 2.0f * 3.1415926f * (vrFloat)i / vrMax_segs;
				vrFloat radius = 0.3;
				vrFloat x = (radius*133.333) * cos(theta);
				vrFloat y = (radius*133.333) * sin(theta);
				glVertex2f(x + p->pos.x *133.333, y + p->pos.y *133.333);
			}
			glEnd();
		}
		glBegin(GL_LINE_LOOP);
		for (int i = 0; i < polygonSize; i++)
		{
			glVertex2f(polygon[i].x, polygon[i].y);
		}
		glEnd();
		if (polygonSize >= 3)
		{
			glColor3f(0, 0, 0);

			int size = 0;
			vrVec2* p = vrAlloc(sizeof(vrVec2) * polygonSize);
			for (int i = 0; i < polygonSize; i++)
			{
				p[i] = polygon[i];
			}
			vrTriangle * tri = vrEarClip(p, polygonSize, &size);
			glColor3f(1, 0, 0);


			for (int i = 0; i < size; i++)
			{
				glBegin(GL_LINE_LOOP);
				glColor3f(1, 0, 0);
				glLineWidth(8);
				glVertex2f(tri[i].a.x, tri[i].a.y);
				glVertex2f(tri[i].b.x, tri[i].b.y);
				glVertex2f(tri[i].c.x, tri[i].c.y);
				glEnd();

			}
			if (keys[GLFW_KEY_SPACE] && timer + 0.4 < glfwGetTime())
			{
				vrRigidBody* body3 = vrBodyInit(vrBodyAlloc());
				vrFloat accumMoment = 0;
				vrFloat mass = .0005;
				vrFloat area = 0.0;
				for (int i = 0; i < size; i++)
				{
					vrTriangle t = tri[i];
					vrShape* s = vrShapeInit(vrShapeAlloc());

					s = vrShapePolyInit(s);

					s->shape = vrPolyTriangleInit(s->shape, t);

					vrUpdatePolyAxes(s->shape);
					vrArrayPush(body3->shape, s);
				}

				for (int i = 0; i < body3->shape->sizeof_active; i++)
				{
					area += vrAreaForPoly(((vrShape*)body3->shape->data[i])->shape);
				}
				mass *= area;
				for (int i = 0; i < body3->shape->sizeof_active; i++)
				{
					accumMoment += vrMomentForPoly(((vrShape*)body3->shape->data[i])->shape, mass);
				}
				
				accumMoment /= size;
				body3->bodyMaterial.restitution = 0.0;
				body3->bodyMaterial.mass = mass;
				body3->bodyMaterial.invMass = 1.0 / body3->bodyMaterial.mass;
				body3->bodyMaterial.invMomentInertia = 1.0 / accumMoment;
				vrWorldAddBody(world, body3);

				timer = glfwGetTime();
				vrFree(polygon);
				polygonSize = 0;
			}
			vrFree(tri);
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
		*/

		for (int i = 0; i < world->bodies->sizeof_active; i++)
		{
			vrRigidBody* vbody = world->bodies->data[i];
			for (int j = 0; j < vbody->shape->sizeof_active; j++)
			{
				vrShape* shape = vbody->shape->data[j];
				glColor3f(0, 0, 0);

				glLineWidth(4);
				if (shape->shapeType == VR_POLYGON)
				{
					glColor3f(vbody->color.r, vbody->color.g, vbody->color.b);
					vrDebugDrawPolygon(shape->shape);


				}
				else
				{
					vrVec2 center = ((vrCircleShape*)shape->shape)->center;
					vrFloat orientation = vbody->orientation;
					vrFloat radius = ((vrCircleShape*)shape->shape)->radius;
					glColor3f(vbody->color.r, vbody->color.g, vbody->color.b);
					vrDebugDrawCircle(shape->shape);
					glColor3f(0, 0, 0);
					glBegin(GL_LINES);
					glVertex2f(center.x, center.y);
					glVertex2f(center.x + cos(orientation)*radius, center.y + sin(orientation)*radius);
					glEnd();
				}
				if (DEBUG)
				{
					vrOrientedBoundingBox obb = shape->obb;
					glBegin(GL_LINE_LOOP);
					glVertex2f(obb.position.x, obb.position.y);
					glVertex2f(obb.position.x + obb.size.x, obb.position.y);
					glVertex2f(obb.position.x + obb.size.x, obb.position.y + obb.size.y);
					glVertex2f(obb.position.x, obb.position.y + obb.size.y);
					glEnd();
					obb = vbody->obb;
					glBegin(GL_LINE_LOOP);
					glVertex2f(obb.position.x, obb.position.y);
					glVertex2f(obb.position.x + obb.size.x, obb.position.y);
					glVertex2f(obb.position.x + obb.size.x, obb.position.y + obb.size.y);
					glVertex2f(obb.position.x, obb.position.y + obb.size.y);
					glEnd();
				}
			}
		}
	


		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(0, 800, 800, 0 + 1.f, 1.f, -1.f);

		glfwSwapBuffers(window);
		glfwPollEvents();

	}

	return 1;
}

void vrDebugDrawCircle(vrCircleShape* circle)
{
	int vrMax_segs = circle->radius / 3;
	glBegin(GL_POLYGON);
	for (int i = 0; i < vrMax_segs; i++)
	{
		vrFloat theta = 2.0f * 3.1415926f * (vrFloat)i / vrMax_segs;

		vrFloat x = circle->radius * cos(theta);
		vrFloat y = circle->radius * sin(theta);
		glVertex2f(x + circle->center.x, y + circle->center.y );
	}
	glEnd();
	glBegin(GL_LINE_LOOP);
	glColor3f(0, 0, 0);
	for (int i = 0; i < vrMax_segs; i++)
	{
		vrFloat theta = 2.0f * 3.1415926f * (vrFloat)i / vrMax_segs;

		vrFloat x = circle->radius * cos(theta);
		vrFloat y = circle->radius * sin(theta);
		glVertex2f(x + circle->center.x, y + circle->center.y);
	}
	glEnd();
}

void vrDebugDrawPolygon(vrPolygonShape* shape)
{
	vrNode* v = shape->vertices->head;

	glBegin(GL_POLYGON);
	while (v)
	{
		glVertex2f(((vrVertex*)v->data)->vertex.x, ((vrVertex*)v->data)->vertex.y);
		v = v->next;
	}
	glEnd();
	if (DEBUG)
	{
		vrNode* axes = shape->axes->head;
		v = shape->vertices->head;

		glColor3f(0, 0, 0);

		while (axes)
		{
			vrVec2 v1 = ((vrVertex*)v->data)->vertex;
			vrVec2 v2;
			if (v->next)
				v2 = ((vrVertex*)v->next->data)->vertex;
			else
				v2 = ((vrVertex*)shape->vertices->head->data)->vertex;

			vrVec2 p = vrScale(vrAdd(v1, v2), 1.0 / 2.0);
			vrVec2 axis = ((vrVertex*)axes->data)->vertex;
			glBegin(GL_LINES);
			glVertex2f(p.x, p.y);
			glVertex2f(p.x + axis.x * 30, p.y + axis.y * 30);
			glEnd();
			axes = axes->next;
			v = v->next;
		}
	}
	glBegin(GL_LINE_LOOP);
	glColor3f(0, 0, 0);
	v = shape->vertices->head;
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