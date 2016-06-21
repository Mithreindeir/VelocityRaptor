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

#include "..\include\vrShape.h"
#include <stdio.h>

void vrVertexDestroy(vrVertex * vertex)
{
	vrFree(vertex);
}

vrShape * vrShapeAlloc()
{
	return vrAlloc(sizeof(vrShape));
}

vrShape * vrShapeInit(vrShape * shape)
{
	shape->shape = NULL;
	shape->shapeType = VR_NOSHAPE;
	return shape;
}

void vrShapeDestroy(vrShape * shape)
{
	vrShapeDataDestroy(shape);
	vrFree(shape);
}

void vrShapeDataDestroy(vrShape * shape)
{
	switch (shape->shapeType)
	{
	case VR_CIRCLE:
		vrCircleDestroy(shape->shape);
		break;
	case VR_POLYGON:
		vrPolyDestroy(shape->shape);
		break;
	}
	shape->shape = NULL;
}

vrPolygonShape * vrPolyAlloc()
{
	return vrAlloc(sizeof(vrPolygonShape));
}

vrPolygonShape * vrPolyInit(vrPolygonShape * polygon)
{
	polygon->center = vrVect(0, 0);
	polygon->num_vertices = 0;
	polygon->vertices = vrLinkedListInit(vrLinkedListAlloc());
	polygon->axes = vrLinkedListInit(vrLinkedListAlloc());
	polygon->vertices->deleteFunc = vrVertexDestroy;
	polygon->axes->deleteFunc = vrVertexDestroy;
	return polygon;
}

void vrPolyDestroy(vrPolygonShape * polygon)
{
	vrLinkedListClear(polygon->vertices);
	vrLinkedListClear(polygon->axes);

	vrFree(polygon);
}

vrShape * vrShapePolyInit(vrShape * shape)
{
	if (shape->shapeType != VR_NOSHAPE)
		vrShapeDataDestroy(shape);

	shape->shape = vrPolyInit(vrPolyAlloc());
	shape->getCenter = &vrPolyGetCenter;
	shape->rotate = &vrRotatePolyShape;
	shape->move = &vrMovePolyShape;
	shape->updateOBB = &vrPolyGetOBB;
	shape->shapeType = VR_POLYGON;
	return shape;
}

void vrAddVertexToPolyShape(vrPolygonShape * shape, vrVec2 vertex)
{
	vrNode* new_vertex = vrAlloc(sizeof(vrNode));
	new_vertex->data = vrAlloc(sizeof(vrVertex));
	((vrVertex*)new_vertex->data)->vertex = vertex;
	vrLinkedListAddBack(shape->vertices, new_vertex);
}

void vrAddNormalToPolyShape(vrPolygonShape * shape, vrVec2 axis)
{
}

void vrMovePolyShape(vrPolygonShape * shape, vrVec2 move)
{
	vrNode* v = shape->vertices->head;
	while (v)
	{
		((vrVertex*)v->data)->vertex = vrAdd(((vrVertex*)v->data)->vertex, move);
		v = v->next;
	}
	vrUpdatePolyCenter(shape);
}

vrPolygonShape* vrPolyBoxInit(vrPolygonShape* shape, vrFloat x, vrFloat y, vrFloat hw, vrFloat hh)
{
	x -= hw;
	y -= hh;
	hw *= 2;
	hh *= 2;
	vrAddVertexToPolyShape(shape, vrVect(x, y));
	vrAddVertexToPolyShape(shape, vrVect(x  + hw,y));
	vrAddVertexToPolyShape(shape, vrVect(x + hw,y + hh));
	vrAddVertexToPolyShape(shape, vrVect(x, y + hh));
	return shape;
}

void vrRotatePolyShape(vrPolygonShape * shape, vrFloat angle)
{
	vrFloat ca = VR_COSINE(angle);
	vrFloat sa = VR_SINE(angle);
	vrVec2 c = shape->center;
	vrNode* v = shape->vertices->head;
	while(v)
	{
		((vrVertex*)v->data)->vertex = vrSub(((vrVertex*)v->data)->vertex, c);
		((vrVertex*)v->data)->vertex = vrVect(((vrVertex*)v->data)->vertex.x * ca - ((vrVertex*)v->data)->vertex.y * sa, ((vrVertex*)v->data)->vertex.x *sa + ((vrVertex*)v->data)->vertex.y * ca);
		((vrVertex*)v->data)->vertex = vrAdd(((vrVertex*)v->data)->vertex, c);
		v = v->next;
	}
	vrUpdatePolyAxes(shape);
}

void vrUpdatePolyCenter(vrPolygonShape * shape)
{
	vrVec2 center = vrVect(0, 0);
	int num_v = 0;
	vrNode* vertex = shape->vertices->head;
	while (vertex)
	{
		center = vrAdd(center, ((vrVertex*)vertex->data)->vertex);
		num_v++;
		vertex = vertex->next;
	}
	center = vrVect(center.x / num_v, center.y / num_v);

	shape->center = center;
}

void vrUpdatePolyAxes(vrPolygonShape * shape)
{
	vrNode* vertex = shape->vertices->head;
	vrNode* axes = shape->axes->head;
	while (vertex)
	{
		if (axes == NULL)
		{
			vrNode* a = vrAlloc(sizeof(vrNode));
			a->data = vrAlloc(sizeof(vrVertex));
			((vrVertex*)a->data)->vertex = vrVect(0, 0);
			vrLinkedListAddBack(shape->axes, a);
			axes = a;
		}
		if (vertex->next == NULL)
		{
			((vrVertex*)axes->data)->vertex = vrSub(((vrVertex*)vertex->data)->vertex, ((vrVertex*)shape->vertices->head->data)->vertex);
			((vrVertex*)axes->data)->vertex = vrNormalize(vrVect(((vrVertex*)axes->data)->vertex.y, -((vrVertex*)axes->data)->vertex.x));
		}
		else
		{
			((vrVertex*)axes->data)->vertex = vrSub(((vrVertex*)vertex->data)->vertex, ((vrVertex*)vertex->next->data)->vertex);
			((vrVertex*)axes->data)->vertex = vrNormalize(vrVect(((vrVertex*)axes->data)->vertex.y, -((vrVertex*)axes->data)->vertex.x));
		}
		axes = axes->next;
		vertex = vertex->next;
	}
}

vrVec2 vrPolyGetCenter(vrPolygonShape * shape)
{
	return shape->center;
}

vrOrientedBoundingBox vrPolyGetOBB(vrPolygonShape * shape)
{
	vrNode* vertex = shape->vertices->head;
	vrVertex* v = ((vrVertex*)vertex->data);
	int least_x = v->vertex.x;
	int farthest_x = v->vertex.x;
	int least_y = v->vertex.y;
	int farthest_y = v->vertex.y;
	while (vertex)
	{
		v = ((vrVertex*)vertex->data);
		vrVec2 vert = v->vertex;
		if (vert.x < least_x) least_x = vert.x;
		if (vert.x > farthest_x) farthest_x = vert.x;
		if (vert.y < least_y) least_y = vert.y;
		if (vert.y > farthest_y) farthest_y = vert.y;

		vertex = vertex->next;
	}
	return vrOBBCreate(vrVect(least_x, least_y), vrVect(farthest_x - least_x, farthest_y - least_y));
}

vrCircleShape * vrCircleAlloc()
{
	return vrAlloc(sizeof(vrCircleShape));
}

vrCircleShape * vrCircleInit(vrCircleShape * circle)
{
	circle->center = vrVect(0, 0);
	circle->radius = 0;
	return circle;
}

void vrCircleDestroy(vrCircleShape * circle)
{
	vrFree(circle);
}

vrShape * vrShapeCircleInit(vrShape * shape)
{
	if (shape->shapeType != VR_NOSHAPE)
		vrShapeDataDestroy(shape);

	shape->shape = vrCircleInit(vrCircleAlloc());
	shape->rotate = &vrRotateCircleShape;
	shape->getCenter = &vrCircleGetCenter;
	shape->move = &vrMoveCircleShape;
	shape->updateOBB = &vrCircleGetOBB;
	shape->shapeType = VR_CIRCLE;
	return shape;
}

void vrRotateCircleShape(vrCircleShape * shape, vrFloat angle)
{
	//Circles rotation can be stored in body
}

void vrMoveCircleShape(vrCircleShape * shape, vrVec2 move)
{
	shape->center = vrAdd(shape->center, move);
}

vrVec2 vrCircleGetCenter(vrCircleShape * shape)
{
	return shape->center;
}

vrOrientedBoundingBox vrCircleGetOBB(vrCircleShape * shape)
{
	return vrOBBCreate(vrVect(shape->center.x - shape->radius, shape->center.y - shape->radius), vrVect(shape->radius * 2, shape->radius * 2));
}
