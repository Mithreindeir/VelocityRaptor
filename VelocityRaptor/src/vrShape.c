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

#include "..\include\vrShape.h"
#include <stdio.h>


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
	polygon->num_axes = 0;
	polygon->vertices = NULL;
	polygon->axes = NULL;
	return polygon;
}

void vrPolyDestroy(vrPolygonShape * polygon)
{
	vrFree(polygon->vertices);
	vrFree(polygon->axes);

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
	shape->vertices = vrRealloc(shape->vertices, sizeof(vrVec2) * (1  + shape->num_vertices));
	shape->vertices[shape->num_vertices] = vertex;
	shape->num_vertices += 1;
	vrUpdatePolyCenter(shape);
}

void vrAddNormalToPolyShape(vrPolygonShape * shape, vrVec2 axis)
{
}

void vrMovePolyShape(vrPolygonShape * shape, vrVec2 move)
{
	for (int i = 0; i < shape->num_vertices; i++)
		shape->vertices[i] = vrAdd(shape->vertices[i], move);

	vrUpdatePolyCenter(shape);
}

vrPolygonShape* vrPolyBoxInit(vrPolygonShape* shape, vrFloat x, vrFloat y, vrFloat w, vrFloat h)
{
	vrAddVertexToPolyShape(shape, vrVect(x, y));
	vrAddVertexToPolyShape(shape, vrVect(x  + w,y));
	vrAddVertexToPolyShape(shape, vrVect(x + w,y + h));
	vrAddVertexToPolyShape(shape, vrVect(x, y + h));
	return shape;
}

vrPolygonShape * vrPolyBoxInitPoint(vrPolygonShape * shape, vrFloat x, vrFloat y, vrFloat hw, vrFloat hh)
{
	x -= hw;
	y -= hh;
	hw *= 2;
	hh *= 2;
	vrAddVertexToPolyShape(shape, vrVect(x, y));
	vrAddVertexToPolyShape(shape, vrVect(x + hw, y));
	vrAddVertexToPolyShape(shape, vrVect(x + hw, y + hh));
	vrAddVertexToPolyShape(shape, vrVect(x, y + hh));
	return shape;
}

vrPolygonShape * vrPolyTriangleInit(vrPolygonShape * shape, vrTriangle t)
{
	vrAddVertexToPolyShape(shape, t.a);
	vrAddVertexToPolyShape(shape, t.b);
	vrAddVertexToPolyShape(shape, t.c);

	return shape;
}

void vrRotatePolyShape(vrPolygonShape * shape, vrFloat angle, vrVec2 center)
{
	vrFloat ca = VR_COSINE(angle);
	vrFloat sa = VR_SINE(angle);
	vrVec2 c = center;
	for (int i = 0; i < shape->num_vertices; i++)
	{
		shape->vertices[i] = vrSub(shape->vertices[i], c);
		shape->vertices[i] = vrVect(shape->vertices[i].x * ca - shape->vertices[i].y * sa, shape->vertices[i].x * sa + shape->vertices[i].y * ca);
		shape->vertices[i] = vrAdd(shape->vertices[i], c);
	}

	vrUpdatePolyAxes(shape);
}

void vrUpdatePolyCenter(vrPolygonShape * shape)
{
	vrVec2 center = vrVect(0, 0);
	for (int i = 0; i < shape->num_vertices; i++)
	{
		center = vrAdd(center, shape->vertices[i]);
	}
	center = vrScale(center, 1.0 / shape->num_vertices);
	shape->center = center;
}

void vrUpdatePolyAxes(vrPolygonShape * shape)
{
	if (shape->num_axes != shape->num_vertices)
	{
		if (shape->num_axes == 0)
		{
			shape->axes = vrAlloc(sizeof(vrVec2) * shape->num_vertices);
			shape->num_axes = shape->num_vertices;
		}
		else
		{
			shape->axes = vrRealloc(shape->axes, sizeof(vrVec2) * shape->num_vertices);
			shape->num_axes = shape->num_vertices;
		}
	}
	for (int i = 0; i < shape->num_vertices; i++)
	{
		vrVec2 axis;
		vrVec2 v1 = shape->vertices[i];
		vrVec2 v2 = (i < (shape->num_vertices - 1)) ? shape->vertices[i + 1] : shape->vertices[0];
		axis = vrSub(v2, v1);
		axis = vrNormalize(vrVect(axis.y, -axis.x));
		if (vrDot(axis, vrSub(shape->center, v1)) >= 0)
			axis = vrVect(-axis.x, -axis.y);
		shape->axes[i] = axis;
	}
}

vrVec2 vrPolyGetCenter(vrPolygonShape * shape)
{
	return shape->center;
}

vrOrientedBoundingBox vrPolyGetOBB(vrPolygonShape * shape)
{
	if (shape->num_vertices == 0) return  vrOBBCreate(vrVect(0, 0), vrVect(0, 0));
	
	int least_x = shape->vertices[0].x;
	int farthest_x = shape->vertices[0].x;
	int least_y = shape->vertices[0].y;
	int farthest_y = shape->vertices[0].y;
	for (int i = 1; i < shape->num_vertices; i++)
	{
		
		vrVec2 vert = shape->vertices[i];
		if (vert.x < least_x) least_x = vert.x;
		if (vert.x > farthest_x) farthest_x = vert.x;
		if (vert.y < least_y) least_y = vert.y;
		if (vert.y > farthest_y) farthest_y = vert.y;

	}
	return vrOBBCreate(vrVect(least_x-1, least_y-1), vrVect(farthest_x - least_x + 1, farthest_y - least_y + 1));
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

void vrRotateCircleShape(vrCircleShape * shape, vrFloat angle, vrVec2 center)
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
