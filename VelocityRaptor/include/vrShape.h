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

#ifndef HEADER_VRSHAPE
#define HEADER_VRSHAPE
#include "vrMath.h"
#include "vrLinkedList.h"
#include "velocityraptor.h"
#include "vrBoundingBox.h"

typedef void*(*vrShapeMoveFunc)(void* shape, vrVec2 move);
typedef void*(*vrShapeRotateFunc)(void* shape, vrFloat angle);
typedef vrVec2(*vrShapeCenterFunc)(void* shape);
typedef vrOrientedBoundingBox(*vrShapeOBBFunc)(void* shape);


typedef enum
{
	VR_POLYGON,
	VR_CIRCLE,
	VR_NOSHAPE
} vrShapeType;

//Vertex in linked list structure
typedef struct vrVertex
{
	vrVec2 vertex;
} vrVertex;

void vrVertexDestroy(vrVertex* vertex);


//Holds shape
typedef struct vrShape
{
	void* shape;
	vrShapeType shapeType;
	vrShapeMoveFunc move;
	vrShapeRotateFunc rotate;
	vrShapeCenterFunc getCenter;
	vrShapeOBBFunc updateOBB;

	vrOrientedBoundingBox obb;
} vrShape;

//Polygon shape

typedef struct vrPolygonShape
{
	vrLinkedList* vertices;
	vrLinkedList* axes;
	int num_vertices;
	vrVec2 center;

} vrPolygonShape;

typedef struct vrCircleShape
{
	vrFloat radius;
	vrVec2 center;
} vrCircleShape;

//Allocates a shape
vrShape* vrShapeAlloc();
vrShape* vrShapeInit(vrShape* shape);
void vrShapeDestroy(vrShape* shape);
//Misc:
void vrShapeDataDestroy(vrShape* shape);
//Returns a shape struct with an empty polygon
vrPolygonShape* vrPolyAlloc();
vrPolygonShape* vrPolyInit(vrPolygonShape* polygon);
void vrPolyDestroy(vrPolygonShape* polygon);
//Misc:
vrShape* vrShapePolyInit(vrShape* shape);
vrPolygonShape* vrPolyBoxInit(vrPolygonShape* shape, vrFloat x, vrFloat y, vrFloat w, vrFloat h);
vrPolygonShape* vrPolyBoxInitPoint(vrPolygonShape* shape, vrFloat x, vrFloat y, vrFloat hw, vrFloat hh);
void vrAddVertexToPolyShape(vrPolygonShape* shape, vrVec2 vertex);
void vrAddNormalToPolyShape(vrPolygonShape* shape, vrVec2 axis);
void vrMovePolyShape(vrPolygonShape* shape, vrVec2 move);
void vrRotatePolyShape(vrPolygonShape* shape, vrFloat angle);
void vrUpdatePolyCenter(vrPolygonShape* shape);
void vrUpdatePolyAxes(vrPolygonShape* shape);
vrVec2 vrPolyGetCenter(vrPolygonShape* shape);
vrOrientedBoundingBox vrPolyGetOBB(vrPolygonShape* shape);

//Returns a shape struct with an empty circle
vrCircleShape* vrCircleAlloc();
vrCircleShape* vrCircleInit(vrCircleShape* circle);
void vrCircleDestroy(vrCircleShape* circle);
//Misc:
vrShape* vrShapeCircleInit(vrShape* shape);
//Rotates a polygon by a given angle
void vrRotateCircleShape(vrCircleShape* shape, vrFloat angle);
void vrMoveCircleShape(vrCircleShape* shape, vrVec2 move);
vrVec2 vrCircleGetCenter(vrCircleShape* shape);
vrOrientedBoundingBox vrCircleGetOBB(vrCircleShape* shape);

#endif
