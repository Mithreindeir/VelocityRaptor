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

#ifndef HEADER_VRSHAPECREATE
#define HEADER_VRSHAPECREATE

#include "vrShape.h"

///Type of polygon, (if unknown, go with concave)
enum vrShapePolyType
{
        VR_CONVEX_POLYGON,
        VR_CONCAVE_POLYGON
};

typedef enum vrShapePolyType vrShapePolyType;
typedef struct vrShapeMold vrShapeMold;
///Current mold being edited
static vrShapeMold* currentMold;
///Current type of polygon mold being edited
static vrShapePolyType currentPolyType;
///Current type of shape being edited
static vrShapeType currentShapeType;

///Holds info needed for polygon
typedef struct vrPolygonMold
{
	///Polygon as an array of vectors
	vrVec2* polygon;
	///Number of vertices
	int num_vertices;
	///Type of polygon
	vrShapePolyType polyType;
} vrPolygonMold;

///Holds info needed for circle
typedef struct vrCircleMold
{
	vrVec2 center;
	vrFloat radius;
} vrCircleMold;
///Shape mold, to make multiple of same type of shape
struct vrShapeMold
{
	vrShapeType type;
	vrVec2 offset;
	union
	{
		vrPolygonMold polyMold;
		vrCircleMold circleMold;
	} mold;
};

///Initializes a shape mold
vrShapeMold vrShapeMoldInit();
///Binds the shape as current mold
void vrShapeMoldBind(vrShapeMold* shapemold);
///Begins shape editing
void vrPolyBegin(vrShapePolyType shapeT);
///Adds a vertex to a mold
void vrAddVertex(vrVec2 v);
///Ends shape editing
void vrPolyEnd();
///Gets a shape containing the edited shape
vrArray* vrShapeMoldGetShape(vrShapeMold* shapemold);

#endif
