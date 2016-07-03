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

#ifndef HEADER_VRCOLLISION
#define HEADER_VRCOLLISION
#include "vrManifold.h"
#include "vrMath.h"

#define GJK_MAX_ITERATIONS 20
typedef struct vrProjection
{
	vrFloat max;
	vrFloat min;
} vrProjection;

typedef struct vrEdge
{
	vrVec2 a;
	vrVec2 b;
	vrVec2 max;
	vrVec2 min;
	vrVec2 edge;
} vrEdge;

typedef struct vrSimplex
{
	vrVec2* points;
	int num_points;
} vrSimplex;

void GJKPoly(vrManifold* manifold, const vrPolygonShape A, const vrPolygonShape B);
void vrPolyPoly(vrManifold* manifold, const vrPolygonShape A, const vrPolygonShape B);
void vrPolyCircle(vrManifold* manifold, const vrPolygonShape A, const vrCircleShape B);
void vrCirclePoly(vrManifold* manifold, const vrCircleShape A, const vrPolygonShape B);
void vrCircleCircle(vrManifold* manifold, const vrCircleShape A, const vrCircleShape B);
vrFloat vrPolyGetLeastAxis(const vrPolygonShape a, const vrPolygonShape b, vrVec2* least_axis, vrEdge* pedge, vrVec2 dir);
vrNode* vrPolyGetFarthestVertex(const vrPolygonShape shape, const vrVec2 normal);
vrEdge vrPolyBestEdge(const vrPolygonShape shape, vrNode* v, const vrVec2 normal);
vrProjection vrProject(const vrPolygonShape a, const vrVec2 axis);
int Clip(const vrVec2 v1, const vrVec2 v2, const vrVec2 n, const vrFloat offset, vrVec2* clipped_points);
vrProjection vrInitProjection(vrFloat max, vrFloat min);
inline vrBOOL BiasGreater(const vrFloat a, const vrFloat b);

#endif
