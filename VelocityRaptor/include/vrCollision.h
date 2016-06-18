#ifndef HEADER_VRCOLLISION
#define HEADER_VRCOLLISION
#include "vrManifold.h"
#include "vrMath.h"

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

void vrPolyPoly(vrManifold* manifold, const vrPolygonShape A, const vrPolygonShape B);
void vrPolyCircle(vrManifold* manifold, const vrPolygonShape A, const vrCircleShape B);
void vrCirclePoly(vrManifold* manifold, const vrCircleShape A, const vrPolygonShape B);
void vrCircleCircle(vrManifold* manifold, const vrCircleShape A, const vrCircleShape B);
vrFloat vrPolyGetLeastAxis(const vrPolygonShape a, const vrPolygonShape b, vrVec2* least_axis);
vrNode* vrPolyGetFarthestVertex(const vrPolygonShape shape, const vrVec2 normal);
vrEdge vrPolyBestEdge(const vrPolygonShape shape, vrNode* v, const vrVec2 normal);
vrProjection vrProject(const vrPolygonShape a, const vrVec2 axis);
int Clip(const vrVec2 v1, const vrVec2 v2, const vrVec2 n, const vrFloat offset, vrVec2* clipped_points);
vrProjection vrInitProjection(vrFloat max, vrFloat min);
inline vrBOOL BiasGreater(const vrFloat a, const vrFloat b);

#endif