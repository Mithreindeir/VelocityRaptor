#include "../include/vrShapeCreate.h"

vrShapeMold vrShapeMoldInit()
{
	vrShapeMold s;
	s.offset = vrVect(0, 0);
	s.type = VR_NOSHAPE;
	return s;
}

void vrShapeMoldBind(vrShapeMold * shapeMold)
{
	currentMold = shapeMold;
}

void vrPolyBegin(vrShapePolyType shapeT)
{
	if (currentMold->type == VR_NOSHAPE)
	{
		currentMold->polyMold.num_vertices = 0;
		currentMold->polyMold.polygon = NULL;
	}
	currentMold->type = VR_POLYGON;
	currentMold->polyMold.polyType = shapeT;
	
}

void vrAddVertex(vrVec2 v)
{
	currentMold->polyMold.num_vertices += 1;
	if (currentMold->polyMold.num_vertices > 1)
		currentMold->polyMold.polygon = vrRealloc(currentMold->polyMold.polygon, currentMold->polyMold.num_vertices*sizeof(vrVec2));
	else
		currentMold->polyMold.polygon = vrAlloc(sizeof(vrVec2));
	currentMold->polyMold.polygon[currentMold->polyMold.num_vertices - 1] = v;
}

void vrPolyEnd()
{
	currentMold = NULL;
}

vrArray * vrShapeMoldGetShape(vrShapeMold * shapemold)
{
	vrArray* shapes = vrArrayInit(vrArrayAlloc(), sizeof(vrShape*));
	if (shapemold->type == VR_CIRCLE)
	{
		vrShape* shape = vrShapeInit(vrShapeAlloc());
		shape->shape = vrShapeCircleInit(shape);
		vrCircleShape* circle = shape->shape;
		circle->center = shapemold->circleMold.center;
		circle->radius = shapemold->circleMold.radius;
		vrArrayPush(shapes, shape);
	}
	else if (shapemold->type == VR_POLYGON)
	{
		if (shapemold->polyMold.polyType == VR_CONCAVE_POLYGON)
		{
			//Use a buffer, because ear clip vrFrees the arrays memory
			int size = 0;
			vrVec2* polygon_buffer = vrAlloc(sizeof(vrVec2) * shapemold->polyMold.num_vertices);
			for (int i = 0; i < shapemold->polyMold.num_vertices; i++)
			{
				polygon_buffer[i] = shapemold->polyMold.polygon[i];
			}
			vrTriangle * tri = vrEarClip(polygon_buffer, shapemold->polyMold.num_vertices, &size);

			for (int i = 0; i < size; i++)
			{
				vrTriangle t = tri[i];
				vrShape* s = vrShapeInit(vrShapeAlloc());

				s = vrShapePolyInit(s);

				s->shape = vrPolyTriangleInit(s->shape, t);

				vrUpdatePolyAxes(s->shape);
				vrArrayPush(shapes, s);
			}
		}
		else
		{
			vrShape* s = vrShapeInit(vrShapeAlloc());

			s = vrShapePolyInit(s);
			for (int i = 0; i < shapemold->polyMold.num_vertices; i++)
			{
				vrAddVertexToPolyShape(s->shape, shapemold->polyMold.polygon[i]);
			}
			vrUpdatePolyAxes(s->shape);
			vrArrayPush(shapes, s);
		}
	}
	return shapes;
}
