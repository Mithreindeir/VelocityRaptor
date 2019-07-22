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
		currentMold->mold.polyMold.num_vertices = 0;
		currentMold->mold.polyMold.polygon = NULL;
	}
	currentMold->type = VR_POLYGON;
	currentMold->mold.polyMold.polyType = shapeT;

}

void vrAddVertex(vrVec2 v)
{
	currentMold->mold.polyMold.num_vertices += 1;
	if (currentMold->mold.polyMold.num_vertices > 1)
		currentMold->mold.polyMold.polygon = vrRealloc(currentMold->mold.polyMold.polygon, currentMold->mold.polyMold.num_vertices*sizeof(vrVec2));
	else
		currentMold->mold.polyMold.polygon = vrAlloc(sizeof(vrVec2));
	currentMold->mold.polyMold.polygon[currentMold->mold.polyMold.num_vertices - 1] = v;
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
		circle->center = shapemold->mold.circleMold.center;
		circle->radius = shapemold->mold.circleMold.radius;
		vrArrayPush(shapes, shape);
	}
	else if (shapemold->type == VR_POLYGON)
	{
		if (shapemold->mold.polyMold.polyType == VR_CONCAVE_POLYGON)
		{
			//Use a buffer, because ear clip vrFrees the arrays memory
			int size = 0;
			vrVec2* polygon_buffer = vrAlloc(sizeof(vrVec2) * shapemold->mold.polyMold.num_vertices);
			for (int i = 0; i < shapemold->mold.polyMold.num_vertices; i++)
			{
				polygon_buffer[i] = shapemold->mold.polyMold.polygon[i];
			}
			vrTriangle * tri = vrEarClip(polygon_buffer, shapemold->mold.polyMold.num_vertices, &size);

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
			for (int i = 0; i < shapemold->mold.polyMold.num_vertices; i++)
			{
				vrAddVertexToPolyShape(s->shape, shapemold->mold.polyMold.polygon[i]);
			}
			vrUpdatePolyAxes(s->shape);
			vrArrayPush(shapes, s);
		}
	}
	return shapes;
}
