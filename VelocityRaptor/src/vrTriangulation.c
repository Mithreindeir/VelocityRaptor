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


#include "../include/vrTriangulation.h"

vrTriangle * vrEarClip(vrVec2* polygon, int num_vertices, int* num_triangles)
{
	vrTriangle* triangles = NULL;
	int* reflex = NULL;
	int reflex_size = 0;

	if (num_vertices < 3)
	{
		return NULL;
	}

	vrVec2 left = polygon[0];
	int index = 0;

	//Find farthest point
	for (int i = 0; i < num_vertices; i++)
	{
		if (polygon[i].x <= left.x && polygon[i].y < left.y)
		{
			index = i;
			left = polygon[i];
		}
	}

	//Use that to calculate farthest triangle which has to be convex
	//Because it is the farthest in a direction
	int l = (index > 0) ? index - 1 : num_vertices - 1;
	int r = (index < num_vertices) ? index + 1 : 0;
	vrTriangle triangle;
	triangle.a = polygon[l];
	triangle.b = polygon[index];
	triangle.c = polygon[r];

	//Determine the orientation
	vrBOOL countercw = vrTriangleCCW(triangle);

	if (num_vertices == 3)
	{
		vrTriangle tri;
		tri.a = polygon[0];
		tri.b = polygon[1];
		tri.c = polygon[2];
		triangles = vrAlloc(sizeof(vrTriangle));
		triangles[0] = tri;
		*num_triangles += 1;
		return triangles;
	}
	while (num_vertices >= 3)
	{
		reflex_size = 0;

		int tip = -1, index = -1;
		for (int i = 0; i < num_vertices; i++)
		{
			index++;

			if (tip >= 0) break;

			int p = (index > 0) ? index - 1 : num_vertices - 1;
			int n = (index < num_vertices) ? index + 1 : 0;

			vrTriangle tri;
			tri.a = polygon[p];
			tri.b = polygon[i];
			tri.c = polygon[n];

			if (vrTriangleCCW(tri) != countercw)
			{

				reflex = vrRealloc(reflex, sizeof(int)*(reflex_size + 1));
				reflex[reflex_size] = index;
				reflex_size += 1;

				continue;
			}
			vrBOOL ear = vrTRUE;
			for (int j = 0; j < reflex_size; j++)
			{
				if (reflex[j] == p || reflex[j] == n) continue;

				vrTriangle t;
				t.a = polygon[p];
				t.b = polygon[i];
				t.c = polygon[n];

				if (vrTrianglePoint(t, polygon[reflex[j]], vrTriangleCCW(t)))
				{
					ear = vrFALSE;
					break;
				}
			}
			if (ear)
			{

				for (int j = index + 1; j < num_vertices; j++)
				{
					vrVec2 v = polygon[j];
					if (vrVec2Equals(v, polygon[p]) ||
						vrVec2Equals(v, polygon[n]) ||
						vrVec2Equals(v, polygon[index])) continue;
					vrTriangle t;
					t.a = polygon[p];
					t.b = polygon[i];
					t.c = polygon[n];
					if (vrTrianglePoint(t, v, vrTriangleCCW(t)))
					{
						ear = vrFALSE;
						break;
					}
				}
			}

			if (ear) tip = index;
		}
		if (tip < 0) break;

		int p = (tip > 0) ? tip - 1 : num_vertices - 1;
		int n = (tip < num_vertices) ? tip + 1 : 0;

		vrTriangle t;
		t.a = polygon[p];
		t.b = polygon[tip];
		t.c = polygon[n];
		//vrVec2Log(t.a);
		//vrVec2Log(t.b);
		//vrVec2Log(t.c);
		triangles = vrRealloc(triangles, sizeof(vrTriangle)*((*num_triangles)+1));
		triangles[(*num_triangles)] = t;
		(*num_triangles) += 1;

		vrVec2* buffer = vrAlloc(sizeof(vrVec2) * (num_vertices));

		int c = 0;
		for (int i = 0; i < num_vertices; i++)
		{
			if (i != tip)
			{
				buffer[c] = polygon[i];
				c++;
			}
		}
		vrFree(polygon);
		num_vertices -= 1;

		polygon = buffer;

	}
	
	vrFree(polygon);

	return triangles;
}

vrBOOL vrTrianglePoint(vrTriangle triangle, vrVec2 point, vrBOOL ccw)
{
	
	vrVec2 det1a = vrSub(triangle.b, triangle.a);
	vrVec2 det1b = vrSub(point, triangle.a);

	vrVec2 det2a = vrSub(triangle.c, triangle.b);
	vrVec2 det2b = vrSub(point, triangle.b);
	
	vrVec2 det3a = vrSub(triangle.a, triangle.c);
	vrVec2 det3b = vrSub(point, triangle.c);

	if (!ccw)
	{
		if (vrCross(det1a, det1b) < 0 && vrCross(det2a, det2b) < 0 && vrCross(det3a, det3b) < 0)
			return vrTRUE;
	}
	else
	{
		if (vrCross(det1a, det1b) > 0 && vrCross(det2a, det2b) > 0 && vrCross(det3a, det3b) > 0)
			return vrTRUE;
	}
	return vrFALSE;
	
}

vrBOOL vrTriangleCCW(const vrTriangle t)
{
	return (t.b.x - t.a.x) * (t.c.y - t.a.y) -
		(t.c.x - t.a.x) * (t.b.y - t.a.y) > 0;
} 


