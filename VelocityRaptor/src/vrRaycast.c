/*
* Copyright (c) 2006-2009 Cormac Grindall (Mithreindeir)
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

#include "../include/vrRaycast.h"

vrRaycastOutput vrRaycast(vrWorld * world, vrRaycastInput input)
{
	vrRaycastOutput output;
	output.body = NULL;
	output.intersection = vrFALSE;
	output.point = vrVect(0, 0);
	output.normal = vrVect(0, 0);
	output.distance = 0;

	vrVec2 lineNorm = vrSub(input.end, input.start);
	vrFloat lineLen = vrLength(lineNorm);
	lineNorm = vrScale(lineNorm, 1.0 / lineLen);
	vrFloat min_len;
	vrVec2 least_norm;
	vrVec2 least_point;
	vrBOOL found = vrFALSE;

	for (int i = 0; i < world->bodies->sizeof_active; i++)
	{
		vrRigidBody* body = world->bodies->data[i];
		if (input.excludeFunc)
		{
			if (input.excludeFunc(body))
			{
				for (int j = 0; j < body->shape->sizeof_active; j++)
				{
					vrShape* shape = body->shape->data[j];
					if (shape->shapeType == VR_CIRCLE)
					{
						vrCircleShape* circle = shape->shape;
						vrVec2 E = input.start;
						vrVec2 L = input.end;
						vrVec2 C = circle->center;
						vrFloat r = circle->radius;
						vrVec2 d = lineNorm;
						vrVec2 f = vrSub(E, C);
						

						vrFloat a = vrDot(d, d);
						vrFloat b = 2.0 * vrDot(f, d);
						vrFloat c = vrDot(f, f) - r*r;
						vrFloat disc = b*b - 4 * a*c;
						if (disc < 0)
						{
							continue;
						}
						else
						{
							disc = sqrt(disc);
							vrFloat t1 = (-b - disc) / (2.0 * a);
							vrFloat t2 = (-b + disc) / (2.0 * a);

							if (t1 >= 0 && t1 <= 1)
							{
								vrVec2 p = vrAdd(E, vrScale(d, t1));
								if (t1 < min_len || !found)
								{
									min_len = t1;
									found = vrTRUE;
									vrVec2 n = vrNormalize(vrSub(p, C));
									least_norm = vrVect(n.y, -n.x);
									least_point = p;
									output.body = body;

								}
							}
							else if (t2 >= 0 && t2 <= 1)
							{
								vrVec2 p = vrAdd(E, vrScale(d, t1));
								if (t2 < min_len || !found)
								{
									min_len = t2;
									found = vrTRUE;
									vrVec2 n = vrNormalize(vrSub(p, C));
									least_norm = vrVect(n.y, -n.x);
									least_point = p;
									output.body = body;

								}
							}
						}
					}
					else
					{
						vrPolygonShape* polygon = shape->shape;
						for (int i = 0; i < polygon->num_vertices; i++)
						{
							int next = i < polygon->num_vertices - 1 ? i + 1 : 0;
							//vrVec2 norm = vrNormalize(vrSub(polygon->vertices[next], polygon->vertices[i]));
							vrVec2 s1, s2;
							vrVec2 p0, p1, p2, p3;
							p0 = input.start;
							p1 = input.end;
							p2 = polygon->vertices[i];
							p3 = polygon->vertices[next];
							s1 = vrSub(p1, p0);
							s2 = vrSub(p3, p2);

							vrFloat s, t;
							s = (-s1.y * (p0.x - p2.x) + s1.x * (p0.y - p2.y)) / (-s2.x * s1.y + s1.x * s2.y);
							t = (s2.x * (p0.y - p2.y) - s2.y * (p0.x - p2.x)) / (-s2.x * s1.y + s1.x * s2.y);

							if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
							{
								vrVec2 point = vrAdd(p0, vrScale(s1, t));
								if (t < min_len || !found)
								{
									min_len = t;
									found = vrTRUE;
									vrVec2 n = vrNormalize(s2);
									least_norm = vrVect(n.y, -n.x);
									least_point = point;
									output.body = body;
								}
							}
							else
							{
								continue;
							}
						}
					}
				}

			}
		}
	}
	output.intersection = found;
	output.normal = least_norm;
	output.point = least_point;
	output.distance = min_len;
	return output;
}
