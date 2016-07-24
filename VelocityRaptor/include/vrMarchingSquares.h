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
#ifndef HEADER_VRMARCHINGSQUARES
#define HEADER_VRMARCHINGSQUARES

#include "vr.h"
#include "vrMath.h"

/*
Config order
 a+----+b
  |    |
  |    |
 d+----+c
 code goes as
 abcd
 eg: 
 1111 (all points are on)
 0001 (bottom left is on)
 etc.
*/
#define INDEX(row, col, width) (row * width + col)
typedef struct vrSquare
{
	vrVec2 pos;
	vrFloat size;

	int a, b, c, d;
} vrSquare;

typedef struct vrGrid
{
	vrSquare* grid;
	int width, height;
} vrGrid;

typedef vrBOOL(*vrPointInShapeFunc)(void* userdata, vrVec2 point);

typedef struct vrMarchingSquares
{
	vrGrid grid;
	vrPointInShapeFunc pointInShape;
} vrMarchingSquares;


#endif