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

#ifndef HEADER_VRMATH
#define HEADER_VRMATH
#include <math.h>
#include "velocityraptor.h"

typedef float vrFloat;

#define NUM_PRIMES 26
static unsigned int primes[] = 
{
	53,
	97,
	193,
	389,
	769,
	1543,
	3079,
	6151,
	12289,
	24593,
	49157,
	98317,
	196613,
	393241,
	786433,
	1572869,
	3145739,
	6291469,
	12582917,
	25165843,
	50331653,
	100663319,
	201326611,
	402653189,
	805306457,
	1610612741
};
typedef struct vrVec2
{
	vrFloat x;
	vrFloat y;
} vrVec2;
typedef struct vrMat2
{
	vrVec2 m;
	vrVec2 n;
} vrMat2;

extern inline vrMat2 vrMat(const vrVec2 m, const vrVec2 n);
extern inline vrMat2 vrMat2Scale(const vrMat2 a, const vrFloat b);
extern inline vrVec2 vrMat2Mult( const vrMat2 a, const vrVec2 b);
extern inline vrVec2 vrVect(vrFloat x, vrFloat y);
extern inline vrVec2 vrAdd(const vrVec2 a, const vrVec2 b);
extern inline vrVec2 vrSub(const vrVec2 a, const vrVec2 b);
extern inline vrVec2 vrMult(const vrVec2 a, const vrVec2 b);
extern inline vrVec2 vrDiv(const vrVec2 a, const vrVec2 b);
extern inline vrFloat vrDot(const vrVec2 a, const vrVec2 b);
extern inline vrFloat vrCross(const vrVec2 a, const vrVec2 b);
extern inline vrVec2 vrCrossScalar(const vrFloat a, const vrVec2 b);
extern inline vrVec2 vrScale(const vrVec2 a, const vrFloat scale);
extern inline vrFloat vrMax(const vrFloat a, const vrFloat b);
extern inline vrFloat vrMin(const vrFloat a, const vrFloat b);
extern inline vrFloat vrDist(const vrVec2 a, const vrVec2 b);
extern inline vrFloat vrDist_Sqr(const vrVec2 a, const vrVec2 b);
extern inline vrFloat vrLength(const vrVec2 a);
extern inline vrFloat vrLengthSqr(const vrVec2 a);
void vrVec2Log(const vrVec2 v);

extern inline vrVec2 vrNormalize(const vrVec2 a);
extern inline vrFloat vrClamp(vrFloat a, vrFloat low, vrFloat high);

#define VR_ABS abs
#define VR_SQRT sqrt
#define VR_POW pow
#define VR_POWF powf
#define VR_COSINE cos
#define VR_SINE sin
#define VR_TANGENT tan
#define VR_ARCTANGENT atan
#define VR_ARCCOSINE acos
#define VR_ARCSINE asin
#define VR_ARCTANGENT2 atan2
#define VR_MAX vrMax
#define VR_MIN vrMin
#define VR_PI (vrFloat)(3.14159265358979)

#endif
