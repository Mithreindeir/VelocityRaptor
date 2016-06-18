#pragma once
#include "../include/vrMath.h"

vrMat2 vrMat(const vrVec2 m, const vrVec2 n)
{
	vrMat2 A;
	A.m = m;
	A.n = n;
	return A;
}

vrMat2 vrMat2Scale(const vrMat2 a, const vrFloat b)
{
	return vrMat(vrScale(a.m, b), vrScale(a.m, b));
}

vrVec2 vrMat2Mult(const vrMat2 a, const vrVec2 b)
{
	return vrVect(a.m.x * b.x + a.m.y * b.y, a.n.x * b.x + a.n.y * b.y);
}

vrVec2 vrVect(vrFloat x, vrFloat y)
{
	vrVec2 v;
	v.x = x;
	v.y = y;
	return v;
}

vrVec2 vrAdd(const vrVec2 a, const vrVec2 b)
{
	return vrVect(a.x + b.x, a.y + b.y);
}

vrVec2 vrSub(const vrVec2 a, const vrVec2 b)
{
	return vrVect(a.x - b.x, a.y - b.y);
}

vrVec2 vrMult(const vrVec2 a, const vrVec2 b)
{
	return vrVect(a.x * b.x, a.y * b.y);
}

vrVec2 vrDiv(const vrVec2 a, const vrVec2 b)
{
	vrVec2 v = vrVect(a.x / b.x, a.y / b.y);
	if (b.x == 0) v.x = 0;
	if (b.y == 0) v.y = 0;
	return v;
}

vrFloat vrDot(const vrVec2 a, const vrVec2 b)
{
	return (a.x * b.x + a.y * b.y);
}

vrFloat vrCross(const vrVec2 a, const vrVec2 b)
{
	return a.x*b.y - a.y*b.x;
}

vrVec2 vrCrossScalar(const vrFloat a, const vrVec2 b)
{
	return vrVect(-a*b.y, a*b.x);
}

vrVec2 vrScale(const vrVec2 a, const vrFloat scale)
{
	return vrVect(a.x * scale, a.y * scale);
}

vrFloat vrMax(const vrFloat a, const vrFloat b)
{
	return a > b ? a : b;
}

vrFloat vrMin(const vrFloat a, const vrFloat b)
{
	return a < b ? a : b;
}

vrFloat vrDist(const vrVec2 a, const vrVec2 b)
{
	return VR_SQRT(VR_POW(a.x - b.x, 2) + VR_POW(a.y - b.y, 2));
}

vrFloat vrDist_Sqr(const vrVec2 a, const vrVec2 b)
{
	return (VR_POW(a.x - b.x, 2) + VR_POW(a.y - b.y, 2));
}

vrFloat vrLength(const vrVec2 a)
{
	return VR_SQRT(a.x*a.x + a.y*a.y);
}

vrVec2 vrNormalize(const vrVec2 a)
{
	vrFloat len = vrDist(vrVect(0, 0), a);
	return vrVect(a.x / len, a.y / len);
}

vrFloat vrClamp(vrFloat a, vrFloat low, vrFloat high)
{
	return VR_MAX(low, VR_MIN(a, high));
}