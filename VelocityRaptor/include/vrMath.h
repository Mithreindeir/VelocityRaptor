#ifndef HEADER_VRMATH
#define HEADER_VRMATH
#include <math.h>

typedef float vrFloat;

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

vrMat2 vrMat(const vrVec2 m, const vrVec2 n);
vrMat2 vrMat2Scale(const vrMat2 a, const vrFloat b);
vrVec2 vrMat2Mult( const vrMat2 a, const vrVec2 b);
vrVec2 vrVect(vrFloat x, vrFloat y);
vrVec2 vrAdd(const vrVec2 a, const vrVec2 b);
vrVec2 vrSub(const vrVec2 a, const vrVec2 b);
vrVec2 vrMult(const vrVec2 a, const vrVec2 b);
vrVec2 vrDiv(const vrVec2 a, const vrVec2 b);
vrFloat vrDot(const vrVec2 a, const vrVec2 b);
vrFloat vrCross(const vrVec2 a, const vrVec2 b);
vrVec2 vrCrossScalar(const vrFloat a, const vrVec2 b);
vrVec2 vrScale(const vrVec2 a, const vrFloat scale);
vrFloat vrMax(const vrFloat a, const vrFloat b);
vrFloat vrMin(const vrFloat a, const vrFloat b);
vrFloat vrDist(const vrVec2 a, const vrVec2 b);
vrFloat vrDist_Sqr(const vrVec2 a, const vrVec2 b);
vrFloat vrLength(const vrVec2 a);

vrVec2 vrNormalize(const vrVec2 a);
vrFloat vrClamp(vrFloat a, vrFloat low, vrFloat high);

#define VR_SQRT sqrt
#define VR_POW pow
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