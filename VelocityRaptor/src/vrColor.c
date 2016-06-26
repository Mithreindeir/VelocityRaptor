#include "..\include\vrColor.h"

vrColor vrColorCreateNormalized(vrFloat r, vrFloat g, vrFloat b)
{
	return vrColorNormalize(vrColorCreate(r, g, b));
}

vrColor vrColorCreate(vrFloat r, vrFloat g, vrFloat b)
{
	vrColor color;
	color.r = r;
	color.g = g;
	color.b = b;
	return color;
}

vrColor vrColorNormalize(vrColor color)
{
	color.r /= 255.0;
	color.g /= 255.0;
	color.b /= 255.0;
	return color;
}
