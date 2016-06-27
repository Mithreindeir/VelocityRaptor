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
