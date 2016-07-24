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

#ifndef HEADER_VRCOLOR
#define HEADER_VRCOLOR

#include "vr.h"
#include "vrMath.h"

///RGB Color struct normalized(values from 0 to 1)
typedef struct vrColor
{
	///Red
	vrFloat r;
	///Green
	vrFloat g;
	///Blue
	vrFloat b;
} vrColor;
///Create color with 0-255 values
vrColor vrColorCreateNormalized(vrFloat r, vrFloat g, vrFloat b);
///Creates a color from rgb values
vrColor vrColorCreate(vrFloat r, vrFloat g, vrFloat b);
///Maps color from 0-255 to 0-1
vrColor vrColorNormalize(vrColor color);

#endif