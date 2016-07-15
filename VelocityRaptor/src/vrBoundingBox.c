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

#include "../include/vrBoundingBox.h"

vrBOOL vrOBBOverlaps(const vrOrientedBoundingBox a, const vrOrientedBoundingBox b)
{
	if (a.position.x < (b.position.x + b.size.x) && (a.position.x + a.size.x) > b.position.x && a.position.y < (b.position.y + b.size.y) && (a.position.y + a.size.y) > b.position.y)
		return vrTRUE;
	return vrFALSE;
}

vrBOOL vrOBBContains(const vrOrientedBoundingBox a, const vrOrientedBoundingBox b)
{
	if (a.position.x < b.position.x && (a.position.x + a.size.x) >(b.position.x + b.size.x) && a.position.y < b.position.y && (a.position.y + a.size.y) >(b.position.y + b.size.y))
		return vrTRUE;
	return vrFALSE;
}

vrOrientedBoundingBox vrOBBCreate(vrVec2 pos, vrVec2 size)
{
	vrOrientedBoundingBox b;
	b.position = pos;
	b.size = size;
	return b;
}
