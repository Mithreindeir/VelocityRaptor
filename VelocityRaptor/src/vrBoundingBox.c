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
