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

#ifndef HEADER_VRRAYCAST
#define HEADER_VRRAYCAST
#include "vrMath.h"
#include "vrRigidBody.h"
#include "vrWorld.h"

typedef vrBOOL(*vrRaycastExcludeFunc)(vrRigidBody* body);

typedef struct vrRaycastInput
{
	vrVec2 start, end;
	vrRaycastExcludeFunc excludeFunc;
} vrRaycastInput;

typedef struct vrRaycastOutput
{
	vrBOOL intersection;
	vrVec2 point;
	vrVec2 normal;
	vrFloat distance;
	vrRigidBody* body;
} vrRaycastOutput;

vrRaycastOutput vrRaycast(vrWorld* world, vrRaycastInput input);

#endif