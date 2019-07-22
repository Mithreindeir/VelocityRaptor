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
#ifndef HEADER_VR
#define HEADER_VR

#include <stdlib.h>
#include <stdio.h>
#define DEBUG_DRAW_SHAPE 0
#define DEBUG_DRAW_CONTACTS 0

#if !defined(alloca)
#if defined(__GLIBC__) || defined(__sun) || defined(__CYGWIN__) || defined(__APPLE__) || defined(__SWITCH__)
#include <alloca.h>     // alloca (glibc uses <alloca.h>. Note that Cygwin may have _WIN32 defined, so the order matters here)
#elif defined(_WIN32)
#include <malloc.h>     // alloca
#if !defined(alloca)
#define alloca _alloca  // for clang with MS Codegen
#endif
#else
#include <stdlib.h>     // alloca
#endif
#endif

#ifdef _MSC_VER
#define inline __inline
#endif

#ifndef vrBOOL
#define vrBOOL int
	#ifndef vrTRUE
		#define vrTRUE 1
	#endif
	#ifndef vrFALSE
		#define vrFALSE 0
	#endif
#endif

#ifndef VR_ASSERT
#define VR_ASSERT(_condition_, msg) if(!_condition_) { printf(msg); abort(); }
#endif

#ifndef vrAlloc
#define vrAlloc malloc
#endif
#ifndef vrFree
#define vrFree(a) free(a); a = NULL;
#endif
#ifndef vrCalloc
#define vrCalloc calloc
#endif
#ifndef vrRealloc
#define vrRealloc realloc
#endif

#ifndef COMBINE_INTS
//Credit to Chipmunk2D for this hashing macro
#define COMBINE_INTS(a, b) (((unsigned int)(a)*(3344921057ul) ^ (unsigned int)(b)*(3344921057ul)))
#endif

#endif
