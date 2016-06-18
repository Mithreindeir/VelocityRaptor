#ifndef HEADER_VELOCITYRAPTOR
#define HEADER_VELOCITYRAPTOR

#include <stdlib.h>
#include <stdio.h>

#ifdef _MSC_VER
#include "malloc.h"
#else
#include <alloca.h>
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
#define vrFree free
#endif

#endif

