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

#ifndef HEADER_VRPARTICLESYS
#define HEADER_VRPARTICLESYS

#include "vrParticle.h"
#include "vrArray.h"
#include "vrRigidBody.h"
#include "vrCollision.h"

static const vrFloat upper_bound = 5.8;
static const vrFloat lower_bound = 0.2;

///Simple Smoothed Particle Hydrodynamics
///Particle system based of paper called
///"Particle-based Viscoelastic Fluid Simulation"
///By Simon Clavet, Philippe Beaudoin, and Pierre Poulin
typedef struct vrParticleSystem
{
	///Array holding all the particles
	vrArray* particles;

	///The resting density
	vrFloat resting_d;
	///Constant of stiffness for near pressure
	vrFloat k_stiffN;
	///Constant of stiffness for pressure
	vrFloat k_stiff;
	///Viscosity
	vrFloat viscosity;
	///Quadratic viscosity
	vrFloat quadratic_viscosity;
	///Constant for spring
	vrFloat k_spring;
	///Resting length of springs
	vrFloat restLen;
	///Gravity
	vrVec2 gravity;

	///Placeholder body for collisions
	vrRigidBody* pBody;
} vrParticleSystem;

///Allocated memory for particle system
vrParticleSystem* vrParticleSystemAlloc();
///Initializes particle system
vrParticleSystem* vrParticleSystemInit(vrParticleSystem* psys);
///Steps particle system
void vrParticleSystemStep(vrParticleSystem* system, vrFloat dt);
///Makes sure particles don't go out of bounds
void vrParticleSystemBoundaries(vrParticleSystem* system);
///Collides system with body
void vrParticleSystemCollide(vrParticleSystem* system, vrRigidBody* body, vrFloat scale, vrFloat dt);
///Calculates viscous impulses
void vrParticleSystemViscousity(vrParticleSystem* system, vrFloat dt);
///Calculates double density
void vrParticleSystemDoubleDensity(vrParticleSystem* system, vrFloat dt);

#endif