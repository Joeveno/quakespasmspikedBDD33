/*
Copyright (C) 1996-2001 Id Software, Inc.
Copyright (C) 2002-2009 John Fitzgibbons and others
Copyright (C) 2010-2014 QuakeSpasm developers

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

*/
// chase.c -- chase camera code

#include "quakedef.h"

cvar_t	chase_back = {"chase_back", "100", CVAR_NONE};
cvar_t	chase_up = {"chase_up", "16", CVAR_NONE};
cvar_t	chase_right = {"chase_right", "0", CVAR_NONE};
cvar_t	chase_active = {"chase_active", "0", CVAR_NONE};

/*
==============
Chase_Init
==============
*/
void Chase_Init (void)
{
	Cvar_RegisterVariable (&chase_back);
	Cvar_RegisterVariable (&chase_up);
	Cvar_RegisterVariable (&chase_right);
	Cvar_RegisterVariable (&chase_active);
}

/*
==============
TraceLine

TODO: impact on bmodels, monsters
==============
*/
void TraceLine (vec3_t start, vec3_t end, float pushoff, vec3_t impact)
{
	trace_t	trace;

	memset (&trace, 0, sizeof(trace));
	trace.fraction = 1;
	trace.allsolid = true;
	VectorCopy (end, trace.endpos);
	SV_RecursiveHullCheck (cl.worldmodel->hulls, start, end, &trace, CONTENTMASK_ANYSOLID);

	VectorCopy (trace.endpos, impact);

	if (pushoff && trace.fraction < 1)	//push away from the impact plane by the distance specified, so our camera's near clip plane does not intersect the wall.
	{
		vec3_t dir;
		VectorSubtract(start, end, dir);
		pushoff = pushoff / DotProduct(dir, trace.plane.normal);	//distance needs to be bigger if the trace is co-planar to the surface
		VectorMA(impact, pushoff, dir, impact);
	}
}

/*
==============
Chase_UpdateForClient -- johnfitz -- orient client based on camera. called after input
==============
*/
void Chase_UpdateForClient (void)
{
	//place camera

	//assign client angles to camera

	//see where camera points

	//adjust client angles to point at the same place
}

/*
==============
Chase_UpdateForDrawing -- johnfitz -- orient camera based on client. called before drawing

TODO: stay at least 8 units away from all walls in this leaf
==============
*/
void Chase_UpdateForDrawing (void)
{
	int		i;
	vec3_t	forward, up, right;
	vec3_t	ideal, crosshair, temp;

	AngleVectors (cl.viewangles, forward, right, up);

	// calc ideal camera location before checking for walls
	for (i=0 ; i<3 ; i++)
		ideal[i] = r_refdef.vieworg[i]
		- forward[i]*chase_back.value
		+ right[i]*chase_right.value;
		//+ up[i]*chase_up.value;
	ideal[2] = r_refdef.vieworg[2] + chase_up.value;

	// make sure camera is not in or behind a wall
	TraceLine(r_refdef.vieworg, ideal, NEARCLIP, ideal);

	// find the spot the player is looking at
	VectorMA (r_refdef.vieworg, 1<<20, forward, temp);
	TraceLine (r_refdef.vieworg, temp, 0, crosshair);

	// place camera
	VectorCopy (ideal, r_refdef.vieworg);

	// calculate camera angles to look at the same spot
	VectorSubtract (crosshair, r_refdef.vieworg, temp);
	VectorAngles (temp, NULL, r_refdef.viewangles);
	if (r_refdef.viewangles[PITCH] >= 89.9 || r_refdef.viewangles[PITCH] <= -89.9)
		r_refdef.viewangles[YAW] = cl.viewangles[YAW];
}

