/*
Copyright (C) 1996-1997 Id Software, Inc.

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
#include "quakedef.h"
#include "pmove.h"

static qboolean PM_TransformedHullCheck (qmodel_t *model, vec3_t start, vec3_t end, vec3_t mins, vec3_t maxs, trace_t *trace, vec3_t origin, vec3_t angles);
static	hull_t		box_hull;
static	mclipnode_t	box_clipnodes[6];
static	mplane_t	box_planes[6];

//axis MUST come from VectorAngles, hence the negation. it being normalized means its also safe to use the transpose(MA-vs-dot) as an inverse to put stuff back into the original coord system. Origin is not handled so subtract/add that before/after.
#define QAxisTransform(a, v, c)	\
	do{	c[0] = DotProduct(a[0], v);\
		c[1] =-DotProduct(a[1], v);\
		c[2] = DotProduct(a[2], v);}while(0)
#define QAxisDeTransform(a, v, c)	\
	do{	VectorScale(a[0], v[0], c);\
		VectorMA(c, -v[1], a[1], c);\
		VectorMA(c, v[2], a[2], c);}while(0)

/*
===================
PM_InitBoxHull

Set up the planes and clipnodes so that the six floats of a bounding box
can just be stored out and get a proper hull_t structure.
===================
*/
void PM_InitBoxHull (void)
{
	int		i;
	int		side;

	box_hull.clipnodes = box_clipnodes;
	box_hull.planes = box_planes;
	box_hull.firstclipnode = 0;
	box_hull.lastclipnode = 5;

	for (i=0 ; i<6 ; i++)
	{
		box_clipnodes[i].planenum = i;

		side = i&1;

		box_clipnodes[i].children[side] = CONTENTS_EMPTY;
		if (i != 5)
			box_clipnodes[i].children[side^1] = i + 1;
		else
			box_clipnodes[i].children[side^1] = CONTENTS_SOLID;

		box_planes[i].type = i>>1;
		box_planes[i].normal[i>>1] = 1;
	}

}


/*
===================
PM_HullForBox

To keep everything totally uniform, bounding boxes are turned into small
BSP trees instead of being compared directly.
===================
*/
static hull_t	*PM_HullForBox (vec3_t mins, vec3_t maxs)
{
	box_planes[0].dist = maxs[0];
	box_planes[1].dist = mins[0];
	box_planes[2].dist = maxs[1];
	box_planes[3].dist = mins[1];
	box_planes[4].dist = maxs[2];
	box_planes[5].dist = mins[2];

	return &box_hull;
}


static unsigned int PM_TransformedModelPointContents (qmodel_t *mod, vec3_t p, vec3_t origin, vec3_t angles)
{
	vec3_t p_l, axis[3], p_t;
	VectorSubtract (p, origin, p_l);

	if (mod->type != mod_brush)
		return CONTENTMASK_FROMQ1(CONTENTS_EMPTY);

	// rotate start and end into the models frame of reference
	if (angles[0] || angles[1] || angles[2])
	{
		AngleVectors (angles, axis[0], axis[1], axis[2]);
		QAxisTransform(axis, p_l, p_t);
		return CONTENTMASK_FROMQ1(SV_HullPointContents(&mod->hulls[0], mod->hulls[0].firstclipnode, p_t));
	}

	return CONTENTMASK_FROMQ1(SV_HullPointContents(&mod->hulls[0], mod->hulls[0].firstclipnode, p_l));
}


/*
==================
PM_PointContents

==================
*/
int PM_PointContents (vec3_t p)
{
	int			num;

	unsigned int pc;
	physent_t *pe;
	qmodel_t *pm;

	//check world.
	pm = pmove.physents[0].model;
	if (!pm || pm->needload)
		return CONTENTBIT_EMPTY;
	pc = CONTENTMASK_FROMQ1(SV_PointContents(p));

	//we need this for e2m2 - waterjumping on to plats wouldn't work otherwise.
	for (num = 1; num < pmove.numphysent; num++)
	{
		pe = &pmove.physents[num];

		if (pe->info == pmove.skipent)
			continue;

		pm = pe->model;
		if (pm)
		{
			if (p[0] >= pe->origin[0]+pm->mins[0] && p[0] <= pe->origin[0]+pm->maxs[0] &&
				p[1] >= pe->origin[1]+pm->mins[1] && p[1] <= pe->origin[1]+pm->maxs[1] &&
				p[2] >= pe->origin[2]+pm->mins[2] && p[2] <= pe->origin[2]+pm->maxs[2])
			{
				if (pe->forcecontentsmask)
				{
					if (PM_TransformedModelPointContents(pm, p, pe->origin, pe->angles) != CONTENTBIT_EMPTY)
						pc |= pe->forcecontentsmask;
				}
				else
					pc |= PM_TransformedModelPointContents(pm, p, pe->origin, pe->angles);
			}
		}
		else if (pe->forcecontentsmask)
		{
			if (p[0] >= pe->origin[0]+pe->mins[0] && p[0] <= pe->origin[0]+pe->maxs[0] &&
				p[1] >= pe->origin[1]+pe->mins[1] && p[1] <= pe->origin[1]+pe->maxs[1] &&
				p[2] >= pe->origin[2]+pe->mins[2] && p[2] <= pe->origin[2]+pe->maxs[2])
				pc |= pe->forcecontentsmask;
		}
	}

	return pc;
}

int PM_ExtraBoxContents (vec3_t p)
{
	int			num;

	int pc = 0;
	physent_t *pe;
	qmodel_t *pm;
	trace_t tr;

	for (num = 1; num < pmove.numphysent; num++)
	{
		pe = &pmove.physents[num];
		pm = pe->model;
		if (pm)
		{
			if (pe->forcecontentsmask)
			{
				if (!PM_TransformedHullCheck(pm, p, p, pmove.player_mins, pmove.player_maxs, &tr, pe->origin, pe->angles))
					continue;
				if (tr.startsolid || tr.inwater)
					pc |= pe->forcecontentsmask;
			}
		}
		else if (pe->forcecontentsmask)
		{
			if (p[0]+pmove.player_maxs[0] >= pe->origin[0]+pe->mins[0] && p[0]+pmove.player_mins[0] <= pe->origin[0]+pe->maxs[0] &&
				p[1]+pmove.player_maxs[1] >= pe->origin[1]+pe->mins[1] && p[1]+pmove.player_mins[1] <= pe->origin[1]+pe->maxs[1] &&
				p[2]+pmove.player_maxs[2] >= pe->origin[2]+pe->mins[2] && p[2]+pmove.player_mins[2] <= pe->origin[2]+pe->maxs[2])
				pc |= pe->forcecontentsmask;
		}
	}

	return pc;
}

/*
===============================================================================

LINE TESTING IN HULLS

===============================================================================
*/

/*returns if it actually did a trace*/
static qboolean PM_TransformedHullCheck (qmodel_t *model, vec3_t start, vec3_t end, vec3_t player_mins, vec3_t player_maxs, trace_t *trace, vec3_t origin, vec3_t angles)
{
	vec3_t		start_l, end_l;
	int i;
	vec3_t		axis[3], start_t, end_t;

	// subtract origin offset
	VectorSubtract (start, origin, start_l);
	VectorSubtract (end, origin, end_l);

	// sweep the box through the model
	if (model && model->type == mod_brush)
	{
		hull_t *hull = &model->hulls[(player_maxs[0]-player_mins[0] < 3)?0:1];
		if (angles[0] || angles[1] || angles[2])
		{
			AngleVectors (angles, axis[0], axis[1], axis[2]);
			QAxisTransform(axis, start_l, start_t);
			QAxisTransform(axis, end_l, end_t);
			SV_RecursiveHullCheck(hull, start_t, end_t, trace, MASK_PLAYERSOLID);
			VectorCopy(trace->plane.normal, end_t);	QAxisDeTransform(axis, end_t, trace->plane.normal);
			VectorCopy(trace->endpos, end_t);		QAxisDeTransform(axis, end_t, trace->endpos);
		}
		else
		{
			for (i = 0; i < 3; i++)
			{
				if (start_l[i]+player_mins[i] > model->maxs[i] && end_l[i] + player_mins[i] > model->maxs[i])
					return false;
				if (start_l[i]+player_maxs[i] < model->mins[i] && end_l[i] + player_maxs[i] < model->mins[i])
					return false;
			}

			memset (trace, 0, sizeof(trace_t));
			trace->fraction = 1;
			trace->allsolid = true;
			SV_RecursiveHullCheck(hull, start_l, end_l, trace, MASK_PLAYERSOLID);
		}
	}
	else
	{
		for (i = 0; i < 3; i++)
		{
			if (start_l[i]+player_mins[i] > box_planes[0+i*2].dist && end_l[i] + player_mins[i] > box_planes[0+i*2].dist)
				return false;
			if (start_l[i]+player_maxs[i] < box_planes[1+i*2].dist && end_l[i] + player_maxs[i] < box_planes[1+i*2].dist)
				return false;
		}

		memset (trace, 0, sizeof(trace_t));
		trace->fraction = 1;
		trace->allsolid = true;
		SV_RecursiveHullCheck (&box_hull, start_l, end_l, trace, MASK_PLAYERSOLID);
	}

	trace->endpos[0] += origin[0];
	trace->endpos[1] += origin[1];
	trace->endpos[2] += origin[2];
	return true;
}

/*
================
PM_TestPlayerPosition

Returns false if the given player position is not valid (in solid)
================
*/
qboolean PM_TestPlayerPosition (vec3_t pos)
{
	int			i;
	physent_t	*pe;
	vec3_t		mins, maxs;
	hull_t		*hull;
	trace_t		trace;

	for (i=0 ; i< pmove.numphysent ; i++)
	{
		pe = &pmove.physents[i];

		if (pe->info == pmove.skipent)
			continue;

		if (pe->forcecontentsmask && !(pe->forcecontentsmask & MASK_PLAYERSOLID))
			continue;

	// get the clipping hull
		if (pe->model)
		{
			if (!PM_TransformedHullCheck (pe->model, pos, pos, pmove.player_mins, pmove.player_maxs, &trace, pe->origin, pe->angles))
				continue;
			if (trace.allsolid)
				return false;
		}
		else
		{
			VectorSubtract (pe->mins, pmove.player_maxs, mins);
			VectorSubtract (pe->maxs, pmove.player_mins, maxs);
			hull = PM_HullForBox (mins, maxs);
			VectorSubtract(pos, pe->origin, mins);

			if (CONTENTMASK_FROMQ1(SV_HullPointContents(hull, hull->firstclipnode, mins)) & MASK_PLAYERSOLID)
				return false;
		}
	}

	pmove.safeorigin_known = true;
	VectorCopy (pmove.origin, pmove.safeorigin);

	return true;
}

/*
================
PM_PlayerTrace
================
*/
trace_t PM_PlayerTrace (vec3_t start, vec3_t end, unsigned int solidmask)
{
	trace_t		trace, total;
	int			i;
	physent_t	*pe;

// fill in a default trace
	memset (&total, 0, sizeof(trace_t));
	total.fraction = 1;
	total.entnum = -1;
	VectorCopy (end, total.endpos);

	for (i=0 ; i< pmove.numphysent ; i++)
	{
		pe = &pmove.physents[i];

		if (pe->info == pmove.skipent)
			continue;
		if (pe->forcecontentsmask && !(pe->forcecontentsmask & solidmask))
			continue;

		if (!pe->model || pe->model->needload)
		{
			vec3_t mins, maxs;

			VectorSubtract (pe->mins, pmove.player_maxs, mins);
			VectorSubtract (pe->maxs, pmove.player_mins, maxs);
			PM_HullForBox (mins, maxs);

			// trace a line through the apropriate clipping hull
			if (!PM_TransformedHullCheck (NULL, start, end, pmove.player_mins, pmove.player_maxs, &trace, pe->origin, pe->angles))
				continue;
		}
		else
		{
			// trace a line through the apropriate clipping hull
			if (!PM_TransformedHullCheck (pe->model, start, end, pmove.player_mins, pmove.player_maxs, &trace, pe->origin, pe->angles))
				continue;
		}

		if (trace.allsolid)
			trace.startsolid = true;
//		if (trace.startsolid)
//			trace.fraction = 0;

	// did we clip the move?
		if (trace.fraction < total.fraction || (trace.startsolid && !total.startsolid))
		{
			// fix trace up by the offset
			total = trace;
			total.entnum = i;
		}
	}

//	//this is needed to avoid *2 friction. some id bug.
	if (total.startsolid)
		total.fraction = 0;
	return total;
}

//for use outside the pmove code. lame, but works.
trace_t PM_TraceLine (vec3_t start, vec3_t end)
{
	VectorClear(pmove.player_mins);
	VectorClear(pmove.player_maxs);
	return PM_PlayerTrace(start, end, MASK_PLAYERSOLID);
}
