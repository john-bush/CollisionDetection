#pragma once

#include <stdio.h>
#include "primitives.h"

//-----------------------------------------------------------------------------
// Triple product expansion is used to calculate perpendicular normal vectors 
// which kinda 'prefer' pointing towards the Origin in Minkowski space
Point2 tripleProduct (Point2 a, Point2 b, Point2 c);


// This is to compute average center (roughly). It might be different from
// Center of Gravity, especially for bodies with nonuniform density,
// but this is ok as initial direction of simplex search in GJK.
Point2 averagePoint (const Point2 * vertices, size_t count);

// Get furthest vertex along a certain direction
size_t indexOfFurthestPoint (const Point2 * vertices, size_t count, Point2 d);

// Minkowski sum support function for GJK
Point2 support (const Point2 * vertices1, size_t count1,
              const Point2 * vertices2, size_t count2, Point2 d);


int gjk (const Point2 * vertices1, size_t count1,
         const Point2 * vertices2, size_t count2);

float Perturbation();

Point2 Jostle(Point2 a);


