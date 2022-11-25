#pragma once

#include <iostream>
#include <stack>
#include <stdlib.h>
#include <vector>
#include "primitives.h"
using namespace std;

// A utility function to find next to top in a stack
Point2 nextToTop(stack<Point2> &S);

// A utility function to swap two points
void swap(Point2 &p1, Point2 &p2);

// A utility function to return square of distance
// between p1 and p2
int distSq(Point2 p1, Point2 p2);

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are collinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(Point2 p, Point2 q, Point2 r);

// A function used by library function qsort() to sort an array of
// points with respect to the first Point2
int compare(const void *vp1, const void *vp2);

// Prints convex hull of a set of n points.
void convexHull(Point2 points[], int n);
