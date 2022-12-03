#include <stdio.h>
#include <iostream>
#include <stack>
#include <stdlib.h>
#include <vector>
#include <time.h>

#define NUM_VERTICES 100


struct Point2
{
	float x, y;
    Point2() {
        x = 0;
        y = 0;
    }
    Point2(float x, float y) {
        
    }
};

//-----------------------------------------------------------------------------
// Gilbert-Johnson-Keerthi (GJK) collision detection algorithm in 2D
// http://www.dyn4j.org/2010/04/gjk-gilbert-johnson-keerthi/
// http://mollyrocket.com/849
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Basic vector arithmetic operations

Point2 subtract (Point2 a, Point2 b) { a.x -= b.x; a.y -= b.y; return a; }
Point2 negate (Point2 v) { v.x = -v.x; v.y = -v.y; return v; }
Point2 perpendicular (Point2 v) { Point2 p = { v.y, -v.x }; return p; }
float dotProduct (Point2 a, Point2 b) { return a.x * b.x + a.y * b.y; }
float lengthSquared (Point2 v) { return v.x * v.x + v.y * v.y; }

//-----------------------------------------------------------------------------
// Triple product expansion is used to calculate perpendicular normal vectors 
// which kinda 'prefer' pointing towards the Origin in Minkowski space

Point2 tripleProduct (Point2 a, Point2 b, Point2 c) {
    
    Point2 r;
    
    float ac = a.x * c.x + a.y * c.y; // perform a.dot(c)
    float bc = b.x * c.x + b.y * c.y; // perform b.dot(c)
    
    // perform b * a.dot(c) - a * b.dot(c)
    r.x = b.x * ac - a.x * bc;
    r.y = b.y * ac - a.y * bc;
    return r;
}

//-----------------------------------------------------------------------------
// This is to compute average center (roughly). It might be different from
// Center of Gravity, especially for bodies with nonuniform density,
// but this is ok as initial direction of simplex search in GJK.

Point2 averagePoint (const Point2 * vertices, size_t count) {
    Point2 avg = { 0.f, 0.f };
    for (size_t i = 0; i < count; i++) {
        avg.x += vertices[i].x;
        avg.y += vertices[i].y;
    }
    avg.x /= count;
    avg.y /= count;
    return avg;
}

//-----------------------------------------------------------------------------
// Get furthest vertex along a certain direction

size_t indexOfFurthestPoint (const Point2 * vertices, size_t count, Point2 d) {
    
    float maxProduct = dotProduct (d, vertices[0]);
    size_t index = 0;
    for (size_t i = 1; i < count; i++) {
        float product = dotProduct (d, vertices[i]);
        if (product > maxProduct) {
            maxProduct = product;
            index = i;
        }
    }
    return index;
}

//-----------------------------------------------------------------------------
// Minkowski sum support function for GJK

Point2 support (const Point2 * vertices1, size_t count1,
              const Point2 * vertices2, size_t count2, Point2 d) {

    // get furthest point of first body along an arbitrary direction
    size_t i = indexOfFurthestPoint (vertices1, count1, d);
    
    // get furthest point of second body along the opposite direction
    size_t j = indexOfFurthestPoint (vertices2, count2, negate (d));

    // subtract (Minkowski sum) the two points to see if bodies 'overlap'
    return subtract (vertices1[i], vertices2[j]);
}

//-----------------------------------------------------------------------------
// The GJK yes/no test



int gjk (const Point2 * vertices1, size_t count1,
         const Point2 * vertices2, size_t count2) {
    
    int iter_count = 0;

    size_t index = 0; // index of current vertex of simplex
    Point2 a, b, c, d, ao, ab, ac, abperp, acperp, simplex[3];
    
    Point2 position1 = averagePoint (vertices1, count1); // not a CoG but
    Point2 position2 = averagePoint (vertices2, count2); // it's ok for GJK )

    // initial direction from the center of 1st body to the center of 2nd body
    d = subtract (position1, position2);
    
    // if initial direction is zero – set it to any arbitrary axis (we choose X)
    if ((d.x == 0) && (d.y == 0))
        d.x = 1.f;
    
    // set the first support as initial point of the new simplex
    a = simplex[0] = support (vertices1, count1, vertices2, count2, d);
    
    if (dotProduct (a, d) <= 0)
        return 0; // no collision
    
    d = negate (a); // The next search direction is always towards the origin, so the next search direction is negate(a)
    
    while (iter_count < 100) {
        iter_count++;
        
        a = simplex[++index] = support (vertices1, count1, vertices2, count2, d);
        
        if (dotProduct (a, d) <= 0)
            return 0; // no collision
        
        ao = negate (a); // from point A to Origin is just negative A
        
        // simplex has 2 points (a line segment, not a triangle yet)
        if (index < 2) {
            b = simplex[0];
            ab = subtract (b, a); // from point A to B
            d = tripleProduct (ab, ao, ab); // normal to AB towards Origin
            if (lengthSquared (d) == 0)
                d = perpendicular (ab);
            continue; // skip to next iteration
        }
        
        b = simplex[1];
        c = simplex[0];
        ab = subtract (b, a); // from point A to B
        ac = subtract (c, a); // from point A to C
        
        acperp = tripleProduct (ab, ac, ac);
        
        if (dotProduct (acperp, ao) >= 0) {
            
            d = acperp; // new direction is normal to AC towards Origin
            
        } else {
            
            abperp = tripleProduct (ac, ab, ab);
            
            if (dotProduct (abperp, ao) < 0)
                return 1; // collision
            
            simplex[0] = simplex[1]; // swap first element (point C)

            d = abperp; // new direction is normal to AB towards Origin
        }
        
        simplex[1] = simplex[2]; // swap element in the middle (point B)
        --index;
    }
    
    return 0;
}

//-----------------------------------------------------------------------------

#include <stdlib.h>
#include <float.h>

float Perturbation()
{
	return ((float)rand() / (float)RAND_MAX) * FLT_EPSILON * 100.0f * ((rand() % 2) ? 1.0f : -1.0f);
}

Point2 Jostle(Point2 a)
{
	Point2 b;
	b.x = a.x + Perturbation();
	b.y = a.y + Perturbation();
	return b;
}

using namespace std;

// A global Point2 needed for sorting points with reference
// to the first Point2 Used in compare function of qsort()
Point2 p0;

// A utility function to find next to top in a stack
Point2 nextToTop(stack<Point2> &S)
{
	Point2 p = S.top();
	S.pop();
	Point2 res = S.top();
	S.push(p);
	return res;
}

// A utility function to swap two points
void swap(Point2 &p1, Point2 &p2)
{
	Point2 temp = p1;
	p1 = p2;
	p2 = temp;
}

// A utility function to return square of distance
// between p1 and p2
int distSq(Point2 p1, Point2 p2)
{
	return (p1.x - p2.x)*(p1.x - p2.x) +
		(p1.y - p2.y)*(p1.y - p2.y);
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are collinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(Point2 p, Point2 q, Point2 r)
{
	int val = (q.y - p.y) * (r.x - q.x) -
			(q.x - p.x) * (r.y - q.y);

	if (val == 0) return 0; // collinear
	return (val > 0)? 1: 2; // clock or counterclock wise
}

// A function used by library function qsort() to sort an array of
// points with respect to the first Point2
int compare(const void *vp1, const void *vp2)
{
	Point2 *p1 = (Point2 *)vp1;
	Point2 *p2 = (Point2 *)vp2;

	// Find orientation
	int o = orientation(p0, *p1, *p2);
	if (o == 0)
		return (distSq(p0, *p2) >= distSq(p0, *p1))? -1 : 1;

	return (o == 2)? -1: 1;
}

// Prints convex hull of a set of n points.
void convexHull(Point2 *points)
{
    int n = NUM_VERTICES;
	// Find the bottommost Point2
	int ymin = points[0].y, min = 0;
	for (int i = 1; i < n; i++)
	{
		int y = points[i].y;

		// Pick the bottom-most or choose the left
		// most Point2 in case of tie
		if ((y < ymin) || (ymin == y &&
			points[i].x < points[min].x))
			ymin = points[i].y, min = i;
	}

	// Place the bottom-most Point2 at first position
	swap(points[0], points[min]);

	// Sort n-1 points with respect to the first Point2.
	// A Point2 p1 comes before p2 in sorted output if p2
	// has larger polar angle (in counterclockwise
	// direction) than p1
	p0 = points[0];
	qsort(&points[1], n-1, sizeof(Point2), compare);

	// If two or more points make same angle with p0,
	// Remove all but the one that is farthest from p0
	// Remember that, in above sorting, our criteria was
	// to keep the farthest Point2 at the end when more than
	// one points have same angle.
	int m = 1; // Initialize size of modified array
	for (int i=1; i<n; i++)
	{
		// Keep removing i while angle of i and i+1 is same
		// with respect to p0
		while (i < n-1 && orientation(p0, points[i],
										points[i+1]) == 0)
			i++;


		points[m] = points[i];
		m++; // Update size of modified array
	}

	// If modified array of points has less than 3 points,
	// convex hull is not possible
	if (m < 3) return;

	// Create an empty stack and push first three points
	// to it.
	stack<Point2> S;
	S.push(points[0]);
	S.push(points[1]);
	S.push(points[2]);

	// Process remaining n-3 points
	for (int i = 3; i < m; i++)
	{
		// Keep removing top while the angle formed by
		// points next-to-top, top, and points[i] makes
		// a non-left turn
		while (S.size()>1 && orientation(nextToTop(S), S.top(), points[i]) != 2)
			S.pop();
		S.push(points[i]);
	}
	
	vector<Point2> pointVector;

	// Now stack has the output points, print contents of stack
	while (!S.empty())
	{
		Point2 p = S.top();
		cout << "(" << p.x << ", " << p.y <<")" << endl;
		S.pop();
		pointVector.push_back(p);
	}

	Point2 out[NUM_VERTICES];
	copy(pointVector.begin(), pointVector.end(), out);

	points = out;
}

void randomPoints(Point2 *randomPoints, const int n) {
    int x, y;
    srand (time(NULL));

    vector<Point2> points;
    
    for (int i = 0; i < n; i++) {
        x = rand() % 15;
        y = rand() % 15;

        Point2 element;
        element.x = x;
        element.y = y;

        points.push_back(element);
    }
    const int length = sizeof (points) / sizeof (Point2);
    Point2 out[length];
    for(int i = 0; i < points.size(); i++)
    {
        out[i] = points[i];
    }

    randomPoints = out;
}

void generateRandomPolygon(Point2 *polygon, int n) {
    Point2 randPoints[1];
    
    randomPoints(randPoints, n);
 
    convexHull(randPoints);

    polygon = randPoints;

    return;
}

void printToFile(Point2 *polygon1, int length1)
{
    std::fstream object1;
    int num_obj = 0;
    std::string fileName = "../Point_Clouds/object_";

    //creates a unique identifier for each object
    struct std::stat buffer;
    while(stat ((fileName + std::to_string(num_obj)).c_str(), &buffer) == 0)
    {
        num_obj ++;
    }
    object1.open(fileName + std::to_string(num_obj));
    for(int x = 0; x < length1; x++)
    {
        object1 << polygon1[x].x << " " << polygon1[x].y << std::endl;
    }

}

int main(int argc, const char * argv[]) {
       
    Point2 vertices1[1];
    Point2 vertices2[1];

    generateRandomPolygon(vertices1, 100);
    generateRandomPolygon(vertices2, 100);
    


    // Point2 vertices1[] = {
    //     { 4.0f, 11.0f },
    //     { 5.0f, 5.0f },
    //     { 9.0f, 9.0f },
    // };
    
    // Point2 vertices2[] = {
    //     { 4.0f, 11.0f },
    //     { 5.0f, 5.0f },
    //     { 9.0f, 9.0f },
    // };

    size_t count1 = sizeof (vertices1) / sizeof (Point2); // == 3
    size_t count2 = sizeof (vertices2) / sizeof (Point2); // == 4

	for(int runs = 0; runs < 4; runs++)
	{
		Point2 a[sizeof (vertices1) / sizeof (Point2)];
		Point2 b[sizeof (vertices2) / sizeof (Point2)];

		for (size_t i = 0; i < count1; ++i) a[i] = Jostle(vertices1[i]);
		for (size_t i = 0; i < count2; ++i) b[i] = Jostle(vertices2[i]);

		int collisionDetected = gjk (a, count1, b, count2);
		if (!collisionDetected)
		{
			printf("Found failing case:\n\t{%f, %f}, {%f, %f}, {%f, %f}\n\t{%f, %f}, {%f, %f}, {%f, %f}\n\n",
				a[0].x, a[0].y, a[1].x, a[1].y, a[2].x, a[2].y,
				b[0].x, b[0].y, b[1].x, b[1].y, b[2].x, b[2].y
			);
		}
		else
		{
			printf("Collision correctly detected\n");
		}
	}
    
    exit(0);
}

