#include <omp.h>
#include <stdio.h>
#include <iostream>
#include <stack>
#include <stdlib.h>
#include <vector>
#include <time.h>
#include <chrono>
#include <thread>

#define NUM_VERTICES 100

// #define DEBUG_POLY_GENERATION
// #define DEBUG

using namespace std;

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

struct Poly
{
    float avgX, avgY;
    vector<Point2> vertices;

    Poly() {
        avgX = 0.0f;
        avgY = 0.0f;
    }

    Poly(vector<Point2> vertices_) {
        avgX = 0;
        avgY = 0;
        vertices = vertices_;
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
Point2 negatePoint (Point2 v) { v.x = -v.x; v.y = -v.y; return v; }
Point2 perpendicular (Point2 v) { Point2 p = { v.y, -v.x }; return p; }
Point2 shiftXY (Point2 v, float x, float y) {v.x += x; v.y += y; return v;}
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

Point2 averagePoint (const vector<Point2> vertices, size_t count) {
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
// ! this function is very costly in terms of runtime

size_t indexOfFurthestPoint (const vector<Point2> vertices, size_t count, Point2 d) {
    
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

Point2 support (const vector<Point2> vertices1, size_t count1,
            const vector<Point2> vertices2, size_t count2, Point2 d) {

    // get furthest point of first body along an arbitrary direction
    size_t i = indexOfFurthestPoint (vertices1, count1, d);
    
    // get furthest point of second body along the opposite direction
    size_t j = indexOfFurthestPoint (vertices2, count2, negatePoint (d));

    // subtract (Minkowski sum) the two points to see if bodies 'overlap'
    return subtract (vertices1[i], vertices2[j]);
}

//-----------------------------------------------------------------------------
// The GJK yes/no test



int gjk (const vector<Point2> vertices1, size_t count1,
         const vector<Point2> vertices2, size_t count2, bool timer) {
    
    // ! Start clock timer
    struct timespec start, stop;
    double time;
    // start clock
    if (timer) {
        if( clock_gettime(CLOCK_REALTIME, &start) == -1) { perror("clock gettime");}
    }
    

    int iter_count = 0;
    int result = 0;

    size_t index = 0; // index of current vertex of simplex
    Point2 a, b, c, d, ao, ab, ac, abperp, acperp, simplex[3];
    
    Point2 position1 = averagePoint (vertices1, count1); // not a CoG but
    Point2 position2 = averagePoint (vertices2, count2); // it's ok for GJK )

    // initial direction from the center of 1st body to the center of 2nd body
    d = subtract (position1, position2);
    
    // if initial direction is zero ??? set it to any arbitrary axis (we choose X)
    if ((d.x == 0) && (d.y == 0))
        d.x = 1.f;
    
    // set the first support as initial point of the new simplex
    a = simplex[0] = support (vertices1, count1, vertices2, count2, d);
    
    if (dotProduct (a, d) <= 0) {
        // ! END CLOCK TIMER & Print time
        if (timer) {
            if( clock_gettime( CLOCK_REALTIME, &stop) == -1 ) { perror("clock gettime");}		
            time = (double)(stop.tv_nsec - start.tv_nsec);
            printf("======   GJK CALL TIME = %f nanoseconds\n", time);
        }       
        return 0; // no collision
    }
    d = negatePoint (a); // The next search direction is always towards the origin, so the next search direction is negatePoint(a)
    
    while (iter_count < 100) {
        iter_count++;
        
        a = simplex[++index] = support (vertices1, count1, vertices2, count2, d);
        
        if (dotProduct (a, d) <= 0) {
            break; // no collision
        }
        ao = negatePoint (a); // from point A to Origin is just negative A
        
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
            
            if (dotProduct (abperp, ao) < 0) {
                
                // ! END CLOCK TIMER & Print time
                // end = std::chrono::steady_clock::now();
                // time_span = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();
                // cout << "GJK Elapsed time: " << time_span << " seconds." << endl;
                result = 1; // collision
                break;
                
            }
            simplex[0] = simplex[1]; // swap first element (point C)

            d = abperp; // new direction is normal to AB towards Origin
        }
        
        simplex[1] = simplex[2]; // swap element in the middle (point B)
        --index;
    }

    if (timer) {
        if( clock_gettime( CLOCK_REALTIME, &stop) == -1 ) { perror("clock gettime");}		
        time = (double)(stop.tv_nsec - start.tv_nsec);
        printf("======   GJK CALL TIME = %f nanoseconds with %d iterations\n", time, iter_count);
    }
   

    return result;
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
vector<Point2> convexHull(vector<Point2> points, int n)
{
    // returned polygon (empty initialization)
    vector<Point2> polygon;

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
	if (m < 3) return polygon;

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
	
	#ifdef DEBUG_POLY_GENERATION
        cout << "Generated polygon:" << endl;
    #endif
    
	// Now stack has the output points, print contents of stack
	while (!S.empty())
	{
		Point2 p = S.top();
		S.pop();
		polygon.push_back(p);

        #ifdef DEBUG_POLY_GENERATION
            cout << "(" << p.x << ", " << p.y <<")" << endl;
        #endif
	}
    #ifdef DEBUG_POLY_GENERATION
        cout << "End of generated polygon...." << endl;
    #endif
    
    return polygon;
}

vector<Point2> randomPoints(const int n, int sizeX, int sizeY, float shiftX = 0, float shiftY = 0) {
    int x, y;
    // srand (time(NULL));
    vector<Point2> randomPoints;
    for (int i = 0; i < n; i++) {
        x = rand() % sizeX;
        y = rand() % sizeY;

        Point2 element;
        element.x = x + shiftX;
        element.y = y + shiftY;

        randomPoints.push_back(element);
    }

    return randomPoints;
}

/**
 * @brief Generates random polygon.
 *          First generates point cloud of size n
 *          Then creates convex polygon from convex hull of the cloud
 *  
 * @param n number of points in random point cloud
 * @return vector of points representing polygon
 */
vector<Point2> generateRandomPolygon(int n, int sizeX, int sizeY, float shiftX = 0, float shiftY = 0) {
    #ifdef DEBUG_POLY_GENERATION
        printf("Generating polygon with max bounded size (%d, %d) shifted by (%f, %f)...\n", sizeX, sizeY, shiftX, shiftY);
    #endif

    vector<Point2> randPoints = randomPoints(n, sizeX, sizeY, shiftX, shiftY);
 
    vector<Point2> polygon = convexHull(randPoints, n);

    return polygon;
}


int main(int argc, const char * argv[]) {
    printf("=====================================================\n");
    printf("==================  Main Started  ===================\n");
    printf("=====================================================\n");
    /**
     * @brief: main method for GJK serial testing
     * 
     * @run: terminal commands:
     *      compile: g++ -o gjk gjk_for.cpp -pg -g
     *      run: gjk
     *      profile: gprof -a gjk.exe > [output file name]
     * 
     * @structure: organized as a 2 phase execution:
     * 
     * Phase 1: generate random polygons in a loop
     *      - Store polygons in a vector
     * 
     * Phase 2: find collisions between the polygons:
     *      - check for collisions between each unique pair of polygons 
     */

    // Polygon Parameters
    const int sqrt_NUM_POLYGONS = 32; // sqrt(number of polygons generated)
    const int NUM_POLYGONS = sqrt_NUM_POLYGONS * sqrt_NUM_POLYGONS;
    const int dimX = 50; // max x dimension of polygon
    const int dimY = 50; // max y dimension of polygon
    const int num_rand_points = 200; // number of random points to generate each poly
    const float space_factor = 0.4; // fraction of dim that polygons are displaced by

    int total_num_points = 0; // stats variable
    int locX, locY;
    srand (time(NULL));

    // polygon vector
    vector<Poly> polygons;
    /**
     * @brief Generate polygons in a loop
     * 
     * @param total_num_points calculates total number of points for averaging later
     * 
     */
    for (int x = 0; x < sqrt_NUM_POLYGONS; x++) {
        for (int y = 0; y < sqrt_NUM_POLYGONS; y++) {
            locX = x * (dimX * space_factor);
            locY = y * (dimY * space_factor);
            vector<Point2> vertices = generateRandomPolygon(num_rand_points, dimX, dimY, locX, locY);
            Poly curr_poly = Poly(vertices);
            polygons.push_back(curr_poly);

            total_num_points += vertices.size();
        }
    }

    float average_num_points = total_num_points/(1.0f * NUM_POLYGONS);

    // Stats variables
    int NUM_PAIRS = NUM_POLYGONS * (NUM_POLYGONS - 1) / 2;
    int num_collisions = 0;

    struct timespec start, stop;
    double time;


    // start clock
    if( clock_gettime(CLOCK_REALTIME, &start) == -1) { perror("clock gettime");}

    // thread private variables
    size_t count1 = 0, count2 = 0;
    int collisionDetected = 0, tid = 0;

    
    int loop_iter = 0;

    /**
     * @brief GJK Call Loop
     * 
     * Iterates over all unique polygon pairs and calls GJK on them.
     * 
     */
    const int NUM_THREADS = 4;
    const int block = NUM_POLYGONS/NUM_THREADS;
    omp_set_num_threads(NUM_THREADS);
    printf("============  Entering Parallel Portion  ============\n");
    #pragma omp parallel shared (polygons, num_collisions) private(count1, count2, collisionDetected)
    {        
        #pragma omp sections nowait
        {
            #pragma omp section 
            {
                vector<Poly> local_vec = polygons;
                printf("Section 1 starting...\n");
                int num_calls = 0;
                bool show_time = false;
                for (int y = 0; y < block; y++ ) {
                    for (int x = y + 1; x < NUM_POLYGONS; x++) {
                        num_calls++;
                        vector<Point2> a = local_vec[x].vertices;
                        vector<Point2> b = local_vec[y].vertices;

                        count1 = a.size();
                        count2 = b.size();

                        if (y == 0 && x == y+1) {
                            show_time = true;
                        } else {
                            show_time = false;
                        } 

                        collisionDetected = gjk (a, count1, b, count2, show_time);

                        if (!collisionDetected)
                        {
                            // printf("No collision detected\n");
                        }
                        else
                        {   
                            #pragma omp atomic
                            num_collisions++;
                            
                            #ifdef DEBUG
                                printf("Collision correctly detected between Polygon %d and Polygon %d\n", x, y);
                            #endif
                        }
                    }
                }
                printf("Number of calls in section 1: %d\n", num_calls);
            }

            #pragma omp section 
            {
                printf("Section 2 starting...\n");
                vector<Poly> local_vec = polygons;
                int num_calls = 0;
                bool show_time = false;
                for (int y = block; y < 2 * block; y++) {
                    for (int x = y + 1; x < NUM_POLYGONS; x++) {
                        num_calls++;
                        vector<Point2> a = local_vec[x].vertices;
                        vector<Point2> b = local_vec[y].vertices;

                        count1 = a.size();
                        count2 = b.size();

                        if (y == block && x == y+1) {
                            show_time = true;
                        } else {
                            show_time = false;
                        }

                        collisionDetected = gjk (a, count1, b, count2, show_time);

                        if (!collisionDetected)
                        {
                            // printf("No collision detected\n");
                        }
                        else
                        {   
                            #pragma omp atomic
                            num_collisions++;
                            
                            #ifdef DEBUG
                                printf("Collision correctly detected between Polygon %d and Polygon %d\n", x, y);
                            #endif
                        }
                    }
                }
                printf("Number of calls in section 2: %d\n", num_calls);
            }

            #pragma omp section 
            {
                printf("Section 3 starting...\n");
                vector<Poly> local_vec = polygons;
                int num_calls = 0;
                bool show_time = false;
                for (int y = 2 * block; y < 3 * block; y++) {
                    for (int x = y + 1; x < NUM_POLYGONS; x++) {
                        num_calls++;
                        vector<Point2> a = local_vec[x].vertices;
                        vector<Point2> b = local_vec[y].vertices;

                        count1 = a.size();
                        count2 = b.size(); 
                        
                        if (y == 2 * block && x == y+1) {
                            show_time = true;
                        } else {
                            show_time = false;
                        }

                        collisionDetected = gjk (a, count1, b, count2, show_time);

                        if (!collisionDetected)
                        {
                            // printf("No collision detected\n");
                        }
                        else
                        {   
                            #pragma omp atomic
                            num_collisions++;
                            
                            #ifdef DEBUG
                                printf("Collision correctly detected between Polygon %d and Polygon %d\n", x, y);
                            #endif
                        }
                    }
                }
                printf("Number of calls in section 3: %d\n", num_calls);
            }

            #pragma omp section 
            {
                printf("Section 4 starting...\n");
                vector<Poly> local_vec = polygons;
                int num_calls = 0;
                bool show_time = false;
                for (int y = 3 * block; y < NUM_POLYGONS - 1; y ++) {
                    for (int x = y + 1; x < NUM_POLYGONS; x++) {
                        num_calls++;
                        vector<Point2> a = local_vec[x].vertices;
                        vector<Point2> b = local_vec[y].vertices;

                        count1 = a.size();
                        count2 = b.size(); 

                        if (y == 3* block && x == y+1) {
                            show_time = true;
                        } else {
                            show_time = false;
                        }

                        collisionDetected = gjk (a, count1, b, count2, show_time);

                        if (!collisionDetected)
                        {
                            // printf("No collision detected\n");
                        }
                        else
                        {   
                            #pragma omp atomic
                            num_collisions++;
                            
                            #ifdef DEBUG
                                printf("Collision correctly detected between Polygon %d and Polygon %d\n", x, y);
                            #endif
                        }
                    }
                }
                printf("Number of calls in section 4: %d\n", num_calls);
            }
        }

    }


    // Timing
	if( clock_gettime( CLOCK_REALTIME, &stop) == -1 ) { perror("clock gettime");}		
    time = (stop.tv_sec - start.tv_sec)+ (double)(stop.tv_nsec - start.tv_nsec)/1e9;

    float avg_time_sec = time/NUM_PAIRS;

    // Summary
    printf("\n\n");
    printf("=====================================================\n");
    printf("=================  Main completed  ==================\n");
    printf("=====================================================\n");
    printf("======   Num Polygons = %d\n", NUM_POLYGONS); 
    printf("======   Avg num points = %f\n", average_num_points);
    printf("======   GJK run on %d pairs of polygons\n", NUM_PAIRS);
    printf("======   Total Num collisions: %d\n", num_collisions);
    printf("======   Total time for GJK = %f seconds\n", time);
    printf("======   Average time per GJK call = %f ms\n", avg_time_sec);
    printf("=====================================================\n");
    printf("=================  End of Summary  ==================\n");
    printf("=====================================================\n");
    
    exit(0);
}

