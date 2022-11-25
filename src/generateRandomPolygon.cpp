#include "..\include\generateRandomPolygon.h"

using namespace std;

void randomPoints(Point2 *randomPoints, int n) {
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

    Point2 out[sizeof (points) / sizeof (Point2)];
    copy(points.begin(), points.end(), out);

    randomPoints = out;
}

void generateRandomPolygon(Point2 *polygon, int n) {
    Point2 randPoints[1];
    
    randomPoints(randPoints, n);
 
    convexHull(randPoints, n);

    polygon = randPoints;

    return;
}

