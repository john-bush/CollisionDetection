#include "..\include\gjk.h"
#include "..\include\generateRandomPolygon.h"

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