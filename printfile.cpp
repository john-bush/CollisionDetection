vector<vector<Point2>> divide_into_sub_polygons(vector<Point2> Poly, int num)
{
    vector<vector<Point2>> subPoly;

    for(int x = 0; x < num; x++)
    {
        vector<Point2> sub = {Poly.begin() + x, Poly.begin()+ x + 1};
        subPoly.push_back(sub);
    }
    return subPoly;
}

void printToFile(Point2 *polygon1, int length1, std::string fileName)
{
    std::fstream object1;
    int num_obj = 0;
    
    object1.open(fileName);
    //creates a unique identifier for each object
    /* struct std::stat buffer;
    while(stat ((fileName + std::to_string(num_obj)).c_str(), &buffer) == 0)
    {
        num_obj ++;
    }*/
   for (int x = 0; x < length1; x++)
   {
    object1 << polygon1[x].x << " " <<polygon1[x].y << std::endl;
   }
   object1.close();
}
vector<Point2> readFromFile(std::string fileName)
{
    vector<Point2> polygon;
    std::fstream object1;
    std::string line;
    object1.open(fileName);

    while(std::getline(object1,line))
    {
        Point2 newPoint;
        newPoint.x = line.substr(0,line.find(" "));
        newPoint.y = line.substr(line.find(" "));
        polygon.push_back(newPoint);
    }
    return polygon;
}
