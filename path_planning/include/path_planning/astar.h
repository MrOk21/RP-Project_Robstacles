#ifndef ASTAR_H
#define ASTAR_H


#include <iostream>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


// Namespace is required, since in opencv there exists already a Node declaration.
namespace path_planning{

enum NodeType{
    obstacle = 0,
    inOpenSet,
    inCloseSet,
    traversable
};

struct Node{
    Point point; // It is a tuple of integers
    Node* parent;
    int F, G, H;

    Node(Point _point = Point(0, 0)) // Point init. to the origin, Otherwise is initialized with a Point obj.
    {

        this->point = _point;
        this->F = 0;
        this->G = 0;
        this->H = 0;
        this->parent = nullptr;

    }
};

struct compute_precedence
{   
    // From priority queue, there is a sorting to take the best F node.
    // Is p1 greater than p2? 
    bool operator() (pair<int, Point> p1, pair<int, Point> p2) // Comparison function for priority queue
    {
        return p1.first > p2.first; // min heap
    }
};

struct Astar_Mods{
    // Astar set-up
    // 1. Set the heuristics,
    // 2. Set the distance from the obstacles.
    bool Eucl_Heuristics; // if true, euclidean distance is h(;)
    int tolerance; // distance from the obstacles
    Astar_Mods(bool _Eucl_Heuristics = true, int _tolerance = 0)
    {
        this->Eucl_Heuristics = _Eucl_Heuristics;
        this->tolerance = _tolerance;
    }

};

class Astar{

public:
    // Interface function
    void InitAstar(Mat& _Map, Mat& Mask, Astar_Mods _config = Astar_Mods()); // Default initialization
    void PathPlanning(Point _startPoint, Point _targetPoint, vector<Point>& path);
    
    // Largely used functions, inline is used for faster execution
    // Points are in the image plane
    inline int point2index(Point point) {
        // Example: point.x = 3, point.y = 1 ---> index = 9; where Map.cols = 6
        return point.y * Map.cols + point.x;
    }
    inline Point index2point(int index) {
        // Example: index = 9 --> point.x: int(9/6)= 1; point.y: 9%6 = 3;
        return Point(int(index / Map.cols), index % Map.cols);
    }

private:
    // This section is only accessible inside the class
    // Process the map: Inflate, Euclidean, OccupancyThreshold
    void MapProcess(Mat& Mask);
    // A* engine
    Node* FindPath();
    // Once the path is found, is reconstructed with GetPath
    void ReconstructPath(Node* TailNode, vector<Point>& path);
    //Object
    Mat Map;
    Point startPoint, targetPoint;
    Mat neighbor;

    Mat ObstacleMap;
    Astar_Mods mods;

    priority_queue<pair<int, Point>, vector<pair<int, Point>>, compute_precedence> PrioritySet; // Sorted Visited Nodes
    unordered_map<int, Node*> OpenSet; // Visited nodes

};

};


std::ostream& operator<<(std::ostream& os, const std::vector<Point>& vec);





#endif // ASTAR_H