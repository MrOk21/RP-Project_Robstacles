#include "path_planning/astar.h"


namespace path_planning{
void Astar::InitAstar(Mat& _Map, Mat& Mask, Astar_Mods _mods)
{
    

    int neighbor_radius[8][2] = {
            {-1, -1}, {-1, 0}, {-1, 1},
            {0, -1},            {0, 1},
            {1, -1},   {1, 0},  {1, 1}
    };


    mods = _mods;
    Map = _Map; 
   
    neighbor = Mat(8, 2, CV_32S, neighbor_radius).clone(); // global variable
    
    
    MapProcess(Mask); // Get the mask for the obstacles....
}

void Astar::PathPlanning(Point _startPoint, Point _targetPoint, vector<Point>& path)
{
    // Get variables, they are the private variables of Astar class
    startPoint = _startPoint;
    targetPoint = _targetPoint;

    // Path Planning
    Node* TailNode = FindPath();
    ReconstructPath(TailNode, path);
}


void Astar::MapProcess(Mat& Mask)
{
    int width = Map.cols;
    int height = Map.rows;
    Mat _Map = Map.clone();

    // Transform RGB to gray image
    if(_Map.channels() == 3)
    {
        cvtColor(_Map.clone(), _Map, cv::COLOR_BGR2GRAY);
    }

    // Binarize
    threshold(_Map.clone(), _Map, 0, 255, cv::THRESH_OTSU);
    

    // Inflate: This is responsible for having the obstacles 'larger'. The margin to consider is proportional to this value.
    Mat src = _Map.clone();
    if(mods.tolerance > 0)
    {
        Mat se = getStructuringElement(MORPH_ELLIPSE, Size(2 * mods.tolerance, 2 * mods.tolerance));
        erode(src, _Map, se);
    }

    // Get mask
    bitwise_xor(src, _Map, Mask);

    // ObstacleMap
    ObstacleMap = Mat::zeros(height, width, CV_8UC1);
    for(int y=0;y<height;y++)
    {
        for(int x=0;x<width;x++)
        {
            if(_Map.at<uchar>(y, x) == 0)
            {
                ObstacleMap.at<uchar>(y, x) = obstacle;
            }
            else
            {
                ObstacleMap.at<uchar>(y, x) = traversable;
            }
        }
    }
}

Node* Astar::FindPath()
{

    //--------------------------------- Initialiazing A* ----------------------------------//
    int width = Map.cols;
    int height = Map.rows;


    // The respective of the ObstacleMap, contains if a node is or is not in the OpenSet
    Mat _ObstacleMap = ObstacleMap.clone();

    // Add startPoint to PrioritySet (priority queue data type)
    Node* startPointNode = new Node(startPoint); // The address is safely allocated in the heap

    // In the Visited Nodes lists (PrioritySet and OpenSet), the memory addresses are stored.
    // PrioritySet corresponds to {(F_value1, Node1), (F_value2, Node2), .....}
    // To access the least F Node ----> PrioritySet.top().second
    PrioritySet.push(pair<int, Point>(startPointNode->F, startPointNode->point));
    
    int index = point2index(startPointNode->point);
    
    OpenSet[index] = startPointNode;
    
    _ObstacleMap.at<uchar>(startPoint.y, startPoint.x) = inOpenSet; // pixel value of the cloned map is set to be visited
    //--------------------------------- Start A* ----------------------------------//
    while(!PrioritySet.empty())
    {
        // Take the node with least F value
        Point CurPoint = PrioritySet.top().second;
        // Remove the node
        PrioritySet.pop();


        int index = point2index(CurPoint);
        Node* CurNode = OpenSet[index]; // Node information maybe retrieved by the OpenSet.
        OpenSet.erase(index); // This will be put in the close list.

        int curX = CurPoint.x;
        int curY = CurPoint.y;
        _ObstacleMap.at<uchar>(curY, curX) = inCloseSet;

        // Determine whether arrive the target point
        if(curX == targetPoint.x && curY == targetPoint.y)
        {
            return CurNode; // Found a valid path
        }

        // Check the neighborhood
        for(int k = 0;k < neighbor.rows;k++)
        {   
            // New Neighbor
            int y = curY + neighbor.at<int>(k, 0);
            int x = curX + neighbor.at<int>(k, 1);

            // Check Map's Boundaries
            if(x < 0 || x >= width || y < 0 || y >= height)
            {
                continue;
            }
            if(_ObstacleMap.at<uchar>(y, x) == traversable || _ObstacleMap.at<uchar>(y, x) == inOpenSet)
            {
                // Determine if diagonal motion is constrained by lateral obstacles.
                int dist1 = abs(neighbor.at<int>(k, 0)) + abs(neighbor.at<int>(k, 1));
                if(dist1 == 2 && _ObstacleMap.at<uchar>(y, curX) == obstacle && _ObstacleMap.at<uchar>(curY, x) == obstacle)
                    continue;

                // Calculate G, H, F value
                int addG, G, H, F;
                if(dist1 == 2)
                {
                    addG = sqrt(2);
                }
                else
                {
                    addG = 1;
                }
                // Cumulate the cost
                G = CurNode->G + addG;
                if(mods.Eucl_Heuristics)
                {
                    int dist2 = pow((x - targetPoint.x), 2) + pow((y - targetPoint.y), 2);
                    H = round(sqrt(dist2));
                }
                else
                {
                    H = abs(x - targetPoint.x) + abs(y - targetPoint.y);
                }
                F = G + H;

                // Update the G, H, F value of node
                if(_ObstacleMap.at<uchar>(y, x) == traversable)
                {   
                    // Instantiating new node
                    Node* node = new Node();
                    node->point = Point(x, y);
                    node->parent = CurNode;
                    node->G = G;
                    node->H = H;
                    node->F = F;
                    PrioritySet.push(pair<int, Point>(node->F, node->point));
                    int index = point2index(node->point);
                    OpenSet[index] = node;
                    _ObstacleMap.at<uchar>(y, x) = inOpenSet;
                }
                else // The other possibility is that it is already in the OpenSet.
                {
                    // Find the node
                    int index = point2index(Point(x, y));
                    Node* node = OpenSet[index];
                    if(G < node->G)
                    {   // Change its parent
                        node->G = G;
                        node->F = F;
                        node->parent = CurNode;
                    }
                }
            }
        }
    }

    return NULL; // Can not find a valid path
}

void Astar::ReconstructPath(Node* TailNode, vector<Point>& path)
{
    path.clear();

    // Save path to vector<Point>
    Node* CurNode = TailNode;
    while (CurNode != nullptr)
    {
        path.push_back(CurNode->point); // Vector of waypoints
        CurNode = CurNode->parent;
    }
    
    // Reverse the path to get the correct order
    reverse(path.begin(), path.end());


    // Release memory
    while(PrioritySet.size()) {
        Point CurPoint = PrioritySet.top().second;
        PrioritySet.pop();
        int index = point2index(CurPoint);
        Node* CurNode = OpenSet[index];
        delete CurNode; // To avoid memory leaks the dynamically allocated variables are deleted. 
    }
    OpenSet.clear();
}

};

std::ostream& operator<<(std::ostream& os, const std::vector<Point>& vec) {
    os << "The path planning has been successfull!!"<<endl;
    os << "The number of waypoints is: " << vec.size()<<endl;
    return os;
}