#include <iostream>
#include <algorithm>
#include <vector>
#include <list>
#include <limits>
#include <cmath>
using namespace std;

struct Node   // This is the struct node, that represent the attributes that every state in the map assumes, we also use it to describe the obstacles.
{
    bool visited=false;
    bool obstacle=false;
    vector<Node*> neighB; //Neighbors of the current node are represented by a vector of pointers to the neighbors.
    double tot;           //This is the sum of the heuristic (optimistic distance from the current goal to the end) + the cost we had to reach the current node
    double g;
    double x;   
    double y;
    Node* prev=nullptr;   //This pointer is a pointer to the previous node, based on the better cost.
};

class Map
{
    private:
        int height;
        int weight;
        Node* initN;
        Node* goalN;
        Node* allN; 
    public:
        int dec=1;
        Map(){}
        Map(int h, int w): height(h),weight(w) //TODO: add some check for the feasibility......
        {
            
            allN=new Node[height*weight*dec];
            for (double x_=0.0;x_<weight;)    //In this nested loops we create our nodes
                {
                    for (double y_=0.0;y_<height;)
                        {
                            
                            allN[(long)(y_*weight+x_)*dec].x=x_;
                            allN[(long)(y_*weight+x_)*dec].y=y_;
                            allN[(long)(y_*weight+x_)*dec].obstacle=false;
                            allN[(long)(y_*weight+x_)*dec].visited=false;
                            allN[(long)(y_*weight+x_)*dec].prev=nullptr;
                            allN[(long)(y_*weight+x_)*dec].tot=numeric_limits<float>::infinity();;
                            allN[(long)(y_*weight+x_)*dec].g=numeric_limits<float>::infinity();;

                            
                            y_+= 1/dec;
                        }
                    
                    x_+=1/dec;    
                }
            for (double x_=0.0;x_<weight;)    
                {
                    for (double y_=0.0;y_<height;)
                        {
                            if(y_>0)
                                allN[(long)(y_*weight + x_)*dec].neighB.push_back(&allN[(long)((y_ - 1) * weight + (x_ + 0))*dec]); 
                                
                            if(y_<height-1)
                                allN[(long)(y_*weight + x_)*dec].neighB.push_back(&allN[(long)((y_ + 1) * weight + (x_ + 0))*dec]); 
                            if (x_>0)
                                allN[(long)(y_*weight + x_)*dec].neighB.push_back(&allN[(long)((y_ + 0) * weight + (x_ - 1))*dec]); 
                            if(x_<weight-1)
                                allN[(long)(y_*weight + x_)*dec].neighB.push_back(&allN[(long)((y_ + 0) * weight + (x_ + 1))*dec]);

                            if (y_>0 && x_>0)
                                allN[(long)(y_*weight + x_)*dec].neighB.push_back(&allN[(long)((y_ - 1) * weight + (x_ - 1))*dec]);
                            if (y_<height-1 && x_>0)
                                allN[(long)(y_*weight + x_)*dec].neighB.push_back(&allN[(long)((y_ + 1) * weight + (x_ - 1))*dec]);
                            if (y_>0 && x_<weight-1)
                                allN[(long)(y_*weight + x_)*dec].neighB.push_back(&allN[(long)((y_ - 1) * weight + (x_ + 1))*dec]);
                            if (y_<height - 1 && x_<weight-1)
                                allN[(long)(y_*weight + x_)*dec].neighB.push_back(&allN[(long)((y_ + 1) * weight + (x_ + 1))*dec]);
                            y_+= 1/dec;
                        }
                    x_+=1/dec; 
                }    
        }

        void get_init_state(double x_, double y_)
        {
            initN=&allN[(long)(x_+ weight*y_)*dec];
            initN->g=0.0;
            initN->prev=nullptr;
            initN->obstacle=false;
            initN->visited=false;

        }

        void get_goal_state(double x_, double y_)
        {
            goalN=&allN[(long)(x_+ weight*y_)*dec];
            
            
            cout<<"The goal state is: ("<<goalN->x<<","<<goalN->y<<")"<<endl;
        }

        void get_obstacle(double x_, double y_)
        {
            allN[(long)(x_+ weight*y_)*dec].obstacle=true;
        }

        void A_star()
        {   

            
                    
            list<Node*> toTestNodes; //toTestNodes contains a list of element for which we want to know their neighbors values.
            toTestNodes.push_back(initN);
            Node* currentN=initN;
            int c;
            while (!toTestNodes.empty() && currentN!=goalN)
                {
                    toTestNodes.sort(CompareNodes()); //CompareNodes is defined within this class, because we need it in the same scope.
                    
                    
                    while (!toTestNodes.empty() && toTestNodes.front()->visited) //We may end up with an empty list and no goal state
                        toTestNodes.pop_front();
                
                    if (toTestNodes.empty()) //This is the case when we abort
                        {
                            return;
                        }

                    
                    currentN=toTestNodes.front();
                    currentN->visited=true;
                    for (auto& it: currentN->neighB)  //We iteratate with a smart pointer to pointers that point to the nodes
                        {
                            if (!(it->visited) && !(it->obstacle))  //If they are not neither visited or neither obstacles, we add to the list of promising nodes
                                {
                                    toTestNodes.push_back(it);
                                }
                            
                            
                            double currentG= currentN->g + distance(it, currentN); //The neighbor node, could be already reachable from other node, so we have first
                                                                                 // to check if from currentN is better to reach it, otherwise we don't consider it 

                            if (currentG < it->g) //the g attribute is set to infinity, at the beginning.
                                {
                                    
                                    it->prev=currentN;
                                    it->g=currentG;
                                    it->tot= it->g + 5*distance(it, goalN);

                                }
                        
                        }
                    
                    
                }
        }

        void display()
            {
                cout<<"This is the display function"<<endl;
                
                list<Node*> path;
                cout<<"c1"<<endl;
                Node* d=goalN;
                cout<<"c2"<<endl;

                path.push_front(d);
                cout<<"c3"<<endl;
                while (d->prev!=nullptr)
                    {
                        
                        d=d->prev;
                        path.push_front(d);
                    }
                for (auto state: path)
                    cout<<state->x <<","<<state->y<<endl;
            }

        struct CompareNodes
        {
            bool operator() (const Node *n1,const Node *n2)
                {
                    
                    return ((n1->tot > n2->tot) ? true:false);
                }
        };

        double distance(const Node* n1,const Node* n2)
        {
            double distance=sqrt(((n1->x - n2->x)*(n1->x - n2->x) + (n1->y - n2->y)*(n1->y - n2->y)));
            return distance;
        }

};


