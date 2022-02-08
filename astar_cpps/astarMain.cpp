#include <iostream>
#include "A_star.cpp"
using namespace std;


int main()
{
    Map m(200,200);

    m.get_init_state(0.0,0.0);
    m.get_goal_state(195.0,120.0);
    m.get_obstacle(2.0,2.0);
    m.get_obstacle(25.0,2.0);
    m.get_obstacle(70.0,42.0);
    m.A_star();
    m.display();

}