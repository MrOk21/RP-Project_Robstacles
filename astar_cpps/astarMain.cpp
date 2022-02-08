#include <iostream>
#include "A_star.cpp"
using namespace std;


int main()
{
    Map m(3,3);

    m.get_init_state(0.0,0.0);
    m.get_goal_state(2.0,2.0);
    m.A_star();
    m.display();

}