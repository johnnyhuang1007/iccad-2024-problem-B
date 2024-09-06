#include"FlowFunc.hpp"
#include <omp.h>
#include <thread>
#include"Plane_extension.h"
using namespace std;

int main(int argc, char** argv)
{
    //start time
    double start = clock();
    double cost;
    Plane_E chip(argv[1]);
    cost = chip.cost();
    chip.debank();
    
    for(int itr = 0 ; itr < 5 ; itr++)
    {
        double costa = cost;
        chip.set_on_site();
        chip.location_legalization();
        chip.output(argv[2]);
        cost = chip.cost();
        if(itr<=1)
            chip.remove_FFs();
        int step = 40;
        
        chip.legality_look_ahead_banking();
        chip.set_on_site();
        chip.location_legalization();
        if(chip.cost() < cost)
        {
            cost = chip.cost();
            chip.output(argv[2]);
        }
        cout<<"MOVEMENT"<<endl;
        //fine tune
        step = 40;
        for(int j = 0 ; j < 15 ; j++)
        {
            chip.robust_slack_optimizer(40);
        }
        if(chip.cost() < cost)
        {
            cost = chip.cost();
            chip.output(argv[2]);
        }
        if(costa < chip.cost()*(1.02))
            break;
    }
    int itr = 0;
    int step = 40;
    while(step >= 1)
    {
        double prev_cost = chip.cost();
        for(int j = 0 ; j < 15 ; j++)
        {
            chip.robust_slack_optimizer(step);
        }
        step*=0.85; 
        if(prev_cost < chip.cost()*(1.005))
            break;
        itr++;
    }
    //cout<< end time
    cout<<(clock() - start) / 1000000.0<<endl;
    chip.output(argv[2]);
    cout<<"END"<<endl;
    
    return 0;
}

