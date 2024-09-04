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
    chip.debank();
    chip.write_input_format("input.txt");
    chip.set_on_site();
    chip.location_legalization();
    chip.output(argv[2]);
    cost = chip.cost();

    chip.remove_FFs();
    int step = 40;
    /*
    step = 40;
    for(int j = 0 ; j < 500 ; j++)
    {
        chip.robust_slack_optimizer(40);
    }
    for(int j = 0 ; j < 300 ; j++)
    {
        chip.robust_slack_optimizer(20);
    }
    for(int j = 0 ; j < 200 ; j++)
    {
        chip.robust_slack_optimizer(3);
    }
    if(chip.cost() < cost)
    {
        cost = chip.cost();
        chip.output(argv[2]);
    }
    */
    /*
    cout<<"HPWL OPTIMIZATION"<<endl;
    double weight = 5000;
    while(weight > 4000)
    {
        for(int i = 0 ; i < 5 ; i++)
        {
            chip.update_slack_pin_weight(weight);
            chip.HPWL_optimizer();
            chip.set_on_site();
        }
        weight*=0.7;
    }

    cout<<"SLACK OPTIMIZATION"<<endl;
     step = 40;
    while(step >= 1)
    {
        for(int j = 0 ; j < 5 ; j++)
        {
            chip.slack_optimizer(step);
        }
        step*=0.7;
    }
    */
    //chip.location_legalization();
    //chip.remove_FFs();
    
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
    for(int j = 0 ; j < 500 ; j++)
    {
        chip.robust_slack_optimizer(40);
    }
    for(int j = 0 ; j < 300 ; j++)
    {
        chip.robust_slack_optimizer(20);
    }
    for(int j = 0 ; j < 200 ; j++)
    {
        chip.robust_slack_optimizer(3);
    }
    if(chip.cost() < cost)
    {
        cost = chip.cost();
        chip.output(argv[2]);
    }
    chip.pin_swapping();
    //cout<< end time
    cout<<(clock() - start) / 1000000.0<<endl;
    chip.write_input_format("input.txt");
    chip.output(argv[2]);
    cout<<"END"<<endl;
    
    return 0;
}

