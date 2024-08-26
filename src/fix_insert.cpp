#include"FlowFunc.hpp"
#include <omp.h>
#include <thread>
#include"Plane_extension.h"
using namespace std;

int main(int argc, char** argv)
{
    //start time
    double start = clock();
    Plane_E chip(argv[1]);
    //chip.debank();
    chip.write_input_format("input.txt");
    cout<<"HPWL OPTIMIZATION"<<endl;
    double weight = 5000;
    while(weight > 1000)
    {
        for(int i = 0 ; i < 50 ; i++)
        {
            chip.update_slack_pin_weight(weight);
            chip.HPWL_optimizer();
            chip.set_on_site();
        }
        weight*=0.7;
    }

    cout<<"SLACK OPTIMIZATION"<<endl;
    weight = 5000;
    while(weight > 1000)
    {
        for(int i = 0 ; i < 50 ; i++)
        {
            chip.update_slack_pin_weight(weight);
            chip.HPWL_optimizer();
            chip.set_on_site();
        }
        weight*=0.7;
    }
    int step = 40;
    while(step >= 1)
    {
        for(int j = 0 ; j < 50 ; j++)
        {
            chip.slack_optimizer(step);
        }
        step*=0.7;
    }
    chip.location_legalization();
    chip.write_input_format("input.txt");
    chip.output(argv[2]);
    
    
    //cout<< end time
    cout<<(clock() - start) / 1000000.0<<endl;

    cout<<"END"<<endl;
    
    return 0;
}

