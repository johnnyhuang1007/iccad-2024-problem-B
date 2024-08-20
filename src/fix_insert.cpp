#include"FlowFunc.hpp"
#include <omp.h>
#include <thread>
#include"Plane_extension.h"
using namespace std;

int main(int argc, char** argv)
{

    Plane_E chip(argv[1]);
    //chip.debank();
    chip.write_input_format("input.txt");
    cout<<"HPWL OPTIMIZATION"<<endl;
    double weight = 5000;
    while(weight > 4999)
    {
        for(int i = 0 ; i < 5 ; i++)
        {
            chip.update_slack_pin_weight(weight);
            chip.HPWL_optimizer();
            chip.set_on_site();
        }
        weight*=0.9;
    }
    chip.location_legalization();
    return 0;
    chip.bank();
    weight = 5000;
    while(weight > 100)
    {
        for(int i = 0 ; i < 40 ; i++)
        {
            chip.update_slack_pin_weight(weight);
            chip.HPWL_optimizer();
            chip.set_on_site();
        }
        weight*=0.9;
    }
    cout<<"SLACK OPTIMIZATION"<<endl;
    double step = 40;
    for(int i = 0 ; i < 10 ; i++)
    {
        for(int i = 0 ; i < 200 ; i++)
        {
            chip.slack_optimizer(step);
        }
        step*=0.7;
    }
    cout<<"END"<<endl;
    chip.insert_FFs();
    chip.write_input_format("input.txt");
    chip.output(argv[2]);
    return 0;
}

