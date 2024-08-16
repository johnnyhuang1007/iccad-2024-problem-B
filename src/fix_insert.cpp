#include"FlowFunc.hpp"
#include <omp.h>
#include <thread>
#include"Plane_extension.h"
using namespace std;

int main(int argc, char** argv)
{

    Plane_E chip(argv[1]);
    chip.loc_randomize();
    cout<<"HPWL OPTIMIZATION"<<endl;
    double weight = 5000;
    while(weight > 4000)
    {
        for(int i = 0 ; i < 40 ; i++)
        {
            chip.HPWL_optimizer(weight);
        }
        weight*=0.9;
    }
    chip.set_on_site();
    double step = 40;
    for(int i = 0 ; i < 10 ; i++)
    {
        for(int i = 0 ; i < 200 ; i++)
        {
            chip.slack_optimizer(step);
        }
        step*=0.7;
    }
    chip.reduce_high_util_pin_weight();
    chip.insert_FFs();
    chip.output(argv[2]);
    return 0;
}

