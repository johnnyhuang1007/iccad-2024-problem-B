#include "module_extension.h"
using namespace std;

Pin::Pin(const Pin& copy)
{
    this->belong_nets = copy.belong_nets;
    this->orginal_Inst = copy.orginal_Inst;
    this->pin_type = copy.pin_type;
    this->org_type = copy.pin_type;
    this->type = copy.type;
    this->belong_Inst = copy.belong_Inst;
    this->relative_loc = copy.relative_loc; //if belong inst is null then it's absolute
    this->org_relative_loc = copy.org_relative_loc;
    this->matching_Pin = copy.matching_Pin;
    this->recursive = 0;
}

Pin::~Pin()
{
}

Pin* binding(Pin* master,Pin* slave)
{
    master->pin_type = slave->pin_type;
    master->belong_Inst = slave->belong_Inst;
    master->relative_loc = slave->relative_loc; 
    delete slave;
    return master;   
}

Inst_data::Inst_data(int width,int height,int bits ,int pinCount,string FF_name,vector<Pin> Pin_locs)
{
    this->width = width;
    this->height = height;
    this->bits = bits;
    this->pinCount = pinCount;
    this->name = FF_name;
    this->Pin_set = Pin_locs;
}

Inst_data::~Inst_data()
{
}


    


