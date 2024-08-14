#include "module_extension.h"
using namespace std;

Inst::Inst(std::string a, Point b, int c, int d):Fixed_Module(a,b, c, d)    // I/O constructor, do not use in FF/GATE generation!!
{
    corr_Lib.first = -1;
    Pin* newP = new Pin;
    newP->org_type = a;
    newP->pin_type = a;
    newP->orginal_Inst = this;
    newP->belong_Inst = this;
    newP->relative_loc.x = 0;
    newP->relative_loc.y = 0;
    this->Pins.push_back(newP);
}

Inst::Inst(std::string name, Point LD, Inst_data* data,std::pair<int,int> T_Idx ,std::string NOT_To_NEW):Fixed_Module(name,LD, data->height, data->width)
{
    corr_Lib = T_Idx;
    this->corr_data = data;
    this->Pins.resize(data->pinCount);
    this->Pins.resize((data->pinCount-1)/2);
    this->Pins.resize((data->pinCount-1)/2);
    
    return;
}

Inst::Inst(std::string name, Point LD, Inst_data* data,std::pair<int,int> T_Idx ):Fixed_Module(name,LD, data->height, data->width)
{
    if(T_Idx.first == 0)
    {
        corr_Lib = T_Idx;
        this->corr_data = data;
        for(const auto& p_data:data->Pin_set)
        {
            
            Pin* newP = new Pin(p_data);
            newP->orginal_Inst = this;
            newP->belong_Inst = this;
            this->Pins.push_back(newP);
        }
        return;
    }
    corr_Lib = T_Idx;
    this->corr_data = data;
    for(const auto& p_data:data->Pin_set)
    {
        Pin* newP = new Pin(p_data);
        newP->orginal_Inst = this;
        newP->belong_Inst = this;
        this->Pins.push_back(newP);
    }
    return;
}

Pin* pair_D(Inst* inst, Pin* p)
{
    string name = p->pin_type;
    name[0] = 'D';
    for(Pin* in:inst->INs)
    {
        if(in->pin_type == name)
            return in;
    }
}
Pin* pair_Q(Inst* inst, Pin* p)
{
    string name = p->pin_type;
    name[0] = 'Q';
    for(Pin* in:inst->OUTs)
    {
        if(in->pin_type == name)
            return in;
    }
}

Inst::Inst(Inst* copy):Fixed_Module(copy->name,LD(copy), height(copy), width(copy))
{
    this->corr_Lib = copy->corr_Lib;
    this->corr_data = copy->corr_data;
    this->Pins.resize(copy->Pins.size());
    for(int i = 0 ; i < copy->Pins.size() ; i++)
    {
        this->Pins[i]= copy->Pins[i];
        this->Pins[i]->belong_Inst = this;
    }
    inserted = 0;
}

void Inst::set_new_loc(Point newLoc)
{
    if(inserted)
    {
        cout<<"YOU SHOULDN'T SET LOCATION AFTER INSERTION"<<endl;
        exit(0);
    }
    this->get_root()->coord[1] = this->get_root()->coord[1]-this->get_root()->coord[0]+newLoc;
    this->get_root()->coord[0] = newLoc;
    
}

