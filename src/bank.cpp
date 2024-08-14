#include "Plane_extension.h"
#include <fstream>
#include<iomanip>
#define u_map_itr(T) unordered_map<std::string,T>::const_iterator
using namespace std;



Inst* Plane_E::bank(std::vector<Inst*> to_bank, Inst_data* objective)
{
    for(int i = 0 ; i < to_bank.size() ; i++)
    {
        min_util(to_bank[i]);
        remove_Inst(to_bank[i]);
        int idx = to_bank[i]->idx;
        FF_list_bank[idx] = FF_list_bank.back();
        FF_list_bank[idx]->idx = idx;
        FF_list_bank.pop_back();
    }
    name_cnter++;
    T_Idx p = find_lib_by_name(objective->name);
    int idx = p.second;
    string name = "rEG_NEW_"+to_string(name_cnter);

    double cx = 0.0,cy = 0.0, total_weight = 0.0;
    for(int i = 0 ; i < to_bank.size() ; i++)
    {
        double weight = 0.0;
        for(int j = 0 ; j < to_bank[i]->INs.size() ; j++)
            weight += to_bank[i]->INs[j]->critical_weight;
        for(int j = 0 ; j < to_bank[i]->OUTs.size() ; j++)
            weight += to_bank[i]->OUTs[j]->critical_weight;
        cx += weight*double(to_bank[i]->LeftDown().x);
        cy += weight*double(to_bank[i]->LeftDown().y);
        total_weight += weight;
    }
    Point newLD;
    newLD.x = cx/total_weight;
    newLD.y = cy/total_weight;

    Inst* N = to_bank[0];
    Tile* root = N->get_root();
    N->corr_data = &FF_lib[idx];
    N->corr_Lib = p;
    root->coord[1] = N->LeftDown() + Point(FF_lib[idx].height,FF_lib[idx].width) - Point(1,1);
    N->set_new_loc(newLD);
    N->set_name(name);

    N->Pins.reserve(N->corr_data->bits);
    N->INs.reserve((N->Pins.size()-1)/2);
    N->OUTs.reserve((N->Pins.size()-1)/2);
    vector<Pin> data_ins,data_out;
    Pin clk;
    data_ins.reserve(N->INs.size());
    data_out.reserve(N->INs.size());
    for(int i = 0 ; i < objective->Pin_set.size() ; i++)
    {
        if(objective->Pin_set[i].type=='I')
            data_ins.push_back(objective->Pin_set[i]);
        else if(objective->Pin_set[i].type=='O')
            data_out.push_back(objective->Pin_set[i]);
        else
            clk = objective->Pin_set[i];
    }

    int in_cnt = 0;
    int out_cnt = 0;
    for(int i = 1 ; i < to_bank.size() ; i++)
    {
        for(int j = 0 ; j < to_bank[i]->INs.size() ; j++)
        {
            N->Pins.push_back(to_bank[i]->INs[j]);
            N->INs.push_back(to_bank[i]->INs[j]);
        }
        for(int j = 0 ; j < to_bank[i]->OUTs.size() ; j++)
        {
            N->Pins.push_back(to_bank[i]->OUTs[j]);
            N->OUTs.push_back(to_bank[i]->OUTs[j]);
        }
    }
    for(int j = 0 ; j < to_bank[0]->INs.size() ; j++)
    {
        to_bank[0]->INs[j]->pin_type = data_ins[in_cnt].pin_type;
        to_bank[0]->INs[j]->relative_loc = data_ins[in_cnt++].relative_loc;
        to_bank[0]->INs[j]->belong_Inst = N;
    }
    for(int j = 0 ; j < to_bank[0]->OUTs.size() ; j++)
    {
        to_bank[0]->OUTs[j]->pin_type = data_ins[out_cnt].pin_type;
        to_bank[0]->OUTs[j]->relative_loc = data_ins[out_cnt++].relative_loc;
        to_bank[0]->OUTs[j]->belong_Inst = N;
    }
    for(int i = 0 ; i < to_bank.size() ; i++)
    {
        to_bank[i]->CLK->pin_type = clk.pin_type;
        to_bank[i]->CLK->belong_Inst = N;
        to_bank[i]->CLK->relative_loc = clk.relative_loc;
    }
    N->Pins.push_back(to_bank[0]->CLK);
    N->CLK = to_bank[0]->CLK;

    N->idx = FF_list_bank.size();
    FF_list_bank.push_back(N);
    for(int i = 1 ; i < to_bank.size() ; i++)
    {
        delete to_bank[i]->get_root();
        delete to_bank[i];
    }
    slack_propagation(N);
    add_util(N);

    return N;
}

std::vector<Inst*> Plane_E::debank(Inst* to_debank, std::vector<Inst_data*> objectives)
{
    int idx = to_debank->idx;
    FF_list_bank[idx] = FF_list_bank.back();
    FF_list_bank[idx]->idx = idx;
    FF_list_bank.pop_back();
    min_util(to_debank);
    remove_Inst(to_debank);
    

    vector<Inst*> new_Insts;
    for(int i = 0 ; i < objectives.size() ; i++)
    {
        name_cnter++;
        T_Idx p = find_lib_by_name(objectives[i]->name);
        int idx = p.second;
        string name = "rEG_NEW_"+to_string(name_cnter);
        Point newLD = to_debank->LeftDown();
        new_Insts.push_back(new Inst(name,newLD,objectives[i],p,"NO PIN COPY"));
    }

    vector<Pin> data_ins,data_out;
    Pin clk;


    int de_cnt = 0;
    for(int i = 0 ; i < new_Insts.size() ; i++)
    {

        vector<Pin> data_ins,data_out;
        Pin clk;
        data_ins.reserve(new_Insts[i]->INs.size());
        data_out.reserve(new_Insts[i]->INs.size());
        for(int j = 0 ; j < objectives[i]->Pin_set.size() ; j++)
        {
            if(objectives[i]->Pin_set[j].type=='I')
                data_ins.push_back(objectives[i]->Pin_set[i]);
            else if(objectives[i]->Pin_set[j].type=='O')
                data_out.push_back(objectives[i]->Pin_set[j]);
            else
                clk = objectives[i]->Pin_set[j];
        }
        new_Insts[i]->Pins.resize(0);
        for(int j = 0 ; j < new_Insts[i]->INs.size() ; i++)
        {
            new_Insts[i]->OUTs[j] = pair_Q(to_debank,to_debank->INs[de_cnt]);
            new_Insts[i]->INs[j] = to_debank->INs[de_cnt++];

            new_Insts[i]->Pins.push_back(new_Insts[i]->OUTs[j]);
            new_Insts[i]->Pins.push_back(new_Insts[i]->INs[j]);

            new_Insts[i]->CLK = new_Insts[i]->INs[j]->belong_Inst->CLK;

            new_Insts[i]->CLK->belong_Inst = new_Insts[i];
            new_Insts[i]->CLK->relative_loc = clk.relative_loc;
            new_Insts[i]->CLK->pin_type = clk.pin_type;

            new_Insts[i]->OUTs[j]->belong_Inst = new_Insts[i];
            new_Insts[i]->OUTs[j]->relative_loc = data_out[j].relative_loc;
            new_Insts[i]->OUTs[j]->pin_type = data_out[j].pin_type;

            new_Insts[i]->INs[j]->belong_Inst = new_Insts[i];
            new_Insts[i]->INs[j]->relative_loc = data_ins[j].relative_loc;
            new_Insts[i]->INs[j]->pin_type = data_ins[j].pin_type;
        }
        new_Insts[i]->Pins.push_back(new_Insts[i]->CLK);
    }

    for(int i = 0 ; i < new_Insts.size() ; i++)
    {
        slack_propagation(new_Insts[i]);
        add_util(new_Insts[i]);
    }
    for(int i = 0 ; i < new_Insts.size() ; i++)
        slack_propagation(new_Insts[i]);
    

    for(int i = 1 ; i < new_Insts.size() ; i++)
    {
        new_Insts[i]->idx = FF_list_bank.size();
        FF_list_bank.push_back(new_Insts[i]);
    }
    delete to_debank->get_root();
    delete to_debank;

    return new_Insts;
}

