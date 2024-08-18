#include "Plane_extension.h"
#include <set>
#include <fstream>
#include<iomanip>
#define u_map_itr(T) unordered_map<std::string,T>::const_iterator
using namespace std;

void Plane_E::debank()
{
    for(net* contain_clk_net : clk_net_list)
    {
        same_domain_debanking(contain_clk_net);
    }
}

void Plane_E::bank()
{

    for(net* contain_clk_net : clk_net_list)
    {
        same_domain_banking(contain_clk_net);
    }
}

void Plane_E::same_domain_debanking(net* cur_domain)
{
    cout<<endl;
    cout<<"FF DEBANK"<<endl;
    cout<<endl;
    vector<Inst*> FFs = get_same_domain_FFs(cur_domain);
    for(int i = 0 ; i < FFs.size() ; i++)
    {
        cout<<FFs[i]->get_name()<<endl;
        for(Pin* p:FFs[i]->Pins)
            cout<<p->get_name()<<endl;
        cout<<endl;
        debank(FFs[i]);
    }
        
}

bool x_comp(Inst* a, Inst* b)
{
    return (a->LeftDown().x + a->corr_data->width/2) < (b->LeftDown().x + b->corr_data->width/2);
}

bool y_comp(Inst* a, Inst* b)
{
    return (a->LeftDown().y + a->corr_data->height/2) < (b->LeftDown().y + b->corr_data->height/2);
}

double dist(Inst* a,Inst* b)
{
    return abs(a->LeftDown().x + a->corr_data->width/2 - b->LeftDown().x - b->corr_data->width/2)\
           + abs(a->LeftDown().y + a->corr_data->height/2 - b->LeftDown().y - b->corr_data->height/2);
}

double Plane_E::effective_dist(Inst* master,Inst* slave)
{
    double len =  dist(master,slave);
    double displace_timing = 0;    //1 dim distance
    for(int i = 0 ; i < slave->INs.size() ; i++)
    {
        if(-slave->INs[i]->slack + displacement_delay* len / 1.414 >= 0)
            displace_timing += -slave->INs[i]->slack + displacement_delay* len / 1.414;
        else   
            displace_timing += -(slave->INs[i]->slack - displacement_delay* len / 1.414)/positive_slack;
    }
    displace_timing += displacement_delay* len* double(slave->OUTs.size()) / 1.414;
    return displace_timing;
}

void Plane_E::same_domain_banking(net* cur_domain)
{
    
    vector<Inst*> FFs = get_same_domain_FFs(cur_domain);
    for(auto&FF:FFs)
    {
        cout<<FF->get_name()<<endl;
    }
    vector<Inst*> FFs_x = FFs;
    sort(FFs_x.begin(),FFs_x.end(),x_comp);
    vector<Inst*> FFs_y = FFs;
    sort(FFs_x.begin(),FFs_x.end(),y_comp);
    vector<pair<Inst*,set<pair<double,Inst*>>>> k_mean_list;  //vector idx follows FFs_x
    for(Inst* FF:FFs_x)
        k_mean_list.push_back(make_pair(FF,set<pair<double,Inst*>>{}));
    
    cout<<FF_lib_bits.size()<<endl;
    for(int x_idx = 0 ; x_idx < k_mean_list.size() ; x_idx++)
    {
        int y_idx = -1;
        for(int t = 0 ; t < FFs_y.size() ; t++)
        {
            if(FFs_y[t] == FFs_x[x_idx])
            {
                y_idx = t;
                break;
            }
        }
        for(int i = x_idx+1 ; i <= x_idx+FF_lib_bits.size() && i < FFs_x.size() ; i++)
        {
            k_mean_list[x_idx].second.insert(make_pair(effective_dist(k_mean_list[x_idx].first,FFs_x[i]),FFs_x[i]));
        }
        for(int i = x_idx-1 ; i >= x_idx-FF_lib_bits.size() && i >= 0  ; i--)
        {
            k_mean_list[x_idx].second.insert(make_pair(effective_dist(k_mean_list[x_idx].first,FFs_x[i]),FFs_x[i]));
        }
        for(int i = y_idx+1 ; i <= y_idx+FF_lib_bits.size() && i < FFs_y.size() ; i++)
        {
            k_mean_list[x_idx].second.insert(make_pair(effective_dist(k_mean_list[x_idx].first,FFs_y[i]),FFs_y[i]));
        }
        for(int i = y_idx-1 ; i >= y_idx-FF_lib_bits.size() && i >= 0  ; i--)
        {
            k_mean_list[x_idx].second.insert(make_pair(effective_dist(k_mean_list[x_idx].first,FFs_y[i]),FFs_y[i]));
        }
        /*
        cout<<"distance with "<<k_mean_list[x_idx].first->get_name()<<endl;
        for(auto& ff_neighbor: k_mean_list[x_idx].second)
        {
            cout<<ff_neighbor.second->get_name()<<endl;
            cout<<ff_neighbor.first<<endl;

        }
        cout<<"END"<<endl;
        */
    }
    cout<<"BANKING"<<endl;
    for(int i = 0 ; i < k_mean_list.size() ; i++)
    {
        bool found = 0;
        Inst* FF = k_mean_list[i].first;
        set<pair<double,Inst*>> neighbor_set = k_mean_list[i].second;
        Inst* to_bank = neighbor_set.begin()->second;
        while(!(found || neighbor_set.empty()))
        {
            to_bank = neighbor_set.begin()->second;
            if(neighbor_set.begin()->first >= 50)
                break;
            for(int j = 0 ; j < k_mean_list.size() ; j++)
            {
                if(k_mean_list[j].first == to_bank)
                {
                    k_mean_list[j] = k_mean_list.back();
                    k_mean_list.pop_back();
                    found = 1;
                    break;
                }
            }
            neighbor_set.erase(neighbor_set.begin());
        }
        if(found)
            bank(vector<Inst*>{FF,to_bank});
        k_mean_list[i] = k_mean_list.back();
        k_mean_list.pop_back();
        i--;
    }

}

vector<Inst*> Plane_E::get_same_domain_FFs(net* cur)
{
    set<Inst*> List;
    for(Pin* p:cur->CLKs)
    {
        if(p->type == 'C')
        {
            List.insert(p->belong_Inst);
        }
    }
    vector<Inst*> v{make_move_iterator(begin(List)), make_move_iterator(end(List))};
    return v;
}

vector<Inst*> Plane_E::get_same_domain_FFs(Inst* cur)
{
    net* cur_net = cur->CLK[0]->belong_net;
    list<Inst*> List;
    for(Pin* p:cur_net->CLKs)
    {
        if(p->type == 'C')
        {
            List.push_back(p->belong_Inst);
        }
    }

    return vector<Inst*>{List.begin(),List.end()};
}


std::vector<Inst*> Plane_E::debank(Inst* to_debank)
{
    int total_bits = to_debank->INs.size();
    vector<Inst_data*> single_bits_ff_vec;
    single_bits_ff_vec.resize(total_bits,FF_lib_bits[1][0]);
    return debank(to_debank,single_bits_ff_vec);
}

Inst* Plane_E::bank(std::vector<Inst*> to_bank)
{
    int total_bits = 0;
    for(int i = 0 ; i < to_bank.size() ; i++)
    {
        total_bits += to_bank[i]->INs.size();
    }
    if(total_bits >= FF_lib_bits.size())    //FF_lib_bits[i] imply using i bits FF
        return NULL;

    return bank(to_bank, FF_lib_bits[total_bits][0]);
}

Inst* Plane_E::bank(std::vector<Inst*> to_bank, Inst_data* objective)
{
    for(int i = 0 ; i < to_bank.size() ; i++)
    {
        min_util(to_bank[i]);
        min_smooth_util(to_bank[i]);
        remove_Inst(to_bank[i]);
        cout<<*to_bank[i]->get_root()<<endl;
        cout<<to_bank[i]->get_name()<<endl;
        int idx = to_bank[i]->idx;
        FF_list_bank[idx] = FF_list_bank.back();
        FF_list_bank[idx]->idx = idx;
        FF_list_bank.pop_back();
    }

    FF_total_area += objective->height * objective->width;
    FF_total_power += objective->power;
    for(int i = 0 ; i < to_bank.size() ; i++)
    {
        FF_total_area -= to_bank[i]->corr_data->height * to_bank[i]->corr_data->width;
        FF_total_power -= to_bank[i]->corr_data->power;
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
            N->CLK.push_back(to_bank[i]->INs[j]->matching_Pin);
            N->Pins.push_back(to_bank[i]->INs[j]->matching_Pin);
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
        to_bank[0]->OUTs[j]->pin_type = data_out[out_cnt].pin_type;
        to_bank[0]->OUTs[j]->relative_loc = data_out[out_cnt++].relative_loc;
        to_bank[0]->OUTs[j]->belong_Inst = N;
    }
    for(int i = 0 ; i < to_bank.size() ; i++)
    {
        for(Pin* C: to_bank[i]->CLK)
        {
            C->pin_type = clk.pin_type;
            C->belong_Inst = N;
            C->relative_loc = clk.relative_loc;
        }
        to_bank[i]->CLK.clear();
    }


    N->idx = FF_list_bank.size();
    FF_list_bank.push_back(N);
    cout<<"DELETION"<<endl;
    for(int i = 0 ; i < to_bank.size() ; i++)
    {
        for(Pin* p:to_bank[i]->Pins)
            cout<<p->get_name()<<endl;
        cout<<*to_bank[i]->get_root()<<endl;
        cout<<to_bank[i]->get_name()<<endl;
        cout<<endl;
    }
    for(int i = 1 ; i < to_bank.size() ; i++)
    {
        cout<<*to_bank[i]->get_root()<<endl;
        cout<<to_bank[i]->get_name()<<endl;
        delete to_bank[i]->get_root();
        delete to_bank[i];
    }
    cout<<"RETURN"<<endl;
    slack_propagation(N);
    
    add_util(N);
    add_smooth_util(N);
    cout<<"RETURN"<<endl;
    return N;
}

std::vector<Inst*> Plane_E::debank(Inst* to_debank, std::vector<Inst_data*> objectives)
{
    int idx = to_debank->idx;
    FF_list_bank[idx] = FF_list_bank.back();
    FF_list_bank[idx]->idx = idx;
    FF_list_bank.pop_back();
    min_util(to_debank);
    min_smooth_util(to_debank);
    remove_Inst(to_debank);
    FF_total_area -= to_debank->corr_data->height * to_debank->corr_data->width;
    FF_total_power -= to_debank->corr_data->power;
    for(int i = 0 ; i < objectives.size() ; i++)
    {
        FF_total_area += objectives[i]->height * objectives[i]->width;
        FF_total_power += objectives[i]->power;
    }
    
    vector<Inst*> new_Insts;
    new_Insts.reserve(objectives.size());
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
        
        vector<Pin*> data_ins,data_out;
        Pin clk;
        data_ins.reserve(new_Insts[i]->INs.size());
        data_out.reserve(new_Insts[i]->INs.size());
        for(int j = 0 ; j < objectives[i]->Pin_set.size() ; j++)
        {
            
            if(objectives[i]->Pin_set[j].type=='I')
                data_ins.push_back(&objectives[i]->Pin_set[j]);
            else if(objectives[i]->Pin_set[j].type=='O')
            {
                data_out.push_back(&objectives[i]->Pin_set[j]);
            }
            else
                clk = objectives[i]->Pin_set[j];
        }
        for(int j = 0 ; j < objectives[i]->bits ; j++)
        {
            new_Insts[i]->OUTs[j] = pair_Q(to_debank,to_debank->INs[de_cnt]);
            new_Insts[i]->INs[j] = to_debank->INs[de_cnt++];

            new_Insts[i]->Pins.push_back(new_Insts[i]->OUTs[j]);
            new_Insts[i]->Pins.push_back(new_Insts[i]->INs[j]);
            new_Insts[i]->Pins.push_back(new_Insts[i]->INs[j]->matching_Pin);
            
            new_Insts[i]->CLK.push_back(new_Insts[i]->INs[j]->matching_Pin);
            
            new_Insts[i]->INs[j]->matching_Pin->belong_Inst = new_Insts[i];
            new_Insts[i]->INs[j]->matching_Pin->relative_loc = clk.relative_loc;
            new_Insts[i]->INs[j]->matching_Pin->pin_type = clk.pin_type;

            new_Insts[i]->OUTs[j]->belong_Inst = new_Insts[i];
            new_Insts[i]->OUTs[j]->relative_loc = data_out[j]->relative_loc;
            new_Insts[i]->OUTs[j]->pin_type = data_out[j]->pin_type;
            
            new_Insts[i]->INs[j]->belong_Inst = new_Insts[i];
            new_Insts[i]->INs[j]->relative_loc = data_ins[j]->relative_loc;
            new_Insts[i]->INs[j]->pin_type = data_ins[j]->pin_type;
        }
    }
    for(int i = 0 ; i < new_Insts.size() ; i++)
    {
        slack_propagation(new_Insts[i]);
        add_util(new_Insts[i]);
        add_smooth_util(new_Insts[i]);
    }
    for(int i = 0 ; i < new_Insts.size() ; i++)
        slack_propagation(new_Insts[i]);
    
    cout<<"DEBANK RESULT"<<endl;
    for(int i = 0 ; i < new_Insts.size() ; i++)
    {
        new_Insts[i]->idx = FF_list_bank.size();
        FF_list_bank.push_back(new_Insts[i]);
        cout<<new_Insts[i]->get_name()<<endl;
        for(Pin* p : new_Insts[i]->Pins)
            cout<<p->get_name()<<endl;
    }
    cout<<endl;
    delete to_debank->get_root();
    delete to_debank;

    return new_Insts;
}

