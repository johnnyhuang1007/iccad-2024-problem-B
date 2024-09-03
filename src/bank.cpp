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

void Plane_E::min_displacement_bank()
{

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

vector<pair<Inst*,set<pair<double,Inst*>>>> Plane_E::min_total_displacement_k_mean(net *cur_domain)
{
    vector<double> total_displace;
    total_displace.resize(FF_list_bank.size(),0);
    vector<Inst*> FFs = get_same_domain_FFs(cur_domain);
    vector<Inst*> FFs_x = FFs;
    sort(FFs_x.begin(),FFs_x.end(),x_comp);
    vector<Inst*> FFs_y = FFs;
    sort(FFs_x.begin(),FFs_x.end(),y_comp);
    vector<pair<Inst*,set<pair<double,Inst*>>>> k_mean_list;  //vector idx follows FFs_x
    for(Inst* FF:FFs_x)
        k_mean_list.push_back(make_pair(FF,set<pair<double,Inst*>>{}));
    
    cout<<"FF_lib_bits "<<FF_lib_bits.size()<<endl;
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
        cout<<y_idx<<endl;
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
    int bank_bit = min_cost_per_bit->bits;
    // get the total displacement of the bank_bits FFs
    for(int i = 0 ; i < k_mean_list.size() ; i++)
    {
        int cnt = k_mean_list[i].first->INs.size();
        for(auto& neighbor:k_mean_list[i].second)
        {
            if(cnt + neighbor.second->INs.size() >= bank_bit)
                break;
            total_displace[k_mean_list[i].first->idx] += neighbor.first;
            cnt += neighbor.second->INs.size();
        }
    }
    sort(k_mean_list.begin(),k_mean_list.end(),[total_displace](pair<Inst*,set<pair<double,Inst*>>>& a,pair<Inst*,set<pair<double,Inst*>>>& b){return total_displace[a.first->idx] < total_displace[b.first->idx];});
    cout<<"READY TO BANK"<<endl;
    return k_mean_list;
}

void Plane_E::same_domain_banking(net* cur_domain)
{
    
    vector<pair<Inst*,set<pair<double,Inst*>>>> k_mean_list = min_total_displacement_k_mean(cur_domain);
    vector<Inst*> still_exist(k_mean_list.size(),NULL);
    for(int i = 0 ; i < still_exist.size() ; i++)
        still_exist[i] = k_mean_list[i].first;
    vector<vector<Inst*>> to_banks(k_mean_list.size(),vector<Inst*>{});
    int bank_bit = min_cost_per_bit->bits;
    for(int i = 0 ; i < k_mean_list.size() ; i++)
    {
        auto itr = find(still_exist.begin(),still_exist.end(),k_mean_list[i].first);
        if(itr == still_exist.end())
            continue;
        still_exist.erase(itr);

        to_banks[i].push_back(k_mean_list[i].first);
        int cnt= k_mean_list[i].first->INs.size();
        for(auto& neighbor:k_mean_list[i].second)
        {
            if(cnt + neighbor.second->INs.size() > bank_bit)
                break;
            itr = find(still_exist.begin(),still_exist.end(),neighbor.second);
            if(itr == still_exist.end())
                continue;
            cnt += neighbor.second->INs.size();
            to_banks[i].push_back(neighbor.second);
            still_exist.erase(itr);
        }
    }
    cout<<"BANKING"<<endl;
    for(int i = 0 ; i < to_banks.size() ; i++)
    {
        cout<<"START"<<endl;
        if(to_banks[i].size() == 0)
            continue;
        int bits = 0;
        for(Inst* to_bank:to_banks[i])
            bits += to_bank->INs.size();
        if(bits >= FF_lib_bits.size())
            continue;
        cout<<"CURRENT BITS: "<<bits<<endl;

        if(FF_lib_bits[bits].size() == 0)
        {
            cout<<"SPECIAL CASE"<<endl;
            bool success = 0;
            //inverse list iteration
            for(int j = to_banks[i].size()-1 ; j >= 0 ; j--)
            {
                if(FF_lib_bits[bits - to_banks[i][j]->INs.size()].size() > 0)
                {
                    bits = bits - to_banks[i][j]->INs.size();
                    to_banks[i][j] = to_banks[i].back();
                    to_banks[i].pop_back();
                    cout<<"SUCCESS"<<   endl;
                    success = 1;
                    break;
                }
            }
            if(!success)
                continue;
        }
        cout<<"BANKING"<<endl;
        bank(to_banks[i],FF_lib_bits[bits][0]);
        cout<<"END"<<endl;
    }
    
    
    /*
    for(int i = 0 ; i < k_mean_list.size() ; i++)
    {
        int cur_bit = k_mean_list[i].first->INs.size();
        bool found = 0;
        Inst* FF = k_mean_list[i].first;
        set<pair<double,Inst*>> neighbor_set = k_mean_list[i].second;
        Inst* to_bank = neighbor_set.begin()->second;
        vector<Inst*> to_bank_list;
        to_bank_list.push_back(FF);
        while(!(cur_bit >= bank_bit || neighbor_set.empty()))
        {
            to_bank = neighbor_set.begin()->second;
            to_bank_list.push_back(to_bank);
            if(to_bank->INs.size() + cur_bit > bank_bit)
            {
                neighbor_set.erase(neighbor_set.begin());
                continue;
            }
            if(neighbor_set.begin()->first >= 50)
                break;
            for(int j = 0 ; j < k_mean_list.size() ; j++)
            {
                if(k_mean_list[j].first == to_bank)
                {
                    k_mean_list[j] = k_mean_list.back();
                    for(auto &neighbor:k_mean_list[j].second)
                    {
                        if( neighbor.second == FF)
                            continue;
                        neighbor_set.insert(neighbor);
                    }
                    k_mean_list.pop_back();
                    cur_bit += to_bank->INs.size();
                    break;
                }
            }
            neighbor_set.erase(neighbor_set.begin());
        }
        if(cur_bit > FF->INs.size())
            bank(to_bank_list);
        k_mean_list[i] = k_mean_list.back();
        k_mean_list.pop_back();
        i--;
    }
    */
   cout<<"END"<<endl;
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

void Plane_E::legality_look_ahead_banking()
{
    for(net* contain_clk_net : clk_net_list)
    {
        legality_look_ahead_banking(contain_clk_net);
    }
}

vector<Inst* > get_n_cloest_FF(vector<Inst*> to_banks,Tile tile, int bits)
{
    if(tile.coord[0].x != __INT_MAX__)
        sort(to_banks.begin(),to_banks.end(),[tile](Inst* a,Inst* b){
            return abs(a->LeftDown().x - tile.coord[0].x) + abs(a->LeftDown().y - tile.coord[0].y) < abs(b->LeftDown().x - tile.coord[0].x) + abs(b->LeftDown().y - tile.coord[0].y);
            });

    list<Inst*> ret;
    int cnt = 0;
    for(int i = 0 ; i < to_banks.size() ; i++)
    {
        cnt += to_banks[i]->INs.size();
        if(cnt > bits)
        {
            cnt -= to_banks[i]->INs.size();
            for(int j = i+1 ; j < to_banks.size() ; j++)
            {
                if(to_banks[j]->INs.size() + cnt == bits)
                {
                    ret.push_back(to_banks[j]);
                    cnt += to_banks[j]->INs.size();
                    break;
                }
            }
            cnt += to_banks[i]->INs.size();
            ret.push_back(to_banks[i]);
            for(auto rm : ret)
            {
                if(rm->INs.size() > to_banks[i]->INs.size())
                {
                    cnt -= rm->INs.size();
                    ret.remove(rm);
                    break;
                }
            }
        }
        else
            ret.push_back(to_banks[i]);
        if(cnt == bits)
            break;
    }
    return vector<Inst*>{ret.begin(),ret.end()};
}

vector<Tile> get_potential;

void Plane_E::legality_look_ahead_banking(net* cur_domain)
{
    vector<pair<Inst*,set<pair<double,Inst*>>>> k_mean_list = min_total_displacement_k_mean(cur_domain);

    vector<Inst*> still_exist(k_mean_list.size(),NULL);
    for(int i = 0 ; i < still_exist.size() ; i++)
        still_exist[i] = k_mean_list[i].first;
    
    vector<vector<Inst*>> to_banks(k_mean_list.size(),vector<Inst*>{});
    int bank_bit = min_cost_per_bit->bits;
    for(int i = 0 ; i < k_mean_list.size() ; i++)
    {
        auto itr = find(still_exist.begin(),still_exist.end(),k_mean_list[i].first);
        if(itr == still_exist.end())
            continue;
        
        cout<<"CURRENT FF: "<<k_mean_list[i].first->get_name()<<endl;

        //get the k nearest FFs
        to_banks[i].push_back(k_mean_list[i].first);
        int bit_max = FF_lib_bits.size()-1;
        int cur_bit = 0;
        for(auto& neighbor:k_mean_list[i].second)
        {
            if(cur_bit == bit_max)
                break;
            if(cur_bit + neighbor.second->INs.size() > bit_max)
                break;
            cur_bit+= neighbor.second->INs.size();

            itr = find(still_exist.begin(),still_exist.end(),neighbor.second);
            if(itr == still_exist.end())
                continue;
            cur_bit += neighbor.second->INs.size();
            to_banks[i].push_back(neighbor.second);
        }

        //search for usable tile
        

        //from all the potential insertable find the best loc
        double best_delta = 0;
        vector<Inst*> best_closest;
        Tile best_to_insert;
        Inst_data* to_type;
        Tile* start = point_finding(to_banks[i][0]->LeftDown());
        for(int BIT = FF_lib_bits.size()-1 ; BIT > 1 ; BIT--)
        {
            int bit_width = __INT_MAX__;
            int bit_height = __INT_MAX__;

            for(int j = 0 ; j < FF_lib_bits[BIT].size() ; j++)
            {
                if(FF_lib_bits[BIT][j]->width < bit_width)
                    bit_width = FF_lib_bits[BIT][j]->width;
                if(FF_lib_bits[BIT][j]->height < bit_height)
                    bit_height = FF_lib_bits[BIT][j]->height;  
            }
            if(FF_lib_bits[BIT].size() == 0)
                continue;
            
            

            cout<<"BIT: "<<BIT<<endl;
            vector<Inst*> closest = get_n_cloest_FF(to_banks[i],Tile(Point(__INT_MAX__,__INT_MAX__),Point(__INT_MAX__,__INT_MAX__)),BIT);
            cout<<"CLOSEST SIZE: "<<closest.size()<<endl;
            int true_bits = 0;
            for(auto closest_FF:closest)
                true_bits += closest_FF->INs.size();
            cout<<"TRUE BITS: "<<true_bits<<endl;
            if(true_bits != BIT)
                continue;

            for(auto rm : closest)
                remove_Inst(rm);
            //search for usable tile

            Tile max_size(Point(__INT_MAX__,__INT_MAX__),Point(-__INT_MAX__,-__INT_MAX__));
            for(auto FF: to_banks[i])
            {
                if(FF->LeftDown().x < max_size.coord[0].x)
                    max_size.coord[0].x = FF->LeftDown().x;
                if(FF->LeftDown().y < max_size.coord[0].y)
                    max_size.coord[0].y = FF->LeftDown().y;
                if(FF->LeftDown().x + (FF->corr_data->width/unit_move_x)+1 > max_size.coord[1].x)
                    max_size.coord[1].x = FF->LeftDown().x + (FF->corr_data->width/unit_move_x)+1;
                if(FF->LeftDown().y + (FF->corr_data->height/unit_move_y)+1 > max_size.coord[1].y)
                    max_size.coord[1].y = FF->LeftDown().y + (FF->corr_data->height/unit_move_y)+1;
            }
            max_size.coord[1].x += width(&max_size);
            if(max_size.coord[1].x > Width)
                max_size.coord[1].x = Width -1;
            max_size.coord[0].x -= width(&max_size);
            if(max_size.coord[0].x < 0)
                max_size.coord[0].x = 1;
            max_size.coord[1].y += height(&max_size);
            if(max_size.coord[1].y > Height)
                max_size.coord[1].y = Height -1;
            max_size.coord[0].y -= height(&max_size);
            if(max_size.coord[0].y < 0)
                max_size.coord[0].y = 1;
            vector<Tile*> space_list = getSpaceTileInRegion(&max_size);
            cout<<"SPACE LIST SIZE: "<<space_list.size()<<endl;
            space_list = remove_not_on_site(space_list);
            cout<<"SPACE LIST SIZE: "<<space_list.size()<<endl;
            list<Tile> potential_insertable = region_insertable(space_list,bit_width,bit_height); 
            cout<<"POTENTIAL INSERTABLE SIZE: "<<potential_insertable.size()<<endl;
            if(potential_insertable.size() == 0)
            {
                for(auto mk : closest)
                    insert_inst(mk);
                continue;
            }
            
            //e.o.s.
            for(auto& tile:potential_insertable)
            {
                double delta_bit = 0;
                vector<Inst*> closest_tile;
                Inst_data* to_type_in_tile;
                for(int j = 0 ; j <  FF_lib_bits[BIT].size() ; j++)
                {
                    /*
                    cout<<"TRY "<<FF_lib_bits[BIT][j]->name<<endl;
                    cout<<"WIDTH: "<<width(&tile)<<endl;
                    cout<<"HEIGHT: "<<height(&tile)<<endl;
                    cout<<"FF WIDTH: "<<FF_lib_bits[BIT][j]->width<<endl;
                    cout<<"FF HEIGHT: "<<FF_lib_bits[BIT][j]->height<<endl;
                    */
                    if(FF_lib_bits[BIT][j]->width > width(&tile) || FF_lib_bits[BIT][j]->height > height(&tile))
                        continue;
                    Tile psu(tile.coord[0],FF_lib_bits[BIT][j]->height,FF_lib_bits[BIT][j]->width);
                    double cost = 0;
                    for(auto closest_FF:closest)
                    {
                        cost -= beta * closest_FF->corr_data->power + gamma * closest_FF->corr_data->width * closest_FF->corr_data->height + alpha * closest_FF->INs.size() * closest_FF->corr_data->QpinDelay;
                    }
                    cost += beta * FF_lib_bits[BIT][j]->power + gamma * FF_lib_bits[BIT][j]->width * FF_lib_bits[BIT][j]->height + alpha * BIT * FF_lib_bits[BIT][j]->QpinDelay;
                    if(cost < delta_bit)
                    {
                        closest_tile = closest;
                        delta_bit = cost;
                        to_type_in_tile = FF_lib_bits[BIT][j];
                    }
                    break;
                }
                double time_cost = 0;
                for(auto FF:closest_tile)
                {
                    for(auto p: FF->INs)
                        time_cost += max({-p->slack + 2*alpha*(abs(tile.coord[0].x - FF->LeftDown().x) + abs(tile.coord[0].y - FF->LeftDown().y),0.0)});
                }
                if(time_cost + delta_bit < best_delta)
                {
                    cout<<"UPDATE"<<endl;
                    best_delta = time_cost;
                    best_closest = closest_tile;
                    best_to_insert = tile;
                    to_type = to_type_in_tile;
                    
                    cout<<to_type->bits<<endl;
                }
            }  
            for(auto mk : closest)
                insert_inst(mk);
        }

        if(best_closest.size() == 0)
        {
            cout<<"NO CLOSEST"<<endl;
            continue;
        }
        to_banks[i] = best_closest;
        for(auto rm : best_closest)
            remove_Inst(rm);
        cout<<"TRY TO BANK "<<to_banks[i].size()<<endl;
        for(int j = 0 ; j < to_banks[i].size() ; j++)
        {
            cout<<to_banks[i][j]->get_name()<<endl;
        }
        if(to_type == NULL)
        {
            cout<<"NO TYPE "<<best_delta<<endl;
            continue;
        }
        cout<<to_type->name<<endl;
        Inst* to_inst = bank(to_banks[i],to_type);
        cout<<"INSERT"<<endl;
        set_and_propagate(to_inst,best_to_insert.coord[0]);
        cout<<"TO INSERT LOC AND SIZE"<<endl;
        cout<<best_to_insert.coord[0]<<endl;
        cout<<best_to_insert.coord[1]<<endl;
        cout<<width(&best_to_insert)<<endl;
        cout<<height(&best_to_insert)<<endl;
        cout<<"CURRENT SIZE"<<endl;
        cout<<to_inst->get_root()->coord[0]<<endl;
        cout<<to_inst->get_root()->coord[1]<<endl;
        cout<<width(to_inst)<<endl;
        cout<<height(to_inst)<<endl;
        if(!insert_inst(to_inst))
            exit(1);
        cout<<"END"<<endl;
        for(auto to_erase : to_banks[i])
        {
            auto itr = find(still_exist.begin(),still_exist.end(),to_erase);
            if(itr != still_exist.end())
                still_exist.erase(itr);
        }
        cout<<"negative_slack   "<<negative_slack<<endl;
        cout<<"positive_slack   "<<positive_slack<<endl;
        cout<<"COST "<<cost()<<endl;
        cout<<"violated_bins_cnt    "<<violated_bins_cnt<<endl;
        cout<<"smoothen_bin_util    "<<smoothen_bin_util<<endl;

    }
    cout<<"END OF NET BANKING"<<endl;
    
}

list<Tile> Plane_E::region_insertable(vector<Tile*> space_list,int req_width,int req_height)
{
    int min_width = __INT_MAX__;
    int min_height = __INT_MAX__;
    for(int i = 0 ; i < FF_lib.size() ; i++)
    {
        if(FF_lib[i].width < min_width)
            min_width = FF_lib[i].width;
        if(FF_lib[i].height < min_height)
            min_height = FF_lib[i].height;
    }
    list<Tile> insertable;
    for(int i = 0 ; i < space_list.size() ; i++)
    {
        list<Tile> region = region_insertable(space_list[i],req_width,req_height);
        insertable.insert(insertable.end(), region.begin(), region.end());
    }
    
    return insertable;
}


list<Tile> Plane_E::region_insertable(vector<Tile*> space_list)
{
    int min_width = __INT_MAX__;
    int min_height = __INT_MAX__;
    for(int i = 0 ; i < FF_lib.size() ; i++)
    {
        if(FF_lib[i].width < min_width)
            min_width = FF_lib[i].width;
        if(FF_lib[i].height < min_height)
            min_height = FF_lib[i].height;
    }
    list<Tile> insertable;
    for(int i = 0 ; i < space_list.size() ; i++)
    {
        list<Tile> region = region_insertable(space_list[i],min_width,min_height);
        insertable.insert(insertable.end(), region.begin(), region.end());
    }
    
    return insertable;
}

#include<stack>
list<Tile> Plane_E::region_insertable(Tile* space,int min_width,int min_height)
{
    list<Tile> accepted_list;
	//include : the tile that is found must include this tile;
	//objective : the height/width of the tile
	stack<Tile> searching_list;
	searching_list.push(*space);
	while(!searching_list.empty())
	{
		Tile header = searching_list.top();
		searching_list.pop();
		Tile upper = Tile(Point(RU(&header).y+1,LD(&header).x), Point(RU(&header).y+1,RU(&header).x));
		vector<Tile*> neighbor = getSpaceTileInRegion(&upper,space);
		for(int i = 0 ; i < neighbor.size() ; i++)
		{
			Tile to_find = *neighbor[i];
			if(RU(&to_find).x > RU(&header).x)
				RU(&to_find).x = RU(&header).x;
			if(LD(&to_find).x < LD(&header).x)
				LD(&to_find).x = LD(&header).x;
            
			/*make a sudo tile*/
			if(width(&to_find) < min_width)
				continue;
            if(height(&to_find) >= min_height)
                accepted_list.push_back(to_find);
			searching_list.push(to_find);
		}
	}

    vector<Tile> accepted_list_vec{accepted_list.begin(),accepted_list.end()};
    //make the LD on site
    for( int i = 0 ; i < accepted_list_vec.size() ; i++)
    {
        auto candicate = accepted_list_vec[i];
        int y_idx = placement_row_idx(candicate.coord[0]);
        int x_idx = (candicate.coord[0].x - PlacementRows[y_idx].left_down.x)/PlacementRows[y_idx].siteWidth;


        if(candicate.coord[0].y != PlacementRows[y_idx].left_down.y && y_idx < PlacementRows.size()-1)
            y_idx++;
        
        if(candicate.coord[0].y > PlacementRows.back().left_down.y)
        {
            accepted_list_vec[i] = accepted_list.back();
            accepted_list_vec.pop_back();
            i--;
            continue;
        }
        if(candicate.coord[0].y < PlacementRows.front().left_down.y)
        {
            accepted_list_vec[i] = accepted_list.back();
            accepted_list_vec.pop_back();
            i--;
            continue;
        }
        if(x_idx < 0 || x_idx >= PlacementRows[y_idx].count)
        {
            accepted_list_vec[i] = accepted_list.back();
            accepted_list_vec.pop_back();
            i--;
            continue;
        }
        x_idx += (candicate.coord[0].x - PlacementRows[y_idx].left_down.x)%PlacementRows[y_idx].siteWidth == 0;
        candicate.coord[0].x = PlacementRows[y_idx].left_down.x + x_idx*PlacementRows[y_idx].siteWidth;
        candicate.coord[0].y = PlacementRows[y_idx].left_down.y;
    }

    //remove inclusion tiles
    
    
    for(int i = 0 ; i < accepted_list_vec.size() ; i++)
    {
        for(int j = 0 ; j < accepted_list_vec.size() ; j++)
        {
            if(i == j)
                continue;
            if(accepted_list_vec[i].coord[0].x <= accepted_list_vec[j].coord[0].x && accepted_list_vec[i].coord[1].x >= accepted_list_vec[j].coord[1].x && accepted_list_vec[i].coord[0].y <= accepted_list_vec[j].coord[0].y && accepted_list_vec[i].coord[1].y >= accepted_list_vec[j].coord[1].y)
            {
                accepted_list_vec[j] = accepted_list.back();
                accepted_list_vec.pop_back();
                j--;
            }
        }
    }
    
    return list<Tile>(accepted_list_vec.begin(),accepted_list_vec.end());
}

vector<Tile*> Plane_E::remove_not_on_site(vector<Tile*> space_list)
{
    for(int i = 0 ; i < space_list.size() ; i++)
    {
        int y_idx = placement_row_idx(space_list[i]->coord[0]);
        if(space_list[i]->coord[0].y != PlacementRows[y_idx].left_down.y)
        {
            space_list[i] = space_list.back();
            space_list.pop_back();
            i--;
            continue;
        }
        if(space_list[i]->coord[1].x <= PlacementRows[y_idx].left_down.x)
        {
            space_list[i] = space_list.back();
            space_list.pop_back();
            i--;
            continue;
        }
        if(space_list[i]->coord[0].x > PlacementRows[y_idx].left_down.x + PlacementRows[y_idx].siteWidth * (PlacementRows[y_idx].count-1))
        {
            space_list[i] = space_list.back();
            space_list.pop_back();
            i--;
            continue;
        }
    }
    cout<<"READY TO RETURN" <<endl;
    return space_list;
}