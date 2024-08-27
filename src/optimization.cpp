#include "Plane_extension.h"
#include <fstream>
#include<iomanip>
#define u_map_itr(T) unordered_map<std::string,T>::const_iterator
using namespace std;

void Plane_E::move_and_propagate(Inst* cur, Point movement)
{
    min_util(cur);
    min_smooth_util(cur);
    cur->set_new_loc(cur->LeftDown() + movement);
    add_util(cur);
    add_smooth_util(cur);
    vector<double> d_slack{0,0};
    for(int i = 0 ; i < cur->INs.size() ; i++)
    {
        vector<double> tmp = cur->INs[i]->slack_propagation();
        d_slack[0] += tmp[0];
        d_slack[1] += tmp[1];
    }
    negative_slack += d_slack[0];
    positive_slack += d_slack[1];
}


void Plane_E::set_and_propagate(Inst* cur, Point loc)
{
    min_util(cur);
    min_smooth_util(cur);
    cur->set_new_loc(loc);
    add_util(cur);
    add_smooth_util(cur);
    vector<double> d_slack{0,0};
    for(int i = 0 ; i < cur->INs.size() ; i++)
    {
        vector<double> tmp = cur->INs[i]->slack_propagation();
        d_slack[0] += tmp[0];
        d_slack[1] += tmp[1];
    }
    negative_slack += d_slack[0];
    positive_slack += d_slack[1];
}

Point Plane_E::next_on_site_move(Inst* cur,string dir,int step)
{
    int y_idx  = 0;
    Point curp = cur->LeftDown();
    if(curp.y >= PlacementRows.back().left_down.y)
        y_idx = PlacementRows.size()-2;
    else if(curp.y <= PlacementRows[0].left_down.y)
        y_idx = 0;
    else
    {
        int move = (PlacementRows.size()+1)/2;

        while(!(PlacementRows[y_idx].left_down.y <= curp.y && PlacementRows[y_idx+1].left_down.y >= curp.y))
        {

            if(PlacementRows[y_idx].left_down.y >= curp.y)
                y_idx -= move;
            else
                y_idx += move;
            move = (move+1)/2;
        }
    }

    Point newP;
    if(dir == "UP")
    {
        if(PlacementRows.size()-1 <= y_idx)
            return cur->LeftDown();
        if(y_idx+step >= PlacementRows.size())
            y_idx = PlacementRows.size() - step - 1;
        newP.y = PlacementRows[y_idx+step].left_down.y;
        int x_idx = (curp.x - PlacementRows[y_idx+step].left_down.x ) / PlacementRows[y_idx+step].siteWidth;
        if(curp.x - PlacementRows[y_idx+step].left_down.x - x_idx * PlacementRows[y_idx+step].siteWidth >= PlacementRows[y_idx+step].siteWidth/2)
            x_idx++;
        if(x_idx == PlacementRows[y_idx+step].count)
            x_idx--;
        newP.x = PlacementRows[y_idx+step].left_down.x + x_idx * PlacementRows[y_idx+step].siteWidth;
        if(newP.y > Height || newP.y < 0)
        {
            cout<<"WRONG CASE OCCUR "<<y_idx<<endl;
            exit(1);
        }
    }
    else if(dir == "DOWN")
    {
        if(0 >= y_idx)
            y_idx = step;
        if(y_idx-step < 0)
            y_idx = step;
        newP.y = PlacementRows[y_idx-step].left_down.y;
        int x_idx = (curp.x - PlacementRows[y_idx-step].left_down.x ) / PlacementRows[y_idx-step].siteWidth;
        if(curp.x - PlacementRows[y_idx-step].left_down.x - x_idx * PlacementRows[y_idx-step].siteWidth >= PlacementRows[y_idx-step].siteWidth/2)
            x_idx++;
        if(x_idx == PlacementRows[y_idx-step].count)
            x_idx--;
        newP.x = PlacementRows[y_idx-step].left_down.x + x_idx * PlacementRows[y_idx-step].siteWidth;
        if(newP.y > Height || newP.y < 0)
        {
            cout<<"WRONG CASE OCCUR "<<y_idx<<endl;
            exit(1);
        }
    }
    else if(dir == "LEFT")
    {
        if(y_idx < 0)
            y_idx = 0;
        if(y_idx >= PlacementRows.size())
            y_idx = PlacementRows.size()-1;
        int x_idx = (curp.x - PlacementRows[y_idx].left_down.x ) / PlacementRows[y_idx].siteWidth;
        if(x_idx == 0)
            return cur->LeftDown();
        if(x_idx - step <= 0)
            x_idx = 0;
        else
            x_idx -= step;
        newP.y = PlacementRows[y_idx].left_down.y;
        newP.x = PlacementRows[y_idx].left_down.x + PlacementRows[y_idx].siteWidth * x_idx;
    }
    else
    {
        if(y_idx < 0)
            y_idx = 0;
        if(y_idx >= PlacementRows.size())
            y_idx = PlacementRows.size()-1;
        int x_idx = (curp.x - PlacementRows[y_idx].left_down.x ) / PlacementRows[y_idx].siteWidth;
        if(x_idx >= PlacementRows[y_idx].count)
            return cur->LeftDown();
        
        if(x_idx + step >= PlacementRows[y_idx].count)
            x_idx = PlacementRows[y_idx].count - 1;
        else
            x_idx += step;
        newP.y = PlacementRows[y_idx].left_down.y;
        newP.x = PlacementRows[y_idx].left_down.x + PlacementRows[y_idx].siteWidth * x_idx;
    }
    return newP;
}

void Plane_E::unit_move_and_propagate(Inst* cur,string dir,int step)  //"UP" "DOWN" "LEFT" "RIGHT"
{

    Point newP = next_on_site_move(cur,dir,step);
    min_util(cur);
    min_smooth_util(cur);
    cur->set_new_loc(newP);
    add_util(cur);
    add_smooth_util(cur);
    vector<double> d_slack{0,0};
    for(int i = 0 ; i < cur->INs.size() ; i++)
    {
        vector<double> tmp = cur->INs[i]->slack_propagation();
        d_slack[0] += tmp[0];
        d_slack[1] += tmp[1];
    }
    negative_slack += d_slack[0];
    positive_slack += d_slack[1];
}


void Plane_E::propagate(Inst* cur)  
{
    
}

void Plane_E::sequence_finder()
{
    for(int i = 0 ; i < FF_list_bank.size() ; i++)
    {
        sequence_finder(FF_list_bank[i]);
    }
}

void Plane_E::sequence_finder(Inst* FF)
{
    
}


double Plane_E::set_on_site()
{
    for(Inst* FF:FF_list_bank)
    {
        set_and_propagate(FF,closest_Legal_locs(FF->LeftDown()));
    }

    cout<<"negative_slack   "<<negative_slack<<endl;
    cout<<"positive_slack   "<<positive_slack<<endl;
    cout<<"COST "<<cost()<<endl;
    cout<<"violated_bins_cnt    "<<violated_bins_cnt<<endl;
    cout<<"smoothen_bin_util    "<<smoothen_bin_util<<endl;
    cout<<endl;
    return negative_slack;
}

double collect_error = 0;

double Plane_E::slack_optimizer(int step)
{
    //cout<<"INITIAL SLACK: "<<slack<<endl;
    bool check1 = 0,check2 = 0;
    double prev_n_slack = negative_slack;
    double prev_p_slack = positive_slack;
    double T_val = (prev_p_slack - prev_n_slack)*(prev_p_slack - prev_n_slack);
    vector<Point> movements;
    movements.reserve(FF_list_bank.size());
    int check = rand()%FF_list_bank.size();
    
    
    for(int i = 0 ; i < FF_list_bank.size() ; i++)
    { 
        double prev_neg_slack = negative_slack;
        double prev_pos_slack = positive_slack;
        double prev_smooth_util = smoothen_bin_util;
        Point prev = FF_list_bank[i]->LeftDown();
        int dir = rand()%4;
        unit_move_and_propagate(FF_list_bank[i],DIRS[dir],step);
        if(0.1*(-negative_slack+prev_neg_slack)/prev_neg_slack + 0.99 * (prev_smooth_util - smoothen_bin_util)/prev_smooth_util >= 0)
            continue;
            
        set_and_propagate(FF_list_bank[i],prev);
    }

    cout<<"negative_slack   "<<negative_slack<<endl;
    cout<<"positive_slack   "<<positive_slack<<endl;
    cout<<"COST "<<cost()<<endl;
    cout<<"violated_bins_cnt    "<<violated_bins_cnt<<endl;
    cout<<"smoothen_bin_util    "<<smoothen_bin_util<<endl;
    cout<<endl;
    return negative_slack;
}

bool isnan(Point p)
{
    return (p.x != p.x) || (p.y != p.y);
}

void Plane_E::update_slack_pin_weight(double WEIGHT)
{
    for(int i = 0 ; i < FF_list_bank.size() ; i++)
    {
        for(int j = 0 ; j < FF_list_bank[i]->INs.size() ; j++)
        {
            if(FF_list_bank[i]->INs[j]->slack < 0)
            {
                for(Pin* cur: FF_list_bank[i]->INs[j]->path_seq)
                {
                    cur->critical_weight -= FF_list_bank[i]->INs[j]->slack/(-negative_slack+0.0000001) * WEIGHT;
                }
            }
        }
    }
}


double Plane_E::HPWL_optimizer()
{

    
    for(int i = 0 ; i < FF_list_bank.size() ; i++)
    {
        for(int j = 0 ; j < FF_list_bank[i]->INs.size() ; j++)
        {
            FF_list_bank[i]->INs[j]->belong_net->set_weight_center();
        }
        for(int j = 0 ; j < FF_list_bank[i]->OUTs.size() ; j++)
        {
            FF_list_bank[i]->OUTs[j]->belong_net->set_weight_center();
        }
        Point P(0,0);
        double Px = 0;
        double Py = 0;
        double wgt = 0;
        for(int j = 0 ; j < FF_list_bank[i]->INs.size() ; j++)
        {
            if(FF_list_bank[i]->INs[j]->belong_net->FROMs.size()==0)
                continue;
            Px = Px + FF_list_bank[i]->INs[j]->belong_net->center_of_FROMs.x * FF_list_bank[i]->INs[j]->belong_net->FROMs_weight;
            Py = Py + FF_list_bank[i]->INs[j]->belong_net->center_of_FROMs.y * FF_list_bank[i]->INs[j]->belong_net->FROMs_weight;
            wgt += FF_list_bank[i]->INs[j]->belong_net->FROMs_weight;
        }
        for(int j = 0 ; j < FF_list_bank[i]->OUTs.size()  ; j++)
        {
            if(FF_list_bank[i]->OUTs[j]->belong_net->TOs.size()==0)
                continue;
            Px = Px + FF_list_bank[i]->OUTs[j]->belong_net->center_of_TOs.x * FF_list_bank[i]->OUTs[j]->belong_net->TOs_weight;
            Py = Py + FF_list_bank[i]->OUTs[j]->belong_net->center_of_TOs.y * FF_list_bank[i]->OUTs[j]->belong_net->TOs_weight;
            wgt += FF_list_bank[i]->OUTs[j]->belong_net->TOs_weight;
        }
        P.x = Px/wgt;
        P.y = Py/wgt;
        if(wgt <= 0)
            continue;
        move_and_propagate(FF_list_bank[i],P - FF_list_bank[i]->LeftDown());   
    }
    


    cout<<"negative_slack   "<<negative_slack<<endl;
    cout<<"positive_slack   "<<positive_slack<<endl;
    cout<<"COST "<<cost()<<endl;
    cout<<"violated_bins_cnt    "<<violated_bins_cnt<<endl;
    cout<<"smoothen_bin_util    "<<smoothen_bin_util<<endl;
    cout<<endl;
    return 0;
}

void Plane_E::set_to_placementRow()
{

    
    for(int i = 0 ; i <FF_list_bank.size() ; i++)
    {
        vector<double> d_slack{0,0};
        set_to_placementRow(FF_list_bank[i]);
        for(int j = 0 ; j < FF_list_bank[i]->INs.size() ; j++)
        {
            vector<double> tmp = FF_list_bank[i]->INs[j]->slack_propagation();
            d_slack[0] += tmp[0];
            d_slack[1] += tmp[1];
        }
        negative_slack += d_slack[0];
        positive_slack += d_slack[1];
    }
    
    cout<<negative_slack<<endl;
}

void Plane_E::set_to_placementRow(Inst* cur)
{
    int curidx = 0;
    Point newPoint(0,0);
    for(int i = 0 ; i < PlacementRows.size()-1 ; i++)
    {
        curidx = i;
        if(cur->LeftDown().y >= PlacementRows[i].left_down.y && cur->LeftDown().y < PlacementRows[i+1].left_down.y)
        {
            break;
        }
    }
    if(abs(cur->LeftDown().y - PlacementRows[curidx].left_down.y) > abs(cur->LeftDown().y - PlacementRows[curidx+1].left_down.y))
        curidx+=1;
    newPoint.y = PlacementRows[curidx].left_down.y;
    int curidxX = int(cur->LeftDown().x - PlacementRows[curidx].left_down.x) / PlacementRows[curidx].siteWidth;
    if(newPoint.x - (PlacementRows[curidx].left_down.x + PlacementRows[curidx].siteWidth * curidxX) > PlacementRows[curidx].siteWidth/2)
        curidxX++;
    curidxX = min({curidxX,PlacementRows[curidx].count});
    newPoint.x = PlacementRows[curidx].left_down.x + PlacementRows[curidx].siteWidth * curidxX;

    
    cur->set_new_loc(newPoint);
    add_util(cur);
    add_smooth_util(cur);
}

void Plane_E::loc_randomize()
{
    cout<<"FF LOC RANDOMIZE"<<endl;

    for(int i = 0 ; i < FF_list_bank.size() ;i++)
    {
        vector<double> d_slack{0.0,0.0};
        min_util(FF_list_bank[i]);
        min_smooth_util(FF_list_bank[i]);
        FF_list_bank[i]->set_new_loc(random());
        add_util(FF_list_bank[i]);
        add_smooth_util(FF_list_bank[i]);
        for(int j = 0 ; j < FF_list_bank[i]->INs.size() ; j++)
        {
            vector<double> tmp = FF_list_bank[i]->INs[j]->slack_propagation();
            d_slack[0] += tmp[0];
            d_slack[1] += tmp[1];
        }
        negative_slack += d_slack[0];
        positive_slack += d_slack[1];
        
    }

}