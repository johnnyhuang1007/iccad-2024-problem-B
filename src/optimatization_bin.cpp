#include "Plane_extension.h"
#include <fstream>
#include<iomanip>
#define u_map_itr(T) unordered_map<std::string,T>::const_iterator
using namespace std;


double overlappingArea(Inst* i1,Inst* i2)
{
    Point l1 = i1->LeftDown();
    Point r1 = i1->get_root()->coord[1] + Point(1.0,1.0);
    Point l2 = i2->LeftDown();
    Point r2 = i2->get_root()->coord[1] + Point(1.0,1.0);
    if(min(r1.x, r2.x) - max(l1.x, l2.x) <= 0 )
        return 0;
    if(min(r1.y, r2.y) - max(l1.y, l2.y) <= 0 )
        return 0;
    return (min(r1.x, r2.x) - max(l1.x, l2.x))*(min(r1.y, r2.y) - max(l1.y, l2.y));
}

double overlappingArea_smoothen(Bin bin, Inst* inst)    //unit area
{
    Point l1 = inst->LeftDown();
    Point r1 = inst->get_root()->coord[1] + Point(1,1);
    Point l2 = bin.left_down;
    Point r2 = bin.left_down + Point(bin.height,bin.width);

    double center_bin_x = double(l2.x + r2.x)/2.0;
    double center_bin_y = double(l2.y + r2.y)/2.0;

    double center_inst_x = double(l1.x + l2.x)/2.0;
    double center_inst_y = double(l2.y + l2.y)/2.0;

    double dx = abs(center_inst_x - center_bin_x);
    double dy = abs(center_inst_y - center_bin_y);

    double a = 4.0 / ( (double(width(inst)) + 2.0 * double(bin.width)) * (double(width(inst)) + 4.0 * double(bin.width)) );
    double b = 2.0 / ( double(bin.width) * (double(width(inst)) + 4.0 * double(bin.width)) );

    double px;
    if (0 <= dx && dx <= double(width(inst))/2.0 + bin.width)
        px = 1.0 - a*dx*dx;
    else if(dx >= double(width(inst))/2.0 + 2.0*bin.width)
        px = 0;
    else
        px = b*(dx - double(width(inst))/2.0 - 2.0*bin.width)*(dy - double(height(inst))/2.0 - 2.0*bin.height);

    a = 4.0 / ( (double(height(inst)) + 2.0 * double(bin.height)) * (double(height(inst)) + 4.0 * double(bin.height)) );
    b = 2.0 / ( double(bin.height) * (double(height(inst)) + 4.0 * double(bin.height)) );

    double py;
    if (0 <= dy && dy <= double(height(inst))/2.0 + bin.height)
        py = 1.0 - a*dy*dy;
    else if(dy >= double(height(inst))/2.0 + 2.0*bin.height)
        py = 0;
    else
        py = b*(dy - double(height(inst))/2.0 - 2.0*bin.height)*(dy - double(height(inst))/2.0 - 2.0*bin.height);

    return px*py;
}

int overlappingArea(Bin bin, Inst* inst)
{
    Point l1 = inst->LeftDown();
    Point r1 = inst->get_root()->coord[1] + Point(1,1);
    Point l2 = bin.left_down;
    Point r2 = bin.left_down + Point(bin.height,bin.width);
    int lenx = min(r1.x, r2.x) - max(l1.x, l2.x);
    int leny = min(r1.y, r2.y) - max(l1.y, l2.y);
    if(lenx <= 0 || leny <= 0)
        return 0;
    return lenx*leny;
}

double util(Bin cur)
{
    return double(cur.used_area)/double(cur.height*cur.width);
}

void Plane_E::add_util(Inst* cur)
{
    Point LEFTDOWN = cur->LeftDown();
    Point RIGHTUP = cur->RightUp();
    int idxL = LEFTDOWN.x/BinWidth;
    int idxR = RIGHTUP.x/BinWidth;
    int idxU = RIGHTUP.y/BinHeight;
    int idxD = LEFTDOWN.y/BinHeight;
    for(int i = idxD ; (i <= idxU) && (i <= (Bins.size()-1)) ; i++)
    {
        for(int j = idxL ; (j <= idxR) && (j <= (Bins[i].size()-1)) ; j++ )
        {
            long pre_area = Bins[i][j].used_area;
            Bins[i][j].used_area += overlappingArea(Bins[i][j],cur);
            if(double(Bins[i][j].used_area)/double(Bins[i][j].height*Bins[i][j].width) >= BinMaxUtil/100.0 \
               && double(pre_area)/double(Bins[i][j].height*Bins[i][j].width) <= BinMaxUtil/100.0)
                violated_bins_cnt++;
        }
    }

}



void Plane_E::min_util(Inst* cur)
{
    Point LEFTDOWN = cur->LeftDown();
    Point RIGHTUP = cur->RightUp();
    int idxL = LEFTDOWN.x/BinWidth;
    int idxR = RIGHTUP.x/BinWidth;
    int idxU = RIGHTUP.y/BinHeight;
    int idxD = LEFTDOWN.y/BinHeight;
    for(int i = idxD ; (i <= idxU) && (i <= (Bins.size()-1)) ; i++)
    {
        for(int j = idxL ; (j <= idxR) && (j < Bins[i].size()) ; j++ )
        {
            long pre_area = Bins[i][j].used_area;
            Bins[i][j].used_area -= overlappingArea(Bins[i][j],cur);
            if(double(Bins[i][j].used_area)/double(Bins[i][j].height*Bins[i][j].width) <= BinMaxUtil/100.0 \
               && double(pre_area)/double(Bins[i][j].height*Bins[i][j].width) >= BinMaxUtil/100.0)
                violated_bins_cnt--;
        }
    }
}

void Plane_E::add_smooth_util(Inst* cur)
{
    Point LEFTDOWN = cur->LeftDown();
    Point RIGHTUP = cur->RightUp();
    int idxL = LEFTDOWN.x/BinWidth-2;
    int idxR = RIGHTUP.x/BinWidth+2;
    int idxU = RIGHTUP.y/BinHeight+2;
    int idxD = LEFTDOWN.y/BinHeight-2;
    list<double> unit_areas;
    for(int i = max(idxD,0) ; (i <= idxU) && (i <= (Bins.size()-1)) ; i++)
        for(int j = max(idxL,0) ; (j <= idxR) && (j <= (Bins[i].size()-1)) ; j++ )
            unit_areas.push_back(overlappingArea_smoothen(Bins[i][j],cur));

    vector<double> unit_areas_vec;
    double tot_area = 0;
    unit_areas_vec.resize(unit_areas.size());
    int cnt = 0;
    for(auto& unit:unit_areas)
    {
        unit_areas_vec[cnt++] = unit;
        tot_area += unit;
    }
    cnt = 0;
    double const_val = double(height(cur)*width(cur))/tot_area;
    double bin_area = Bins[0][0].height * Bins[0][0].width;
    for(int i = max(idxD,0) ; (i <= idxU) && (i <= (Bins.size()-1)) ; i++)
    {
        for(int j = max(idxL,0) ; (j <= idxR) && (j <= (Bins[i].size()-1)) ; j++ )
        {
            smoothen_bin_util -= pow(max(0.0,Bins[i][j].smoothen_area - bin_area * BinMaxUtil / 400.0),2);
            Bins[i][j].smoothen_area += const_val*unit_areas_vec[cnt++];
            smoothen_bin_util += pow(max(0.0,Bins[i][j].smoothen_area - bin_area * BinMaxUtil / 400.0),2);
        }
    }
}

void Plane_E::min_smooth_util(Inst* cur)
{
    Point LEFTDOWN = cur->LeftDown();
    Point RIGHTUP = cur->RightUp();
    int idxL = LEFTDOWN.x/BinWidth-2;
    int idxR = RIGHTUP.x/BinWidth+2;
    int idxU = RIGHTUP.y/BinHeight+2;
    int idxD = LEFTDOWN.y/BinHeight-2;
    list<double> unit_areas;
    for(int i = max(idxD,0) ; (i <= idxU) && (i <= (Bins.size()-1)) ; i++)
        for(int j = max(idxL,0) ; (j <= idxR) && (j <= (Bins[i].size()-1)) ; j++ )
            unit_areas.push_back(overlappingArea_smoothen(Bins[i][j],cur));

    vector<double> unit_areas_vec;
    double tot_area = 0;
    unit_areas_vec.resize(unit_areas.size());
    int cnt = 0;
    for(auto& unit:unit_areas)
    {
        unit_areas_vec[cnt++] = unit;
        tot_area += unit;
    }
    cnt = 0;
    double const_val = double(height(cur)*width(cur))/tot_area;
    double bin_area = Bins[0][0].height * Bins[0][0].width;
    for(int i = max(idxD,0) ; (i <= idxU) && (i <= (Bins.size()-1)) ; i++)
    {
        for(int j = max(idxL,0) ; (j <= idxR) && (j <= (Bins[i].size()-1)) ; j++ )
        {
            smoothen_bin_util -= pow(max(0.0,Bins[i][j].smoothen_area - bin_area * BinMaxUtil / 400.0),2);
            Bins[i][j].smoothen_area -= const_val*unit_areas_vec[cnt++];
            smoothen_bin_util += pow(max(0.0,Bins[i][j].smoothen_area - bin_area * BinMaxUtil / 400.0),2);
            if(smoothen_bin_util < 0)
            {
                cout<<"ERROR"<<endl;
                exit(1);
            }
        }
    }
}

pair<pair<int,int>,pair<int,int>> Plane_E::occupied_bin_idx(Inst* cur)
{
    Point LEFTDOWN = cur->LeftDown();
    Point RIGHTUP = cur->RightUp();
    int idxL = LEFTDOWN.x/BinWidth;
    int idxR = RIGHTUP.x/BinWidth;
    int idxU = RIGHTUP.y/BinHeight;
    int idxD = LEFTDOWN.y/BinHeight;
    return pair<pair<int,int>,pair<int,int>>(make_pair(idxL,idxR),make_pair(idxD,idxU));
}

bool Plane_E::is_in_highly_used_bin(Inst* cur)
{
    Point LEFTDOWN = cur->LeftDown();
    Point RIGHTUP = cur->RightUp();
    int idxL = LEFTDOWN.x/BinWidth;
    int idxR = RIGHTUP.x/BinWidth;
    int idxU = RIGHTUP.y/BinHeight;
    int idxD = LEFTDOWN.y/BinHeight;
    for(int i = idxD ; (i <= idxU) && (i <= (Bins.size()-1)) ; i++)
    {
        for(int j = idxL ; (j <= idxR) && (j <= (Bins[i].size()-1)) ; j++ )
        {
            if(double(Bins[i][j].used_area)/double(Bins[i][j].height*Bins[i][j].width) >= BinMaxUtil/100.0)
                return 1;
        }
    }
    return 0;
}

void Plane_E::reduce_high_util_pin_weight()
{
    for(Inst* cur : G_list)
    {
        if(is_in_highly_used_bin(cur))
            for(Pin* p : cur->Pins)
                p->critical_weight +=1.0/double(violated_bins_cnt)*5.0;
        else
            for(Pin* p : cur->Pins)
            {
                p->critical_weight -=1.0/double(Bins[0].size() * Bins.size() - violated_bins_cnt)*5.0;
                if(p->critical_weight <= 0.01)
                    p->critical_weight = 0.01;
            }
    }
    for(Inst* cur : FF_list_bank)
    {
        if(is_in_highly_used_bin(cur))
            for(Pin* p : cur->Pins)
                p->critical_weight +=1.0/double(violated_bins_cnt)*5.0;
        else
            for(Pin* p : cur->Pins)
            {
                p->critical_weight -=1.0/double(Bins[0].size() * Bins.size() - violated_bins_cnt)*5.0;
                if(p->critical_weight <= 0.01)
                    p->critical_weight = 0.01;
            }
    }
}

int abs(Point cur)
{
	return abs(cur.x) + abs(cur.y);
}

//abort
void Plane_E::repulsing_force_based_bin_optimizer()
{

    vector<Inst*> FFs_x = FF_list_bank;
    vector<Inst*> FFs_y = FF_list_bank;

    
    
    FFs_x.reserve(FF_list_bank.size() + G_list.size());
    FFs_y.reserve(FF_list_bank.size() + G_list.size());
    for(Inst* Gate : G_list)
    {
        FFs_x.push_back(Gate);
        FFs_y.push_back(Gate);
    }
    
    sort(FFs_x.begin() , FFs_x.end(),[](Inst* a, Inst* b){return a->center().x < b->center().x; });
    sort(FFs_y.begin() , FFs_y.end(),[](Inst* a, Inst* b){return a->center().y < b->center().y; });
    cout<<"SORT DONE"<<endl;
    vector<pair<int,int>> FFs_seqence;
    FFs_seqence.resize(FF_list_bank.size(),pair<int,int>(-1,-1));
    for(int i = 0 ; i < FFs_x.size() ; i++)
    {
        if(FFs_x[i]->is_gate())
            continue;
        FFs_seqence[FFs_x[i]->idx].first = i;
    }
    for(int i = 0 ; i < FFs_y.size() ; i++)
    {
        if(FFs_y[i]->is_gate())
            continue;
        FFs_seqence[FFs_y[i]->idx].second = i;
    }
    cout<<"PAIRING DONE"<<endl;


    int region_y = Bins[0][0].height*2;
    int region_x = Bins[0][0].width*2;

    
    double unit_area = FF_lib[0].height * FF_lib[0].width;
    for(auto& lib : FF_lib)
    {
        if(lib.height * lib.width < unit_area)
            unit_area = lib.height * lib.width;
    }

    for(auto& lib : G_lib)
    {
        if(lib.height * lib.width < unit_area)
            unit_area = lib.height * lib.width;
    }

    vector<pair<double,double>> repulseing_forces;
    vector<pair<double,double>> spring_forces; 
    repulseing_forces.resize(FF_list_bank.size(), pair<double,double>(0,0));
    spring_forces.resize(FF_list_bank.size(), pair<double,double>(0,0));
    for(int i = 0 ; i < FF_list_bank.size() ; i++)
    {
        int x_idx = FFs_seqence[FF_list_bank[i]->idx].first;
        int y_idx = FFs_seqence[FF_list_bank[i]->idx].second;
        set<Inst*> force_to_cal;
        for(int j = x_idx+1 ; j < FFs_x.size() && FFs_x[j]->center().x <= FFs_x[x_idx]->center().x + region_x ; j++)
        {
            if(FFs_x[j]->center().y > FF_list_bank[i]->center().y + region_y)
                continue;
            if(FFs_x[j]->center().y < FF_list_bank[i]->center().y - region_y)
                continue;
            force_to_cal.insert(FFs_x[j]);
        }

        for(int j = x_idx-1 ; j >= 0 && FFs_x[j]->center().x >= FFs_x[x_idx]->center().x - region_x ; j--)
        {
            if(FFs_x[j]->center().y > FF_list_bank[i]->center().y + region_y)
                continue;
            if(FFs_x[j]->center().y < FF_list_bank[i]->center().y - region_y)
                continue;
            force_to_cal.insert(FFs_x[j]);
        }

        for(int j = y_idx ; j < FFs_y.size() && FFs_y[j]->center().y <= FFs_y[y_idx]->center().y + region_y ; j++)
        {
            if(FFs_y[j]->center().x > FF_list_bank[i]->center().x + region_x)
                continue;
            if(FFs_y[j]->center().x < FF_list_bank[i]->center().x - region_x)
                continue;
            force_to_cal.insert(FFs_y[j]);
        }

        for(int j = y_idx ; j >= 0 && FFs_y[j]->center().y >= FFs_y[y_idx]->center().y - region_y ; j--)
        {
            if(FFs_y[j]->center().x > FF_list_bank[i]->center().x + region_x)
                continue;
            if(FFs_y[j]->center().x < FF_list_bank[i]->center().x - region_x)
                continue;
            force_to_cal.insert(FFs_y[j]);
        }
        double repulse_force_x = 0;
        double repulse_force_y = 0;
        //cout<<"SIZE OF NEAREST : "<<force_to_cal.size()<<endl;
        for(Inst* near_ff : force_to_cal)
        {
            if(near_ff->center() == FF_list_bank[i]->center())
                continue;
            double dist_part = pow(abs(FF_list_bank[i]->center() - near_ff->center()),3);
            double charge_part = FF_list_bank[i]->corr_data->height * FF_list_bank[i]->corr_data->width * near_ff->corr_data->height * near_ff->corr_data->width;
            charge_part = charge_part;
            repulse_force_x += (charge_part/dist_part) * (near_ff->center() - FF_list_bank[i]->center()).x;
            repulse_force_y += (charge_part/dist_part) * (near_ff->center() - FF_list_bank[i]->center()).y;
            /*
            cout<<FF_list_bank[i]->center()<<endl;
            cout<<near_ff->center()<<endl;
            cout<<dist_part<<endl;
            cout<<charge_part<<endl;
            cout<<repulse_force_x<<endl;
            cout<<repulse_force_y<<endl;
            */
        }
        /*
        double spring_force_x = 0;
        double spring_force_y = 0;
        for(int j = 0 ; j < FF_list_bank[i]->INs.size() ; j++)
        {
            FF_list_bank[i]->INs[j]->belong_net->set_weight_center();
        }
        for(int j = 0 ; j < FF_list_bank[i]->OUTs.size() ; j++)
        {
            FF_list_bank[i]->OUTs[j]->belong_net->set_weight_center();
        }
        for(int j = 0 ; j < FF_list_bank[i]->INs.size() ; j++)
        {
            if(FF_list_bank[i]->INs[j]->belong_net->FROMs.size()==0)
                continue;
            spring_force_x += (FF_list_bank[i]->INs[j]->belong_net->center_of_FROMs.x - FF_list_bank[i]->INs[j]->abs_loc().x) * FF_list_bank[i]->INs[j]->belong_net->FROMs_weight;
            spring_force_y += (FF_list_bank[i]->INs[j]->belong_net->center_of_FROMs.y - FF_list_bank[i]->INs[j]->abs_loc().y) * FF_list_bank[i]->INs[j]->belong_net->FROMs_weight;
        }
        for(int j = 0 ; j < FF_list_bank[i]->OUTs.size()  ; j++)
        {
            if(FF_list_bank[i]->OUTs[j]->belong_net->TOs.size()==0)
                continue;
            spring_force_x += (FF_list_bank[i]->OUTs[j]->belong_net->center_of_TOs.x - FF_list_bank[i]->OUTs[j]->abs_loc().x) * FF_list_bank[i]->OUTs[j]->belong_net->TOs_weight;
            spring_force_y += (FF_list_bank[i]->OUTs[j]->belong_net->center_of_TOs.y - FF_list_bank[i]->OUTs[j]->abs_loc().y) * FF_list_bank[i]->OUTs[j]->belong_net->TOs_weight;
        }

        spring_forces[FF_list_bank[i]->idx] = pair<double,double>(spring_force_x,spring_force_y);
        */
       
        repulseing_forces[FF_list_bank[i]->idx] = pair<double,double>(repulse_force_x,repulse_force_y);
    }
    
    /*
    double Factor_r = 0;
    double Factor_s = 0;
    for(auto& f:repulseing_forces)
        Factor_r += abs(f.first) + abs(f.second);
    for(auto& f:spring_forces)
        Factor_s += abs(f.first) + abs(f.second);

    if(Factor_r > Factor_s)
        for(auto& f:repulseing_forces)
        {
            f.first*= (Factor_s/Factor_r);
            f.second*=(Factor_s/Factor_r);
        }
    else
        for(auto& f:spring_forces)
        {
            f.first*= (Factor_r/Factor_s);
            f.second*=(Factor_r/Factor_s);
        }

    double max_step = -1;
    vector<Point> movements;
    movements.resize(FF_list_bank.size(),Point(0,0));
    for(int i = 0 ; i < movements.size() ; i++)
    {
        movements[i].x = repulseing_forces[i].first + spring_forces[i].first;
        movements[i].y = repulseing_forces[i].second + spring_forces[i].second;

        if(movements[i].x > max_step)
            max_step = movements[i].x;
        if(movements[i].y > max_step)
            max_step = movements[i].x;
    }
    */
    double max_step = -1;
    vector<Point> movements;
    movements.resize(FF_list_bank.size(),Point(0,0));
    for(int i = 0 ; i < movements.size() ; i++)
    {
        movements[i].x = repulseing_forces[i].first *1000;
        movements[i].y = repulseing_forces[i].second *1000;

        if(movements[i].x > max_step)
            max_step = movements[i].x;
        if(movements[i].y > max_step)
            max_step = movements[i].x;
    }
    double max_displace = min(Bins[0][0].height/4,Bins[0][0].width/4);
    if(max_step > max_displace)
    {
        for(int i = 0 ; i < movements.size() ; i++)
        {
            movements[i].x = double(movements[i].x)*max_displace/max_step;
            movements[i].y = double(movements[i].x)*max_displace/max_step;
        }
    }
    
    for(int i = 0 ; i < FF_list_bank.size() ; i++)
    {
        move_and_propagate(FF_list_bank[i],movements[FF_list_bank[i]->idx]);
    }
    cout<<"negative_slack   "<<negative_slack<<endl;
    cout<<"positive_slack   "<<positive_slack<<endl;
    cout<<"COST "<<cost()<<endl;
    cout<<"violated_bins_cnt    "<<violated_bins_cnt<<endl;
    cout<<"smoothen_bin_util    "<<smoothen_bin_util<<endl;
    cout<<endl;
}
