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
    return max({(min(r1.x, r2.x) - max(l1.x, l2.x))*(min(r1.y, r2.y) - max(l1.y, l2.y)),0});
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
            
            smoothen_bin_util -= pow(max(0.0,Bins[i][j].smoothen_area - bin_area * BinMaxUtil / 100.0),2);
            Bins[i][j].smoothen_area += const_val*unit_areas_vec[cnt++];
            smoothen_bin_util += pow(max(0.0,Bins[i][j].smoothen_area - bin_area * BinMaxUtil / 100.0),2);
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
            smoothen_bin_util -= pow(max(0.0,Bins[i][j].smoothen_area - bin_area * BinMaxUtil / 100.0),2);
            Bins[i][j].smoothen_area -= const_val*unit_areas_vec[cnt++];
            smoothen_bin_util += pow(max(0.0,Bins[i][j].smoothen_area - bin_area * BinMaxUtil / 100.0),2);
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
