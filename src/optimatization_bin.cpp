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

int overlappingArea(Bin bin, Inst* inst)
{
    Point l1 = inst->LeftDown();
    Point r1 = inst->get_root()->coord[1] + Point(1,1);
    Point l2 = bin.left_down;
    Point r2 = bin.left_down + Point(bin.height,bin.width);
    return max({(min(r1.x, r2.x) - max(l1.x, l2.x))*(min(r1.y, r2.y) - max(l1.y, l2.y)),0});
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
                if(p->critical_weight <= 0)
                    p->critical_weight = 0;
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
                if(p->critical_weight <= 0)
                    p->critical_weight = 0;
            }
    }
}
