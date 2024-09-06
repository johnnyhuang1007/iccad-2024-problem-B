#include "Plane_extension.h"
#include <fstream>
#include<iomanip>
#define u_map_itr(T) unordered_map<std::string,T>::const_iterator
using namespace std;

bool slack_compare(Inst* a, Inst* b)
{
    double slack1 = 0;
    double slack2 = 0;
    for(Pin* p:a->INs)
        slack1 += p->slack;
    for(Pin* p:b->INs)
        slack2 += p->slack;
    return slack1 < slack2;
}

bool Plane_E::is_on_site(Point cur)
{
    for(int i = 0 ; i < PlacementRows.size() ; i++)
    {
        if(PlacementRows[i].left_down.y == cur.y)
        {
            if((cur.x - PlacementRows[i].left_down.x)%PlacementRows[i].siteWidth != 0)
                return 0;
            if((cur.x - PlacementRows[i].left_down.x)/PlacementRows[i].siteWidth >= PlacementRows[i].count)
                return 0;
            if(cur.x < PlacementRows[i].left_down.x)
                return 0;
            return 1;
        }
    }
    return 1;
}

bool Plane_E::is_on_site(Inst* cur)
{
    for(int i = 0 ; i < PlacementRows.size() ; i++)
    {
        if(PlacementRows[i].left_down.y == cur->LeftDown().y)
        {
            if((cur->LeftDown().x - PlacementRows[i].left_down.x)%PlacementRows[i].siteWidth != 0)
                return 0;
            if((cur->LeftDown().x - PlacementRows[i].left_down.x)/PlacementRows[i].siteWidth >= PlacementRows[i].count)
                return 0;
            if(cur->LeftDown().x < PlacementRows[i].left_down.x)
                return 0;
            return 1;
        }
    }
    return 1;
}

void Plane_E::robust_slack_optimizer(int step)  //it makes the move only if the next result is legal, and the solution is better
{
    //cout<<"INITIAL SLACK: "<<slack<<endl;
    bool check1 = 0,check2 = 0;
    double prev_n_slack = negative_slack;
    double prev_p_slack = positive_slack;
    double T_val = (prev_p_slack - prev_n_slack)*(prev_p_slack - prev_n_slack);

    int check = rand()%FF_list_bank.size();
    vector<Inst*> FFs_x = FF_list_bank;
    vector<Inst*> FFs_y = FF_list_bank;
    sort(FFs_x.begin(),FFs_x.end(),[](Inst* a, Inst* b){return a->LeftDown().x < b->LeftDown().x;});
    sort(FFs_y.begin(),FFs_y.end(),[](Inst* a, Inst* b){return a->LeftDown().y < b->LeftDown().y;});
    vector<int> x_idx_map(FFs_x.size());
    vector<int> y_idx_map(FFs_y.size());
    for(int i = 0 ; i < FFs_x.size() ; i++)
    {
        x_idx_map[FFs_x[i]->idx] = i;
    }
    for(int i = 0 ; i < FFs_y.size() ; i++)
    {
        y_idx_map[FFs_y[i]->idx] = i;
    }
    //randomize FF_list
    
    for(int i = 0 ; i < FF_list_bank.size() ; i++)
    { 
        double prev_neg_slack = negative_slack;
        double prev_pos_slack = positive_slack;
        double prev_smooth_util = smoothen_bin_util;
        Point prev = FF_list_bank[i]->LeftDown();
        
        int choice = rand()%35;
        if(choice == 0)
        {
            list<int> idx_l;
            for(int j = x_idx_map[FF_list_bank[i]->idx]+1 ; j < FF_list_bank.size() && FFs_x[j]->LeftDown().x <= FF_list_bank[i]->LeftDown().x + Width/20 ; j++)
                idx_l.push_back(FFs_x[j]->idx);
            for(int j = x_idx_map[FF_list_bank[i]->idx]-1 ; j >= 0 && FFs_x[j]->LeftDown().x >= FF_list_bank[i]->LeftDown().x - Width/20 ; j--)
                idx_l.push_back(FFs_x[j]->idx);
            for(int j = y_idx_map[FF_list_bank[i]->idx]+1 ; j < FF_list_bank.size() && FFs_y[j]->LeftDown().y <= FF_list_bank[i]->LeftDown().y + Height/20 ; j++)
                idx_l.push_back(FFs_y[j]->idx);
            for(int j = y_idx_map[FF_list_bank[i]->idx]-1 ; j >= 0 && FFs_y[j]->LeftDown().y >= FF_list_bank[i]->LeftDown().y - Height/20 ; j--)
                idx_l.push_back(FFs_y[j]->idx);
            vector<int> idxs{idx_l.begin(),idx_l.end()};
            sort(idxs.begin(),idxs.end(),[this,i](int a, int b){
                return abs(FF_list_bank[i]->LeftDown().x - FF_list_bank[a]->LeftDown().x) + abs(FF_list_bank[i]->LeftDown().y - FF_list_bank[a]->LeftDown().y) < abs(FF_list_bank[i]->LeftDown().x - FF_list_bank[b]->LeftDown().x) + abs(FF_list_bank[i]->LeftDown().y - FF_list_bank[b]->LeftDown().y);});
            
            double prev_cost = cost();
            double prev_pos = positive_slack;
            Point prev1 = FF_list_bank[i]->LeftDown();
            remove_Inst(FF_list_bank[i]);
            for(int j = 0 ; j < 10 && j < idxs.size() ; j++)
            {
                int idx = idxs[j];
                Point prev2 = FF_list_bank[idx]->LeftDown();
                remove_Inst(FF_list_bank[idx]);
                set_and_propagate(FF_list_bank[i],prev2);
                set_and_propagate(FF_list_bank[idx],prev1);
                if(insert_inst(FF_list_bank[i]) && insert_inst(FF_list_bank[idx]))
                {
                    if(cost() < prev_cost || (positive_slack >= prev_pos && cost() <= prev_cost))
                    {
                        swap(FFs_x[x_idx_map[FF_list_bank[i]->idx]],FFs_x[x_idx_map[FF_list_bank[idx]->idx]]);
                        swap(FFs_y[y_idx_map[FF_list_bank[i]->idx]],FFs_y[y_idx_map[FF_list_bank[idx]->idx]]);
                        swap(x_idx_map[FF_list_bank[i]->idx],x_idx_map[FF_list_bank[idx]->idx]);
                        swap(y_idx_map[FF_list_bank[i]->idx],y_idx_map[FF_list_bank[idx]->idx]);
                        break;
                    }
                    remove_Inst(FF_list_bank[i]);
                    remove_Inst(FF_list_bank[idx]);
                    set_and_propagate(FF_list_bank[i],prev1);
                    set_and_propagate(FF_list_bank[idx],prev2);
                    insert_inst(FF_list_bank[idx]);
                }
                else
                {
                    remove_Inst(FF_list_bank[i]);
                    remove_Inst(FF_list_bank[idx]);
                    set_and_propagate(FF_list_bank[i],prev1);
                    set_and_propagate(FF_list_bank[idx],prev2);
                    insert_inst(FF_list_bank[idx]);
                }
            }
            insert_inst(FF_list_bank[i]);
            
        }
        else
        {
            int dir = rand()%4;
            if(prev.y < 0 || prev.y > Height)
            {
                cout<<prev<<endl;
                exit(1);
            }
            int S = rand()%step + 1;
            bool success = 0;
            for(int STEP = S ; STEP >= max(1, S-15) ; STEP--)
            {
                //cout<<STEP<<endl;
                Point net_LD = next_on_site_move(FF_list_bank[i],DIRS[dir],STEP);
                Tile next_Placement = Tile(net_LD,net_LD + Point(height(FF_list_bank[i]->get_root()) - 1,width(FF_list_bank[i]->get_root()) - 1));
                Tile* start;
                if(!FF_list_bank[i]->inserted)
                {
                    start = G_list[rand()%G_list.size()]->get_root();
                }
                else
                    start = FF_list_bank[i]->get_root();
                //cout<<"CHECK "<< next_Placement<<endl;
                if(!checkAllSpace(&next_Placement,start))
                {
                    continue;
                }
                else
                {
                    success =1;
                    S = STEP;
                    break;
                }
            }   
            if(!success)
            {
                continue;
            }
            double prev_cost = cost();
            remove_Inst(FF_list_bank[i]);
            unit_move_and_propagate(FF_list_bank[i],DIRS[dir],S);
            insert_inst(FF_list_bank[i]);

            //maintain order
            if(cost() <= prev_cost)
            {
                int cur_x_idx = x_idx_map[FF_list_bank[i]->idx];
                int cur_y_idx = y_idx_map[FF_list_bank[i]->idx];
                while(cur_x_idx > 0 && FFs_x[cur_x_idx]->LeftDown().x < FFs_x[cur_x_idx-1]->LeftDown().x)
                {
                    swap(FFs_x[cur_x_idx],FFs_x[cur_x_idx-1]);
                    swap(x_idx_map[FFs_x[cur_x_idx]->idx],x_idx_map[FFs_x[cur_x_idx-1]->idx]);
                    cur_x_idx--;
                }
                while(cur_x_idx < FFs_x.size()-1 && FFs_x[cur_x_idx]->LeftDown().x > FFs_x[cur_x_idx+1]->LeftDown().x)
                {
                    swap(FFs_x[cur_x_idx],FFs_x[cur_x_idx+1]);
                    swap(x_idx_map[FFs_x[cur_x_idx]->idx],x_idx_map[FFs_x[cur_x_idx+1]->idx]);
                    cur_x_idx++;
                }
                while(cur_y_idx > 0 && FFs_y[cur_y_idx]->LeftDown().y < FFs_y[cur_y_idx-1]->LeftDown().y)
                {
                    swap(FFs_y[cur_y_idx],FFs_y[cur_y_idx-1]);
                    swap(y_idx_map[FFs_y[cur_y_idx]->idx],y_idx_map[FFs_y[cur_y_idx-1]->idx]);
                    cur_y_idx--;
                }
                while(cur_y_idx < FFs_y.size()-1 && FFs_y[cur_y_idx]->LeftDown().y > FFs_y[cur_y_idx+1]->LeftDown().y)
                {
                    swap(FFs_y[cur_y_idx],FFs_y[cur_y_idx+1]);
                    swap(y_idx_map[FFs_y[cur_y_idx]->idx],y_idx_map[FFs_y[cur_y_idx+1]->idx]);
                    cur_y_idx++;
                }
                continue;
            }
            remove_Inst(FF_list_bank[i]);
            set_and_propagate(FF_list_bank[i],prev);
            insert_inst(FF_list_bank[i]);
            continue;
        }
    }

    cout<<"negative_slack   "<<negative_slack<<endl;
    cout<<"positive_slack   "<<positive_slack<<endl;
    cout<<"COST "<<cost()<<endl;
    cout<<"violated_bins_cnt    "<<violated_bins_cnt<<endl;
    cout<<"smoothen_bin_util    "<<smoothen_bin_util<<endl;
    cout<<endl;
    return ;//negative_slack;
}

void Plane_E::location_legalization(vector<Inst*> to_fix)
{
    
    sort(to_fix.begin(),to_fix.end(),slack_compare);
    int cnt = 0;
    Tile max_region = Tile(to_fix[0]->get_root());
    for(int i = 0 ; i < to_fix.size() ; i++)
    {
        Inst* FF = to_fix[i];
        //cout<<cnt++<<endl;
        //cout<<negative_slack<<endl;
        //cout<<FF->idx<<endl;
        //cout<<FF->get_name()<<endl; 
        if(insert_inst(FF) || FF->inserted)
        {
            continue;
        }
        //cout<<"FAIL"<<endl;
        Tile pseudo_tile = Tile(FF->get_root());
        if(width(&max_region) > width(&pseudo_tile))
        {
            int width_diff = (width(&max_region) - width(&pseudo_tile))/2;
            LD(&pseudo_tile).x -= width_diff;
            RU(&pseudo_tile).x += width_diff;
            if(RU(&pseudo_tile).x > Width -1)
                RU(&pseudo_tile).x = Width -1;
            if(LD(&pseudo_tile).x < 1)
                LD(&pseudo_tile).x = 1;
        }
        if(height(&max_region) > height(&pseudo_tile))
        {
            int height_diff = (height(&max_region) - height(&pseudo_tile))/2;
            LD(&pseudo_tile).y -= height_diff;
            RU(&pseudo_tile).y += height_diff;
            if(RU(&pseudo_tile).y > Height -1)
                RU(&pseudo_tile).y = Height -1;
            if(LD(&pseudo_tile).y < 1)
                LD(&pseudo_tile).y = 1;
        }
        //cout<<"REQURIED SIZE"   <<endl;
        //cout<<pseudo_tile<<endl;

        bool inserted = 0;
        vector<Tile> possible_tiles;
        vector<Tile*> space_vec;
        Tile* start = point_finding(FF->LeftDown());
        find:
        while(!inserted)
        {
            
            //cout<<"SEARECHING REGION"<<endl;
            //cout<<pseudo_tile<<endl;
            start = point_finding(LD(&pseudo_tile),start);
            vector<Tile*> solid_vec = getSolidTileInRegion(&pseudo_tile,start);
            //find min size to include all the solid
            //cout<<"REGION SOLID"<<endl;
            for(auto& solid : solid_vec)
            {
                //cout<<*solid<<endl;
                if(RU(solid).x > RU(&pseudo_tile).x)
                    RU(&pseudo_tile).x = RU(solid).x;
                if(RU(solid).y > RU(&pseudo_tile).y)
                    RU(&pseudo_tile).y = RU(solid).y;
                if(LD(solid).x < LD(&pseudo_tile).x)
                    LD(&pseudo_tile).x = LD(solid).x;
                if(LD(solid).y < LD(&pseudo_tile).y)
                    LD(&pseudo_tile).y = LD(solid).y;
            }
            int delta_height = (width(&pseudo_tile) - height(&pseudo_tile))/2;
            if(width(&pseudo_tile) > height(&pseudo_tile))
            {
                LD(&pseudo_tile).y -= delta_height;
                RU(&pseudo_tile).y += delta_height;
            }
            if(RU(&pseudo_tile).x < Width -1)
                RU(&pseudo_tile).x++;
            else
                RU(&pseudo_tile).x = Width -1;
            if(RU(&pseudo_tile).y < Height -1)
                RU(&pseudo_tile).y++;
            else
                RU(&pseudo_tile).y = Height -1;
            if(LD(&pseudo_tile).x > 1)  //TOFIX 
                LD(&pseudo_tile).x--;
            else
                LD(&pseudo_tile).x = 1;
            if(LD(&pseudo_tile).y > 1)
                LD(&pseudo_tile).y--;
            else
                LD(&pseudo_tile).y = 1;
            //cout<<"SEARECHING REGION"<<endl;
            
            //cout<<pseudo_tile<<endl;
            start = point_finding(LD(&pseudo_tile),start);
            vector<Tile*> region_spaces = getSpaceTileInRegion(&pseudo_tile,start);
            //cout<<pseudo_tile<<endl;
            //cout<<"REGION SPACE"<<endl;
            for(auto& space : region_spaces)
            {
                if(width(space) > width(&pseudo_tile) * 2)
                    continue;
                //cout<<*space<<endl;
                if(RU(space).x > RU(&pseudo_tile).x)
                    RU(&pseudo_tile).x = RU(space).x;
                if(RU(space).y > RU(&pseudo_tile).y)
                    RU(&pseudo_tile).y = RU(space).y;
                if(LD(space).x < LD(&pseudo_tile).x)
                    LD(&pseudo_tile).x = LD(space).x;
                if(LD(space).y < LD(&pseudo_tile).y)
                    LD(&pseudo_tile).y = LD(space).y;
            }
            delta_height = (width(&pseudo_tile) - height(&pseudo_tile))/2;
            if(width(&pseudo_tile) > height(&pseudo_tile))
            {
                LD(&pseudo_tile).y -= delta_height;
                RU(&pseudo_tile).y += delta_height;
            }
            
            if(RU(&pseudo_tile).x < Width -1)
                RU(&pseudo_tile).x++;
            else
                RU(&pseudo_tile).x = Width -1;
            if(RU(&pseudo_tile).y < Height -1)
                RU(&pseudo_tile).y++;
            else
                RU(&pseudo_tile).y = Height -1;
            if(LD(&pseudo_tile).x > 1)  //TOFIX 
                LD(&pseudo_tile).x--;
            else
                LD(&pseudo_tile).x = 1;
            if(LD(&pseudo_tile).y > 1)
                LD(&pseudo_tile).y--;
            else
                LD(&pseudo_tile).y = 1;
            //space_vec = region_spaces or space_vec;
            list<Tile*> to_search;
            //to_search = region_spaces exclude space_vec;
            for(auto& space : region_spaces)
            {
                bool is_in = 0;
                for(auto& space2 : space_vec)
                {
                    if(space == space2)
                    {
                        is_in = 1;
                        break;
                    }
                }
                if(!is_in)
                    to_search.push_back(space);
            }
            for(auto& space : to_search)
            {
                //cout<<"SEARCHING"<<endl;
                //cout<<*space<<endl;
                Tile to_push = findUsableRect(space, FF->get_root());
                //cout<<"RESULT"<<endl;
                //cout<<to_push<<endl;
                if(RU(&to_push).x != __INT_MAX__)
                {
                    inserted = 1;
                    possible_tiles.push_back(to_push);
                }
            }
        }
        inserted = 0;
        double dist = __DBL_MAX__;
        Tile* best_tile = NULL;
        //sort by distance between FF and tile
        sort(possible_tiles.begin(),possible_tiles.end(),\
        [FF](Tile a, Tile b){return (abs(a.coord[0].x - FF->LeftDown().x) + abs(a.coord[0].y - FF->LeftDown().y)) < (abs(b.coord[0].x - FF->LeftDown().x) + abs(b.coord[0].y - FF->LeftDown().y));});
        //for(Tile t:possible_tiles)
        //{
        //    cout<<t<<endl;
        //}
        bool success = 0;
        Point org = FF->LeftDown();
        double prev_cost = __DBL_MAX__;
        Point best;
        for(Tile pos : possible_tiles)
        {
            //cout<<"TRYING: "<<pos<<endl;
            //cout<<width(FF->get_root())<<endl;
            //cout<<height(FF->get_root())<<endl;
            Point to_insert_pos = min_displacement_loc(FF,&pos);//condition and region  //use packing method and need to be legal
            //cout<<"to_insert_pos "<<to_insert_pos<<endl;
            if(to_insert_pos.x > Width)
                continue;
            //cout<<"INSERTION"  <<endl;
            set_and_propagate(FF,to_insert_pos);
            //cout<<"INSERTION";
            if(insert_inst(FF))
            {
                if(cost() < prev_cost)
                {
                    prev_cost = cost();
                    best = to_insert_pos;
                }
                success = 1;
                //cout<<" SUCCESS"<<endl;
                remove_Inst(FF);
            }
            //cout<<" FAIL"<<endl;
        }
        if(!success)
        {
            set_and_propagate(FF,org);
            goto find;
        }
        else
        {
            set_and_propagate(FF,best);
            insert_inst(FF);
        }
        if(width(&pseudo_tile)/1000 > width(&max_region))
        {
            LD(&max_region).x = 0;
            RU(&max_region).x = (RU(&pseudo_tile).x - LD(&pseudo_tile).x)/1000;
        }
        if(height(&pseudo_tile)/1000 > height(&max_region))
        {
            LD(&max_region).y = 0;
            RU(&max_region).y = (RU(&pseudo_tile).y - LD(&pseudo_tile).y)/1000;
        }
    }

}


Point Plane_E::min_displacement_loc(Inst* condition, Tile* region)
{
   // cout<<"REGION "<<*region<<endl;
   // cout<<"CONDITION "<<*condition->get_root()<<endl;
    //cout<<"Width "<<width(condition)<<endl;
    //cout<<"Height "<<height(condition)<<endl;
    Point cur = condition->LeftDown();
    Point RU = condition->get_root()->coord[1];
    Point region_LD = region->coord[0];
    Point region_RU = region->coord[1];
    int LD_yidx = placement_row_idx(region_LD);
    int RU_yidx = placement_row_idx(region_RU - Point(height(condition) - 1,0));
    while(PlacementRows[LD_yidx].left_down.y < region_LD.y && LD_yidx < PlacementRows.size()-1)
    {
        LD_yidx++;
    }
    if(PlacementRows[RU_yidx+1].left_down.y <= region_RU.y - height(condition) + 1 && RU_yidx < PlacementRows.size()-1)
        RU_yidx++;
    while(PlacementRows[RU_yidx].left_down.y > region_RU.y - height(condition) + 1 && RU_yidx > 0)
    {
        RU_yidx--;
    } 


    //cout<<"LD_yidx "<<PlacementRows[LD_yidx].left_down.y<<endl;
    //cout<<"RU_yidx "<<PlacementRows[RU_yidx].left_down.y<<endl;
    int min_dist = __INT_MAX__;
    Point insert(Height,Width);

    for(int i = LD_yidx ; i <= RU_yidx ; i++)
    {
        int LD_xidx = (region_LD.x - PlacementRows[i].left_down.x)/PlacementRows[i].siteWidth;
        if((region_LD.x - PlacementRows[i].left_down.x)%PlacementRows[i].siteWidth!=0)
            LD_xidx++;
        if(LD_xidx < 0)
            LD_xidx = 0;
        if(LD_xidx >= PlacementRows[i].count)
            LD_xidx = PlacementRows[i].count -1;
        
        int RU_xidx = (region_RU.x - width(condition) + 1 - PlacementRows[i].left_down.x)/PlacementRows[i].siteWidth;
        if(RU_xidx < 0)
            RU_xidx = 0;
        if(RU_xidx >= PlacementRows[i].count)
            RU_xidx = PlacementRows[i].count -1;
        
        //cout<<"cur I "<<i<<endl;
        //cout<<"LD_xidx "<<PlacementRows[i].left_down.x + PlacementRows[i].siteWidth * LD_xidx<<endl;
        //cout<<"RU_xidx "<<PlacementRows[i].left_down.x + PlacementRows[i].siteWidth * RU_xidx<<endl;
        if(LD_xidx > RU_xidx)
            continue;
        //cout<<PlacementRows[i].left_down.x + PlacementRows[i].siteWidth * LD_xidx + width(condition) -1<<endl;
        //cout<<region_RU.x<<endl;
        if(PlacementRows[i].left_down.x + PlacementRows[i].siteWidth * LD_xidx + width(condition) -1 > region_RU.x)
            continue;
        Point vec = Point(PlacementRows[i].left_down.y,PlacementRows[i].left_down.x + PlacementRows[i].siteWidth *LD_xidx)  - LD(condition);
        int cur_dist = abs(vec.x) + abs(vec.y);
        if(cur_dist < min_dist)
        {
            min_dist = cur_dist;
            insert = Point(PlacementRows[i].left_down.y,PlacementRows[i].left_down.x + PlacementRows[i].siteWidth *LD_xidx);
        }

        vec = Point(PlacementRows[i].left_down.y,PlacementRows[i].left_down.x + PlacementRows[i].siteWidth *RU_xidx)  - LD(condition);
        cur_dist = abs(vec.x) + abs(vec.y);
        if(cur_dist < min_dist)
        {
            min_dist = cur_dist;
            insert = Point(PlacementRows[i].left_down.y,PlacementRows[i].left_down.x + PlacementRows[i].siteWidth *RU_xidx);
        }

    }

    return insert;
}


void Plane_E::loc_sequence_based_legalization()
{
    vector<Inst*> FFs_x;
    vector<Inst*> FFs_y;
    FFs_x = FF_list_bank;
    FFs_y = FF_list_bank;
    sort(FFs_x.begin(),FFs_x.end(),[](Inst* a, Inst* b){return a->LeftDown().x < b->LeftDown().x;});
    sort(FFs_x.begin(),FFs_x.end(),[](Inst* a, Inst* b){return a->LeftDown().y < b->LeftDown().y;});

    int min_allowed_x = Height;
    for(int i = 0 ; i < PlacementRows.size() ; i++)
    {
        if(min_allowed_x > PlacementRows[i].left_down.x)
            min_allowed_x = PlacementRows[i].left_down.x;
    }
    int min_allowed_y = PlacementRows[0].left_down.y;
    int max_allowed_x = -1;
    for(int i = 0 ; i < PlacementRows.size() ; i++)
    {
        if(max_allowed_x < PlacementRows[i].left_down.x + PlacementRows[i].siteWidth * (PlacementRows[i].count-1))
            max_allowed_x = PlacementRows[i].left_down.x + PlacementRows[i].siteWidth * (PlacementRows[i].count-1);
    }

    int max_allowed_y = PlacementRows[PlacementRows.size()-1].left_down.y;

    //obj: min overlap and min slack
    if(FFs_x.front()->LeftDown().x < min_allowed_x)
    {
        set_and_propagate(FFs_x.front(),Point(FFs_x.front()->LeftDown().y,min_allowed_x));
    }
    if(FFs_x.back()->LeftDown().x > max_allowed_x)
    {
        set_and_propagate(FFs_x.back(),Point(FFs_x.back()->LeftDown().y,max_allowed_x));
    }

    if(FFs_y.front()->LeftDown().y < min_allowed_y)
    {
        set_and_propagate(FFs_y.front(),Point(min_allowed_y,FFs_y.front()->LeftDown().x));
    }
    if(FFs_y.back()->LeftDown().y > max_allowed_y)
    {
        set_and_propagate(FFs_y.back(),Point(max_allowed_y,FFs_y.back()->LeftDown().x));
    }

    int min_total_slack = 100000000;
    int center_x;
    int center_y;
    for(int i = 0 ; i < FFs_x.size() ; i++)
    {
        int total_slack = 0;
        for(Pin* p:FFs_x[i]->INs)
        {
            total_slack += p->slack;
        }
        if(total_slack < min_total_slack)
        {
            min_total_slack = total_slack;
            center_x = i;
        }
    }
    min_total_slack = 100000000;
    for(int i = 0 ; i < FFs_y.size() ; i++)
    {
        int total_slack = 0;
        for(Pin* p:FFs_y[i]->INs)
        {
            total_slack += p->slack;
        }
        if(total_slack < min_total_slack)
        {
            min_total_slack = total_slack;
            center_y = i;
        }
    }

    for(int i = center_x ; i < FFs_x.size()-1 ; i++)
    {
        if(overlappingArea(FFs_x[i],FFs_x[i+1]) > 0)
        {
            int movement = FFs_x[i]->LeftDown().x + width(FFs_x[i]) - FFs_x[i+1]->LeftDown().x;
            if(i+1 == FFs_x.size()-1)
            {
                if(FFs_x[i+1]->LeftDown().x + movement > max_allowed_x)
                {
                    movement = max_allowed_x - FFs_x[i+1]->LeftDown().x;
                }
            }
            else if(FFs_x[i+1]->LeftDown().x + movement > FFs_x[i+2]->LeftDown().x)
            {
                movement = FFs_x[i+2]->LeftDown().x - FFs_x[i+1]->LeftDown().x;
            }
            set_and_propagate(FFs_x[i+1],Point(FFs_x[i+1]->LeftDown().y,FFs_x[i+1]->LeftDown().x + movement));
        }
    }

    for(int i = center_x ; i > 0 ; i--)
    {
        if(overlappingArea(FFs_x[i],FFs_x[i-1]) > 0)
        {
            int movement = FFs_x[i]->LeftDown().x - FFs_x[i-1]->LeftDown().x - width(FFs_x[i-1]);
            if(i-1 == 0)
            {
                if(FFs_x[i-1]->LeftDown().x + movement < min_allowed_x)
                {
                    movement = min_allowed_x - FFs_x[i-1]->LeftDown().x;
                }
            }
            else if(FFs_x[i-1]->LeftDown().x + movement < FFs_x[i-2]->LeftDown().x)
            {
                movement = FFs_x[i-2]->LeftDown().x - FFs_x[i-1]->LeftDown().x;
            }
            set_and_propagate(FFs_x[i-1],Point(FFs_x[i-1]->LeftDown().y,FFs_x[i+1]->LeftDown().x + movement));
        }
    }

    for(int i = center_y ; i < FFs_y.size()-1 ; i++)
    {
        if(overlappingArea(FFs_y[i],FFs_y[i+1]) > 0)
        {
            int movement = FFs_y[i]->LeftDown().y + height(FFs_y[i]) - FFs_y[i+1]->LeftDown().y;
            if(i+1 == FFs_y.size()-1)
            {
                if(FFs_y[i+1]->LeftDown().y + movement > max_allowed_y)
                {
                    movement = max_allowed_y - FFs_y[i+1]->LeftDown().y;
                }
            }
            else if(FFs_y[i+1]->LeftDown().y + movement > FFs_y[i+2]->LeftDown().y)
            {
                movement = FFs_y[i+2]->LeftDown().y - FFs_y[i+1]->LeftDown().y;
            }
            set_and_propagate(FFs_y[i+1],Point(FFs_y[i+1]->LeftDown().y + movement,FFs_y[i+1]->LeftDown().x));
        }
    }

    for(int i = center_y ; i > 0 ; i--)
    {
        if(overlappingArea(FFs_y[i],FFs_y[i-1]) > 0)
        {
            int movement = FFs_y[i]->LeftDown().y - FFs_y[i-1]->LeftDown().y - height(FFs_x[i-1]);
            if(i-1 == 0)
            {
                if(FFs_y[i-1]->LeftDown().y + movement < min_allowed_y)
                {
                    movement = min_allowed_y - FFs_y[i-1]->LeftDown().y;
                }
            }
            else if(FFs_y[i-1]->LeftDown().y + movement < FFs_y[i-2]->LeftDown().y)
            {
                movement = FFs_y[i-2]->LeftDown().y - FFs_y[i-1]->LeftDown().y;
            }
            set_and_propagate(FFs_x[i-1],Point(FFs_x[i-1]->LeftDown().y + movement,FFs_x[i+1]->LeftDown().x));
        }
    }

}