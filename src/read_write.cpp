#include "Plane_extension.h"
#include <fstream>
#include<iomanip>
#define u_map_itr(T) unordered_map<std::string,T>::const_iterator
using namespace std;

void Plane_E::read_output_format(string f)
{
    ifstream fin;
    fin.open(f);
    /*
    CellInst 2
    Inst reg5 SVT_FF_2 5952 3600 
    Inst reg6 SVT_FF_2 1278 3600 
    reg1/D map reg5/D0
    reg1/Q map reg5/Q0
    reg1/CLK map reg5/CLK
    reg2/D map reg5/D1
    reg2/Q map reg5/Q1
    reg2/CLK map reg5/CLK
    reg3/D map reg6/D0
    reg3/Q map reg6/Q0
    reg3/CLK map reg6/CLK
    reg4/D map reg6/D1
    reg4/Q map reg6/Q1
    reg4/CLK map reg6/CLK
    */

    string dummy;
    int new_FF_list_bank_size;
    fin>>dummy>>new_FF_list_bank_size;
    for(int i = 0 ; i < FF_list_bank.size() ; i++)
    {
        remove_Inst(FF_list_bank[i]);
        delete FF_list_bank[i]->get_root();
        delete FF_list_bank[i];
    }
    
    FF_list_bank.resize(new_FF_list_bank_size);
    unordered_map<string,Inst*> FF_list_bank_u;
    for(int i = 0 ; i < FF_list_bank.size() ; i++)
    {
        string name,Inst_type;
        Point LD;
        fin>>dummy>>name>>Inst_type>>LD.x>>LD.y;
        pair<int,int> p = find_lib_by_name(Inst_type);
        int idx = p.second;
        FF_list_bank[i] = new Inst(name,LD,&FF_lib[idx],p);
        FF_list_bank_u.insert(make_pair(FF_list_bank[i]->get_name(),FF_list_bank[i]));
    }

    for(int i = 0 ; i < FF_list_bank.size() ; i++)
    {
        Inst* cur = FF_list_bank[i];
        for(int j = 0 ; j < FF_list_bank[i]->Pins.size() ; j++)
        {
            Pin* cur_p = FF_list_bank[i]->Pins[j];
            if(toupper(cur_p->org_type[0]) == 'C')
            {
                FF_list_bank[i]->Pins[j] = FF_list_bank[i]->Pins.back();
                FF_list_bank[i]->Pins.pop_back();
                j--;
                delete cur_p;
            }
        }
    }

    while(!fin.eof())
    {

        string name_org,name_new;
        if(!(fin>>name_org))
            break;
        fin>>dummy>>name_new;
        pair<string,string> name_set = split_by_symbol(name_org);
        
        unordered_map<std::string,Inst*>::const_iterator got = FF_list_u.find(name_set.first);
        Inst* corrI = got->second;
        Pin* org_p = find_pin_by_name(name_set.second,corrI);
        
        name_set = split_by_symbol(name_new);
        got = FF_list_bank_u.find(name_set.first);
        Inst* newI = got->second;
        if(toupper(org_p->pin_type[0]) == 'C')
        {
            org_p->pin_type = name_set.second;
            org_p->belong_Inst = newI;
            newI->CLK.push_back(org_p);
            newI->Pins.push_back(org_p);
        }
        else
        {
            Pin* new_p = find_pin_by_name(name_set.second,newI);
            for(int loc = 0 ; loc < newI->Pins.size() ; loc++)
            {
                if(newI->Pins[loc] == new_p)
                    newI->Pins[loc] = binding(org_p,newI->Pins[loc]);
            }
        }
    }

    for(int i = 0 ; i < FF_list_bank.size() ; i++)
    {
        Inst* cur = FF_list_bank[i];
        for(int j = 0 ; j < FF_list_bank[i]->Pins.size() ; j++)
        {
            Pin* cur_p = FF_list_bank[i]->Pins[j];
            if(toupper(cur_p->org_type[0]) == 'D')
            {
                cur_p->type = 'I';
                cur->INs.push_back(cur_p);
            }
            else if(toupper(cur_p->org_type[0]) == 'Q')
            {
                cur_p->type = 'O';
                cur->OUTs.push_back(cur_p);
            }
            else
            {
                //cout<<cur_p->org_type<<endl;
            }
        }
    }

    cout<<"PASS READ FILE"<<endl;
    for(auto& in_d:In_list)
    {
        Inst* in = in_d.second;
        for(Pin* p : in->Pins)
        {
            p->slack = 0;
            p->arrival_time = -1;
        }
    }
    for(auto& in_d:Out_list)
    {
        Inst* in = in_d.second;
        for(Pin* p : in->Pins)
            p->arrival_time = -1;
    }
    for(Inst* in:FF_list)
    {
        for(Pin* p : in->Pins)
            p->arrival_time = -1;
    }
    for(Inst* in:G_list)
    {
        for(Pin* p : in->Pins)
            p->arrival_time = -1;
    }
    slack_propagation();
    HPWL = 0;
    for(int i = 0 ; i < FF_list_bank.size() ; i++)
    {
        
        for(Pin* p:FF_list_bank[i]->INs)
        {
            for(int k = 0 ; k < p->belong_nets.size() ; k++)
            {
                p->belong_nets[k]->set_center();
                Point HPWL_vec = p->belong_nets[k]->center_of_FROMs -  p->abs_loc();
                HPWL += abs(HPWL_vec.x) + abs(HPWL_vec.y);
            }
        }
        for(Pin* p:FF_list_bank[i]->OUTs)
        {
            for(int k = 0 ; k < p->belong_nets.size() ; k++)
            {
                p->belong_nets[k]->set_center();
                Point HPWL_vec = p->belong_nets[k]->center_of_TOs -  p->abs_loc();
                HPWL += abs(HPWL_vec.x) + abs(HPWL_vec.y);
            }
        }
    }

    cout<<"negative_slack   "<<negative_slack<<endl;
    cout<<"positive_slack   "<<positive_slack<<endl;
    cout<<"COST "<<cost()<<endl;
    cout<<"violated_bins_cnt    "<<violated_bins_cnt<<endl;
    cout<<"smoothen_bin_util    "<<smoothen_bin_util<<endl;
    cout<<endl;
}

void Plane_E::write_input_format(string f)
{
    /*
    Alpha 10
    Beta 10
    Gamma 0.0000002
    Lambda 10
    DieSize 0 0 23475 23280
    NumInput 2
    Input IN 8344 22840
    Input CLK 0 1970
    NumOutput 1
    Output out 23075 11410
    FlipFlop 1 SVT_FF_1 741 480 3
    Pin D 152 30 
    Pin CLK 494 30 
    Pin Q 38 270 
    FlipFlop 2 SVT_FF_2 798 1960 5
    Pin D0 494 350 
    Pin CLK 95 30 
    Pin Q0 665 30 
    Pin D1 494 510 
    Pin Q1 665 750 
    Gate G_1 460 460 2
    Pin OUT1 90 420
    Pin IN1 90 10
    NumInstances 5
    Inst reg1 SVT_FF_1 5952 3600 
    Inst reg2 SVT_FF_1 1278 3600 
    Inst reg3 SVT_FF_1 1278 6000 
    Inst reg4 SVT_FF_1 3615 3600 
    Inst G1 G_1 480 3600
    NumNets 6
    Net p0 2
    Pin reg1/Q
    Pin reg2/D
    Net p1 2
    Pin reg2/Q
    Pin reg3/D
    Net p2 2
    Pin reg3/Q
    Pin reg4/D
    Net out 2
    Pin reg4/Q
    Pin out
    Net in 2
    Pin IN
    Pin reg1/D
    Net clk 5
    Pin CLK
    Pin reg4/CLK
    Pin reg3/CLK
    Pin reg2/CLK
    Pin reg1/CLK
    BinWidth 1200
    BinHeight 1200
    BinMaxUtil 25
    PlacementRows 480 3600 57 240 395
    PlacementRows 480 6000 57 240 395
    DisplacementDelay 0.01
    QpinDelay SVT_FF_1 0.02
    QpinDelay SVT_FF_2 0.06
    TimingSlack reg1 D -0.183134
    TimingSlack reg2 D 0.149378
    TimingSlack reg3 D -0.152106
    TimingSlack reg4 D 0.150923
    GatePower SVT_FF_1 14.781
    GatePower SVT_FF_2 52.515
    */
    cout<<"START"<<endl;
    ofstream fout;
    fout.open(f);
    fout<< fixed  <<setprecision(10);
    fout<<"Alpha "<<alpha<<endl;
    fout<<"Beta "<<beta<<endl;
    fout<<"Gamma "<<gamma<<endl;
    fout<<"Lambda "<<lambda<<endl;
    fout.unsetf(std::ios_base::fixed);

    fout<<"DieSize 0 0 "<<Width<<" "<<Height<<endl;
    fout<<"NumInput "<<In_list.size()<<endl;
    for(auto& in_data : In_list)
    {
        Inst* in = in_data.second;
        fout<<"Input "<<in->get_name()<<" "<<in->LeftDown()<<endl;
    }

    fout<<"NumOutput "<<Out_list.size()<<endl;
    for(auto& in_data : Out_list)
    { 
        Inst* in = in_data.second;
        fout<<"Output "<<in->get_name()<<" "<<in->LeftDown()<<endl;
    }

    /*
    FlipFlop 1 SVT_FF_1 741 480 3
    Pin D 152 30 
    Pin CLK 494 30 
    Pin Q 38 270 
    */
   for(int i = 0 ; i < FF_lib.size() ; i++)
   {
        fout<<"FlipFlop "<<FF_lib[i].bits<<" "<<FF_lib[i].name<<" "<<FF_lib[i].width<<" "<<FF_lib[i].height<<" "<<FF_lib[i].pinCount<<endl;
        for(Pin p : FF_lib[i].Pin_set)
            fout<<"Pin "<<p.org_type<<" "<<p.org_relative_loc<<endl;
   }

    /*
    Gate G_1 460 460 2
    Pin OUT1 90 420
    Pin IN1 90 10
    */
    for(int i = 0 ; i < G_lib.size() ; i++)
    {
            fout<<"Gate "<<G_lib[i].name<<" "<<G_lib[i].width<<" "<<G_lib[i].height<<" "<<G_lib[i].pinCount<<endl;
            for(Pin p : G_lib[i].Pin_set)
                fout<<"Pin "<<p.org_type<<" "<<p.org_relative_loc<<endl;
    }

    /*
    NumInstances 5
    Inst reg1 SVT_FF_1 5952 3600 
    Inst reg2 SVT_FF_1 1278 3600 
    Inst reg3 SVT_FF_1 1278 6000 
    Inst reg4 SVT_FF_1 3615 3600 
    Inst G1 G_1 480 3600
    */

    fout<<"NumInstances "<<FF_list_bank.size()+G_list.size()<<endl;
    for(int i = 0 ; i < FF_list_bank.size() ; i++)
    {
        Point LD = FF_list_bank[i]->LeftDown();
        LD.toInt();
        fout<<"Inst "<<FF_list_bank[i]->getName()<<" "<<FF_list_bank[i]->corr_data->name<<" "<<LD<<endl;
    }
    for(int i = 0 ; i < G_list.size() ; i++)
    {
        Point LD = G_list[i]->LeftDown();
        LD.toInt();
        fout<<"Inst "<<G_list[i]->getName()<<" "<<G_list[i]->corr_data->name<<" "<<LD<<endl;
    }

    /*
    NumNets 6
    Net p0 2
    Pin reg1/Q
    Pin reg2/D
    */
   fout<<"NumNets "<<net_list.size()<<endl;
   for(int i = 0 ; i < net_list.size() ; i++)
   {
        fout<<"Net "<<net_list[i]->name<<" "<<net_list[i]->relative.size()<<endl;
        for(int j = 0 ; j < net_list[i]->relative.size() ; j++)
        {
            fout<<"Pin "<< net_list[i]->relative[j]->get_name()<<endl;
        }
   }

    /*
    BinWidth 1200
    BinHeight 1200
    BinMaxUtil 25
    PlacementRows 480 3600 57 240 395
    PlacementRows 480 6000 57 240 395
    DisplacementDelay 0.01
    QpinDelay SVT_FF_1 0.02
    QpinDelay SVT_FF_2 0.06
    TimingSlack reg1 D -0.183134
    TimingSlack reg2 D 0.149378
    TimingSlack reg3 D -0.152106
    TimingSlack reg4 D 0.150923
    GatePower SVT_FF_1 14.781
    GatePower SVT_FF_2 52.515
    */

    fout<<"BinWidth "<<BinWidth<<endl;
    fout<<"BinHeight "<<BinHeight<<endl;
    fout<<"BinMaxUtil "<<BinMaxUtil<<endl;
    for(int i = 0 ; i < PlacementRows.size() ; i++)
    {
        fout<<"PlacementRows "<<PlacementRows[i].left_down<<" "<<PlacementRows[i].siteWidth<<" "<<PlacementRows[i].siteHeight<<" "<<PlacementRows[i].count<<endl;
    }
    fout<< fixed  <<setprecision(10);
    fout<<"DisplacementDelay "<<displacement_delay<<endl;
    for(int i = 0 ; i < FF_lib.size() ; i++)
    {
        fout<<"QpinDelay "<<FF_lib[i].name<<" "<< FF_lib[i].QpinDelay<<endl;
    }
    for(int i = 0 ; i < evaluation_Pins.size() ; i++)
    {
        fout<<"TimingSlack "<<evaluation_Pins[i]->belong_Inst->get_name()<<" "<<evaluation_Pins[i]->pin_type<<" "<<evaluation_Pins[i]->slack<<endl;
    }
    for(int i = 0 ; i < FF_lib.size() ; i++)
    {
        fout<<"GatePower "<<FF_lib[i].name<<" "<<FF_lib[i].power<<endl;
    }
    cout<<"PASS"<<endl;
    fout.unsetf(std::ios_base::fixed);
}