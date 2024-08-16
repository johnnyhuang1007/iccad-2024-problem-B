#include "Plane_extension.h"
#include <fstream>
#include<iomanip>
#define u_map_itr(T) unordered_map<std::string,T>::const_iterator
using namespace std;

string toupper(string name)
{
    for(int i = 0 ; i < name.size() ; i++)
    {
        if(isalpha(name[i]))
            name[i] = toupper(name[i]);
    }
    return name;
}

string tolower(string name)
{
    for(int i = 0 ; i < name.size() ; i++)
    {
        if(isalpha(name[i]))
            name[i] = tolower(name[i]);
    }
    return name;
}

Plane_E::Plane_E(string input)
{
    ifstream fin;
    fin.open(input);
    string dummy;
    fin>>dummy>>this->alpha;
    fin>>dummy>>this->beta;
    fin>>dummy>>this->gamma;
    fin>>dummy>>this->lambda;

    fin >>dummy>>dummy>>dummy>>Width>>Height;  //DieSize 0.0 0.0 50.0 30.0
    
    int size;
    /*
    NumInput 2
    Input in 8344 22840
    Input clk 0 1970
    */
    //corner stitching init
    //start
    Module* dummyModule = new Module("boundary", 0);
	boundary[0] = new Tile(Point(0,-1 -1000000000), Point(Height +1000000000, -1), dummyModule);
	boundary[1] = new Tile(Point(Height, 0), Point(Height +1000000000, Width +1000000000), dummyModule);
	boundary[2] = new Tile(Point(-1 -1000000000, Width), Point(Height-1, Width +1000000000), dummyModule);
	boundary[3] = new Tile(Point(-1 -1000000000, -1 -1000000000), Point(-1, Width-1), dummyModule);

	Tile* Space = new Tile(Point(0, 0), Height, Width);
	boundary[0]->stitch[0] = boundary[1];
	boundary[0]->stitch[1] = boundary[3];
	boundary[1]->stitch[1] = Space;
	boundary[1]->stitch[2] = boundary[0];
	boundary[2]->stitch[2] = boundary[3];
	boundary[2]->stitch[3] = boundary[1];
	boundary[3]->stitch[0] = boundary[2];
	boundary[3]->stitch[3] = Space;

    for (size_t i = 0; i < 4; i++)
		Space->stitch[i] = boundary[(i + 2) % 4];
    //end


    cout<<"READ INPUT NODE"<<endl;
    fin>>dummy>>size;
    for(int i = 0 ; i < size ; i++) //to do
    {
        
        string name;//inst name is also its pin_type
        Point loc;
        fin>>dummy>>name>>loc.x>>loc.y;
        name = name;
        Inst* N = new Inst(name,loc,1,1);
        In_list.insert(make_pair(N->get_name(),N));
        insert_inst(N);
    }

    cout<<"READ OUTPUT NODE"<<endl;
    fin>>dummy>>size;//NumOutput 2
    for(int i = 0 ; i < size ; i++) //to do
    {
        
        string name;//inst name is also its pin_type
        Point loc;
        fin>>dummy>>name>>loc.x>>loc.y;
        name = name;
        Inst* N = new Inst(name,loc,1,1);
        Out_list.insert(make_pair(N->get_name(),N));
        insert_inst(N);
    }
    //
    SET_FF_GATE_LIB(fin);
    //it also setups every pin data
    SET_FF_GATE_INFO(fin);

    SET_NET(fin);
    IOC_distinguish();
 
    fin>>dummy>>BinWidth;
    fin>>dummy>>BinHeight;
    fin>>dummy>>BinMaxUtil;
    cout<<dummy<<"  "<<BinMaxUtil<<endl;
    int BinX,BinY;
    BinY = Height/BinHeight + 1;
    BinX = Width/BinWidth + 1;
    Bins.resize(BinY);
    for(int i = 0 ; i < BinY ; i++)
    {
        Bins[i].resize(BinX);
        for(int j = 0 ; j < BinX ; j++)
        {
            Bins[i][j].left_down.y = i*BinHeight;
            Bins[i][j].left_down.x = j*BinWidth;
            Bins[i][j].height = BinHeight;
            Bins[i][j].width = BinWidth;
            Bins[i][j].used_area = 0;
        }
    }
    //PlacementRows <startX> <startY> <siteWidth> <siteHeight> <totalNumOfSites>
    list<PlacementRow> tmp_row;
    fin>>dummy;
    while(dummy=="PlacementRows")
    {
        PlacementRow t;
        fin>>t.left_down.x >>t.left_down.y>>t.siteWidth>>t.siteHeight>>t.count;
        tmp_row.push_back(t);
        fin>>dummy;
    }

    PlacementRows.reserve(tmp_row.size());
    for(auto& t:tmp_row)
    {
        PlacementRows.push_back(t);
    }
    for(int i = 0 ; i < PlacementRows.size()-1 ; i++)
    {
        for(int j = i+1 ; j < PlacementRows.size() ; j++)
        {
            if(PlacementRows[j].left_down.y < PlacementRows[i].left_down.y)
            {
                PlacementRow t = PlacementRows[j];
                PlacementRows[j] = PlacementRows[i];
                PlacementRows[i] = t;
            }
        }
    }


    cout<<"READ DELAY INFO"<<endl;
    SET_COST_INFO(fin);
    cout<<"SLACK CALCULATION"<<endl;
    required_time_init();
    cout<<"GATE INSTANCE INSERTION"<<endl;
    for (int i = 0; i < G_list.size(); i++)
    {
        if(checkAllSpace(G_list[i]->get_root()))
        {
		    insert(G_list[i]->get_root());
            G_list[i]->inserted = 1;
        }
    }

    for (auto& IN:In_list)
    {
        if(checkAllSpace(IN.second->get_root()))
        {
		    insert(IN.second->get_root());
            IN.second->inserted = 1;
        }
    }

    for (auto& IN:Out_list)
    {
        if(checkAllSpace(IN.second->get_root()))
        {
		    insert(IN.second->get_root());
            IN.second->inserted = 1;
        }
    }


    violated_bins_cnt = 0;
    for(int idx = 0 ; idx < G_list.size() ; idx++)
    {
        add_util(G_list[idx]);
        Point LEFTDOWN = G_list[idx]->LeftDown();
        Point RIGHTUP = G_list[idx]->RightUp();
        int idxL = LEFTDOWN.x/BinWidth;
        int idxR = RIGHTUP.x/BinWidth;
        int idxU = RIGHTUP.y/BinHeight;
        int idxD = LEFTDOWN.y/BinHeight;
        for(int i = idxD ; (i <= idxU) && (i <= (Bins.size()-1)) ; i++)
        {
            for(int j = idxL ; (j <= idxR) && (j <= (Bins[i].size()-1)) ; j++ )
            {
                Bins[i][j].smoothen_area += overlappingArea(Bins[i][j],G_list[idx]);
            }
        }
    }
    smoothen_bin_util = 0;
    FF_total_area = 0;
    FF_total_power = 0;
    for(int idx = 0 ; idx < FF_list_bank.size() ; idx++)
    {
        FF_list_bank[idx]->idx = idx;
        add_util(FF_list_bank[idx]);
        add_smooth_util(FF_list_bank[idx]);
        FF_total_area+=FF_list_bank[idx]->area();
        FF_total_power+=FF_list_bank[idx]->corr_data->power;
    }

    cout<<violated_bins_cnt<<endl;
    cout<<cost()<<endl;


    //obtain clk_domain_list for banking usage
    /*
    for(net* cur_net: net_list)
    {
        bool is_clk = 0;
        for(int i = 0 ; i < cur_net->TOs.size() ; i++)
        {
            if(cur_net->TOs[i]->type == 'C')
            {
                is_clk = 1;
                break;
            }
        }
        if(!is_clk)
            continue;
        clk_domain_Insts.insert(make_pair(cur_net,vector<Inst*>{}));
    } 
    for(int i = 0 ; i < )
    */
} 

double Plane_E::cost()
{
    return -alpha * negative_slack + beta * FF_total_power + gamma * double(FF_total_area) + lambda * double(violated_bins_cnt);
}

void Plane_E::SET_COST_INFO(ifstream& fin)
{
    string dummy;
    fin>>this->displacement_delay;

    for(int i = 0 ; i < net_list.size() ; i++)
        net_list[i]->displacement_delay = displacement_delay;
    fin>>dummy;
    while(dummy == "QpinDelay")
    {
        
        string name;
        double delay;
        fin>>name>>delay;
        name = name;
        Inst_data* data = find_data_by_name(name);
        data->QpinDelay = delay;
        fin>>dummy;
    }
    list<Pin*> eval_Pins_list;
    while(dummy == "TimingSlack")
    {
        string ffname,pname;
        fin>>ffname>>pname;
        ffname = ffname;
        pname = pname;
        pname = pname;
        pname = ffname + '/' + pname;
        Pin* corr_pin = find_pin_by_FFsPinName(pname);
        eval_Pins_list.push_back(corr_pin);
        fin>>corr_pin->slack;
        corr_pin->slack_determined = 1;
        fin>>dummy;
    }
    evaluation_Pins = vector<Pin*>(eval_Pins_list.begin(),eval_Pins_list.end());
    while(dummy == "GatePower")
    {
        string ffname;
        fin>>ffname;
        Inst_data* corrData = find_lib_by_name_ptr(ffname);
        fin>>corrData->power;
        if(!(fin>>dummy))
            break;
    }
}

Inst_data* Plane_E::find_data_by_name(string name)
{
    for(int i = 0 ; i < FF_lib.size() ; i++)
    {
        if(FF_lib[i].name == name)
            return &FF_lib[i];
    }
    for(int i = 0 ; i < G_lib.size() ; i++)
    {
        if(G_lib[i].name == name)
            return &G_lib[i];
    }
}

void Plane_E::SET_FF_GATE_LIB(ifstream& fin)
{
    cout<<"SETUP INSTANCE LIBARAY"<<endl;
    list<Inst_data> FF_tmps;
    list<Inst_data> G_tmps;
    string Word;
    fin>>Word;
    while(Word!="NumInstances") //FlipFlop 1 FF1 5.0 10.0 3         Gate G1 5.0 10.0 2
    {
        string dummy;
        Inst_data tmpF;
        Inst_data tmpG;
        if(Word=="FlipFlop")
        {
            fin>>tmpF.bits>>tmpF.name>>tmpF.width>>tmpF.height>>tmpF.pinCount;
            for(int i = 0 ; i < tmpF.pinCount ; i++)
            {
                Pin tmp;
                fin>>dummy>>tmp.pin_type>>tmp.relative_loc.x>>tmp.relative_loc.y;//Pin D0 0.0 9.0
                tmp.pin_type = tmp.pin_type;
                tmp.org_relative_loc = tmp.relative_loc;
                tmp.org_type = tmp.pin_type;
                tmpF.Pin_set.push_back(tmp);
            }
            FF_tmps.push_back(tmpF);
        }
        else
        {
            fin>>tmpG.name>>tmpG.width>>tmpG.height>>tmpG.pinCount;
            for(int i = 0 ; i < tmpG.pinCount ; i++)
            {   
                Pin tmp;
                fin>>dummy>>tmp.pin_type>>tmp.relative_loc.x>>tmp.relative_loc.y;//Pin D0 0.0 9.0
                tmp.pin_type = tmp.pin_type;
                tmp.org_relative_loc = tmp.relative_loc;
                tmp.org_type = tmp.pin_type;
                tmpG.Pin_set.push_back(tmp);
            }
            G_tmps.push_back(tmpG);
        }
        fin>>Word;
    }
    FF_lib.reserve(FF_tmps.size());
    for(auto& FF:FF_tmps)
        FF_lib.push_back(FF);
    
    G_lib.reserve(G_tmps.size());
    for(auto& G:G_tmps)
        G_lib.push_back(G);
    
    
    vector<int> bit_ff_cnts;
    int max_cnt = -1;
    for(int i = 0 ; i < FF_lib.size() ; i++)
    {
        if((FF_lib[i].bits-1)/2 > max_cnt)
            max_cnt = (FF_lib[i].bits-1)/2;
    }
    bit_ff_cnts.resize(max_cnt+1,0);
    for(int i = 0 ; i < FF_lib.size() ; i++)
    {
        bit_ff_cnts[(FF_lib[i].bits-1)/2]++;
    }
    FF_lib_bits.resize(max_cnt+1);
    for(int i = 0 ; i < max_cnt ; i++)
    {
        FF_lib_bits[i].reserve(bit_ff_cnts[i]);
    }
    for(int i = 0 ; i < FF_lib.size() ; i++)
    {
        FF_lib_bits[(FF_lib[i].bits-1)/2].push_back(&FF_lib[i]);
    }


    for(int i = Word.size()-1 ; i >= 0 ; i--)
    {
        fin.putback(Word[i]);
    }
    
}


pair<int,int> Plane_E::find_lib_by_name(string name)
{
    for(int i = 0 ; i < FF_lib.size() ; i++)
    {
        if(name == FF_lib[i].name)
            return T_Idx(0,i);
    }
    for(int i = 0 ; i < G_lib.size() ; i++)
    {
        if(name == G_lib[i].name)
            return T_Idx(1,i);
    }
}

Inst_data* Plane_E::find_lib_by_name_ptr(string name)
{
    for(int i = 0 ; i < FF_lib.size() ; i++)
    {
        if(name == FF_lib[i].name)
            return &FF_lib[i];
    }
    for(int i = 0 ; i < G_lib.size() ; i++)
    {
        if(name == G_lib[i].name)
            return &G_lib[i];
    }
}

void Plane_E::SET_FF_GATE_INFO(ifstream& fin)
{
    cout<<"SETUP INITIAL INSTANCE INFO"<<endl;
    string Word;
    fin>>Word;
    int size;
    fin>>size;

    
    G_list.reserve(size);
    FF_list.reserve(size);

    for(int i = 0 ; i < size ; i++)
    {
        string name, lib_name;
        Point ld;
        fin>>name>>name>>lib_name>>ld.x>>ld.y;
        T_Idx p = find_lib_by_name(lib_name);
        int idx = p.second;
        if(p.first == 0)
        {
            //Fixed_Module(std::string, Point, int height, int width)
            //Inst* N = new Inst(name,ld,FF_lib[idx].height,FF_lib[idx].width);
            Inst* N = new Inst(name,ld,&FF_lib[idx],p);
            FF_list.push_back(N);
            FF_list_u.insert(make_pair(N->get_name(),N));
        }
        else
        {
            //Fixed_Module(std::string, Point, int height, int width)
            //Inst* N = new Inst(name,ld,G_lib[idx].height,G_lib[idx].width);
            Inst* N = new Inst(name,ld,&G_lib[idx],p);
            G_list.push_back(N);
            G_list_u.insert(make_pair(N->get_name(),N));
        }
    }

    FF_list_bank.resize(FF_list.size());
    for(int i = 0 ; i < FF_list.size() ; i++)
    {
        FF_list_bank[i] = new Inst(FF_list[i]); //it copy the original ff info, but it do not copy pin (shouldn't though)
        /*
        for(int j = 0 ; j < FF_list_bank[i]->Pins.size() ; j++)
        {
            cout<<FF_list_bank[i]->Pins[j]<<endl;
            cout<<FF_list[i]->Pins[j]<<endl;
            cout<<endl;
        }
        */
    }
}

void Plane_E::SET_NET(ifstream& fin)
{
    //Inst_Pins
    cout<<"SETUP NODE CONNECTION"<<endl;
    string Word;
    fin>>Word;
    int size;
    fin>>size;
    net_list.reserve(size);
    for(int i = 0 ; i < size ; i++)
    {

        /*
        Net p0 2
        Pin reg1/Q
        Pin reg2/D
        */
        net* new_net = new net;
        string name;
        int pcount;
        fin>>Word>>new_net->name>>pcount;
        new_net->relative.reserve(pcount);
        for(int j = 0 ; j < pcount ; j++)
        {
            fin>>Word>>Word;
            Pin* cur_pin = find_pin_by_FFsPinName(Word);
            cur_pin->belong_net = new_net;
            new_net->relative.push_back(cur_pin);
        }
        net_list.push_back(new_net);
    }

}

pair<string,string> split_by_symbol(string str)
{
    std::string delimiter = "/";
    size_t pos = 0;
    std::string token;

    
    pos = str.find(delimiter);
    if (pos == string::npos)
    {
        return pair<string,string>(str,str);
    }
    token = str.substr(0, pos); 

    
    str.erase(0, pos + delimiter.length());

    return pair<string,string>(token,str);
}

Pin* Plane_E::find_pin_by_FFsPinName(string input)
{
    pair<string,string> name_set = split_by_symbol(input);
    Inst* corrI = find_inst_by_name(name_set.first);
    Pin* corrP = find_pin_by_name(name_set.second,corrI);
    return corrP;
}

Inst* Plane_E::find_inst_by_name(std::string name)
{
    if(name == "CLK" && FF_list_bank.size() == 4)
        name = "clk";
    unordered_map<std::string,Inst*>::const_iterator got = FF_list_u.find(name);
    if(got!=FF_list_u.end())
        return got->second;

    got = In_list.find(name);
    if(got!=In_list.end())
        return got->second;

    got = Out_list.find(name);
    if(got!=Out_list.end())
        return got->second;

    got = G_list_u.find(name);
    if(got!=G_list_u.end())
        return got->second;
    
    cout<<"NOT FOUND!?"<<name<<endl;
    exit(0);
}

Pin* Plane_E::find_pin_by_name(std::string name,Inst* corr)
{
    for(int i = 0 ; i < corr->Pins.size() ; i++)
    {
        if (corr->Pins[i]->pin_type == name)
            return corr->Pins[i];
        if (FF_list_bank.size()==4 && toupper(corr->Pins[i]->pin_type) == toupper(name))
            return corr->Pins[i];
    }
}

void Plane_E::outimg()
{
    ofstream fout;
	fout.open("./IMG/img.txt");
	fout <<fixed<<setprecision(0)<< Width << " " << Height << endl;
	fout << "GATE " << G_list.size()+In_list.size()+Out_list.size() << endl;
	for (int i = 0; i < G_list.size(); i++)
	{
		Tile* tmp = G_list[i]->get_root();
		fout << LD(tmp) << " " << width(tmp) << " " << height(tmp) << endl;
	}
    for (auto& T:In_list)
	{
		Tile* tmp = T.second->get_root();
		fout << LD(tmp) << " " << width(tmp) << " " << height(tmp) << endl;
	}
    for (auto& T:Out_list)
	{
		Tile* tmp = T.second->get_root();
		fout << LD(tmp) << " " << width(tmp) << " " << height(tmp) << endl;
	}
	fout << "FF " <<fixed<< FF_list_bank.size() << endl;
	for (int i = 0; i < FF_list_bank.size(); i++)
	{
		Tile* tmp = FF_list_bank[i]->get_root();
		fout << LD(tmp) << " " << width(tmp) << " " << height(tmp) << endl;
	}

	fout << "Wire " <<fixed<<net_list.size()<<endl;//Wire_set.size() << endl;
    
	for (int i = 0; i < net_list.size(); i++)
	{
        fout<<net_list[i]->relative.size()<<endl;
        Point Center(0,0);
        for(int j = 0 ; j < net_list[i]->relative.size() ; j++)
        {
            Center = Center + net_list[i]->relative[j]->abs_loc();
        }
        Center.x /= net_list[i]->relative.size();
        Center.y /= net_list[i]->relative.size();

        for(int j = 0 ; j < net_list[i]->relative.size() ; j++)
        {
            fout <<fixed<< net_list[i]->relative[j]->abs_loc() << "	" << Center  << endl;
        }
	}
    
	fout.close();
}

bool on_P_Row(Inst* cur,vector<PlacementRow> P_rows)
{
    for(int i = 0 ; i < P_rows.size() ; i++)
    {
        if (LD(cur).y != P_rows[i].left_down.y)
        {
            continue;
        }
        if(int(LD(cur).x - P_rows[i].left_down.x)%P_rows[i].siteWidth!=0)
        {
            continue;
        }
        
        int cnt = int(LD(cur).x - P_rows[i].left_down.x)/P_rows[i].siteWidth;
        if(cnt<0 || cnt > P_rows[i].count)
        {
            continue;
        }
        return 1;
    }
    return 0;
}

vector<Inst*> Plane_E::get_not_legal_location()
{
    list<Inst*> tmps;
    for(int i = 0 ; i < FF_list_bank.size() ; i++)
    {
        if(!on_P_Row(FF_list_bank[i],PlacementRows))
            tmps.push_back(FF_list_bank[i]);
    }
    vector<Inst*> ERR{ begin(tmps), end(tmps) };
    return ERR;
}

bool Plane_E::is_legal()
{
    vector<Inst*> ERR;
    ERR = get_not_legal_location();
    return ERR.size()==0;
}

Point Plane_E::random()
{
    Point loc;
    int ROW = rand()%PlacementRows.size();
    loc.y = PlacementRows[ROW].left_down.y;
    loc.x = PlacementRows[ROW].left_down.x + rand()%PlacementRows[ROW].count*PlacementRows[ROW].siteWidth;
    return loc;
}

Point Plane_E::closest_Legal_locs(Point cur)    //to Improve
{
    int y_idx  = 0;
    if(cur.y >= PlacementRows.back().left_down.y)
        y_idx = PlacementRows.size()-2;
    else if(cur.y <= PlacementRows[0].left_down.y)
        y_idx = 0;
    else
    {
        int move = (PlacementRows.size()+1)/2;

        while(!(PlacementRows[y_idx].left_down.y <= cur.y && PlacementRows[y_idx+1].left_down.y > cur.y))
        {
            if(PlacementRows[y_idx].left_down.y > cur.y)
                y_idx -= move;
            else
                y_idx += move;
            move = (move+1)/2;
        }
    }
    if(y_idx >= PlacementRows.size()-1)
        y_idx = PlacementRows.size()-2;
    
    int x_idx1  = (cur.x - PlacementRows[y_idx].left_down.x)/PlacementRows[y_idx].siteWidth;
    if(x_idx1 >= PlacementRows[y_idx].count)
        x_idx1 = PlacementRows[y_idx].count-1;
    if(cur.x - PlacementRows[y_idx].left_down.x - x_idx1*PlacementRows[y_idx].siteWidth >= PlacementRows[y_idx].siteWidth/2)
        x_idx1++;

    int x_idx2  = (cur.x - PlacementRows[y_idx+1].left_down.x)/PlacementRows[y_idx+1].siteWidth;
    if(x_idx2 >= PlacementRows[y_idx+1].count)
        x_idx2 = PlacementRows[y_idx+1].count-1;
    if(cur.x - PlacementRows[y_idx+1].left_down.x - x_idx2*PlacementRows[y_idx+1].siteWidth >= PlacementRows[y_idx+1].siteWidth/2)
        x_idx2++;


    Point newP1;
    newP1.y = PlacementRows[y_idx].left_down.y;
    newP1.x = PlacementRows[y_idx].left_down.x + x_idx1*PlacementRows[y_idx].siteWidth;

    Point newP2;
    newP2.y = PlacementRows[y_idx+1].left_down.y;
    newP2.x = PlacementRows[y_idx+1].left_down.x + x_idx2*PlacementRows[y_idx+1].siteWidth;

    int v1 = abs(newP1.x - cur.x) + abs(newP1.y - cur.y);
    int v2 = abs(newP2.x - cur.x) + abs(newP2.y - cur.y);

    if(v1 < v2)
        return newP1;
    return newP2;
};

void Plane_E::location_legalization(vector<Inst*> to_fix)
{

}

void Plane_E::set_km_result(string input,string output)
{
    for(int i = 0 ; i < FF_list_bank.size() ; i++)
    {
        if(FF_list_bank[i]->inserted)
        {
            remove(FF_list_bank[i]->get_root());
            FF_list_bank[i]->set_root(NULL);
        }
        else if(FF_list_bank[i]->get_root()!=NULL)
        {
            delete FF_list_bank[i]->get_root();
            FF_list_bank[i]->set_root(NULL);
        }
        delete FF_list_bank[i];
    }
    FF_list_bank.clear();

    ifstream fin;
    fin.open(input);
    string dummy;
    int size;
    fin>>dummy;
    fin>>size;
    FF_list_bank.reserve(size);

    //Inst(std::string name, Point LD, FF_data data,std::pair<int,int> T_Idx );
    //std::pair<int,int> find_lib_by_name(std::string name);
    for(int i = 0 ; i < size ; i++)
    {
        string name, type;
        Point LD;
        fin>>name>> name>>type;
        fin>>LD.x>>LD.y;
        T_Idx p = find_lib_by_name(type);
        Inst* newI = new Inst(name,LD,&FF_lib[p.second],p);  //to fix mapping problem
        Point newP = closest_Legal_locs(LD);
        newI->set_new_loc(newP);
        FF_list_bank.push_back(newI);
        while(!checkAllSpace(newI->get_root()))
        {
            newI->set_new_loc(random());
        }
        insertFix(newI);
        newI->inserted = 1;
    }

    

    ofstream fout;
    fout.open(output);
    fout<<dummy<<" "<<size<<endl;
    for(int i = 0 ; i < size ; i++)
    {
        fout<<"Inst "<<FF_list_bank[i]->getName()<<" ";
        T_Idx p = FF_list_bank[i]->corr_Lib;
        fout<<FF_lib[p.second].name<<" "<<fixed<<setprecision(0)<<LD(FF_list_bank[i])<<endl;
    }
    while(!fin.eof())
    {
        getline(fin,dummy);
        if(dummy.size()>2)
            fout<<dummy<<endl;
    }
    
}


void Plane_E::output(char* file)
{
    ofstream fout;
    fout.open(file);
    fout<<"CellInst "<<FF_list.size()<<endl;
    fout<<fixed;
    for(int i = 0 ; i < FF_list_bank.size() ; i++)
    {
        FF_list_bank[i]->set_name("reg" + to_string(FF_list_bank.size()+i));
    }
    for(int i = 0 ; i < FF_list_bank.size() ; i++)
    {
        fout<<"Inst "<<FF_list_bank[i]->getName()<<" ";
        T_Idx p = FF_list_bank[i]->corr_Lib;
        fout<<FF_lib[p.second].name<<" "<<setprecision(0)<<LD(FF_list_bank[i])<<endl;

    }

    for(int i = 0 ; i < FF_list_bank.size() ; i++)
    {
        T_Idx p = FF_list_bank[i]->corr_Lib;
        for(int j = 0 ; j < FF_list_bank[i]->Pins.size() ; j++)
        {
            fout<< FF_list_bank[i]->Pins[j]->orginal_Inst->get_name() <<"/"<<FF_list_bank[i]->Pins[j]->org_type<<" map "<<FF_list_bank[i]->Pins[j]->get_name() <<endl;
        }
    }
}

bool Plane_E::insert_inst(Inst* cur)    
{   
    if(cur->inserted == 1)  //you can only modify inst data if not inserted
        return 1;
    if(!checkAllSpace(cur->get_root()))
    {
        return 0;
    }
    insert(cur->get_root());
    cur->inserted = 1;
    return 1;
}

bool Plane_E::insert_inst(Inst* cur,Tile* start)    
{   
    if(cur->inserted == 1)  //you can only modify inst data if not inserted
        return 1;
    if(!checkAllSpace(cur->get_root(),start))
    {
        return 0;
    }
    insert(cur->get_root());
    cur->inserted = 1;
    return 1;
}

bool Plane_E::remove_Inst(Inst* cur)    //return 1 if removed successful
{
    if(!cur->inserted)  //you can only modify inst data if not inserted
    {
        return 1;
    }
    Tile* newT = new Tile(cur->get_root(),0); //don't copy stitches
    remove(cur->get_root());
    cur->set_root_to_NULL();
    cur->set_root(newT);
    cur->inserted = 0;
    return 1;
}

void Plane_E::FFs_insertion()
{
    for(Inst* I: FF_list_bank)
    {
        insert_inst(I);
    }
}

Plane_E::~Plane_E()
{
    
    for(int i = 0 ; i < FF_list.size() ; i++)
    {
        for(int j = 0 ; j < FF_list[i]->Pins.size() ; j++)
            delete FF_list[i]->Pins[j];
        delete FF_list[i]->get_root();
        delete FF_list[i];
    }
	for (int i = 0; i < FF_list_bank.size(); i++)
	{
		remove_Inst(FF_list_bank[i]);
        
        delete FF_list_bank[i]->get_root();
        delete FF_list_bank[i];
	}
	for (int i = 0; i < G_list.size(); i++)
	{
		remove(G_list[i]->get_root());
        for(int j = 0 ; j < G_list[i]->Pins.size() ; j++)
            delete G_list[i]->Pins[j];
		delete G_list[i];
	}
    vector<Inst*> in_copys;
    for (auto& in_d : In_list)
        in_copys.push_back(in_d.second);
    for(int i = 0 ; i < in_copys.size() ; i++)
	{
        Inst* in = in_copys[i];
		remove_Inst(in);
        delete in->get_root();
        delete in->Pins[0];
		delete in;
	}
    for (auto& in_d : Out_list)
	{
        Inst* in = in_d.second;
		remove_Inst(in);
        delete in->get_root();
        delete in->Pins[0];
		delete in;
	}
    for(int i = 0 ; i < net_list.size() ; i++)
    {
        delete net_list[i];
    }
}

void Plane_E::IOC_distinguish()
{
    //The Input of the chip is the output for the inner gates;
    for(auto& data:In_list)
    {
        Inst* cur = data.second;
        cur->Pins[0]->type = 'O';
        cur->OUTs.push_back(cur->Pins[0]);
    }
    for(auto& data:Out_list)
    {
        Inst* cur = data.second;
        cur->Pins[0]->type = 'I';
        cur->INs.push_back(cur->Pins[0]);
    }
    for(auto& cur:FF_list_bank)
    {
        for(Pin* cur_p:cur->Pins)
        {
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
            else if(toupper(cur_p->org_type[0]) == 'C')
            {
                cur_p->type = 'C';
                cur->CLK = cur_p;
            }
            else
            {
                //cout<<cur_p->org_type<<endl;
            }
        }
    }
    for(auto& cur:G_list)
    {
        for(Pin* cur_p:cur->Pins)
        {
            if(cur_p->org_type[0] == 'I')
            {
                cur_p->type = 'I';
                cur->INs.push_back(cur_p);
            }
            else if(cur_p->org_type[0] == 'O')
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

    /*
          ____ from               to  ____
        -|D  Q|----------------------|I  O|
        -|C   |                      |    |
         |____|                      |____|

    */
    for(net* cur:net_list)
    {
        for(Pin* p : cur->relative)
        {
            if(p->type == 'O')
            {
                cur->FROMs.push_back(p);
            }
            else if(p->type == 'I')
                cur->TOs.push_back(p);
            else if(p->type == 'C')
                cur->CLK = p;
            else
                cout<<"UNKNOWN TYPE"<<endl;
        }
    }
}


void Plane_E::insert_FFs()
{
    for(int i = 0 ; i < FF_list_bank.size() ; i++)
    {
        cout<<i<<endl;
        Point NewP = closest_Legal_locs(FF_list_bank[i]->LeftDown());
        cout<<NewP<<endl;
        FF_list_bank[i]->set_new_loc(NewP);
        int j = 2 ;
        int count = 0;
        Tile* start = point_finding(FF_list_bank[i]->LeftDown());
        while(!insert_inst(FF_list_bank[i],start))
        {
            Point New1(0,0);
            New1 = random();
            double x = double(NewP.x) + double(rand()%(2*j+1) - j) * PlacementRows[0].siteWidth;
            double y = double(NewP.y) + double(rand()%(2*j+1) - j) * PlacementRows[0].siteHeight;
            New1.x = x;
            New1.y = y;
            New1 = closest_Legal_locs(New1);
            FF_list_bank[i]->set_new_loc(New1);
            count++;
            if(count == 4*j)
            {
                j++;
                count = 1;
            }
        }
    }
}