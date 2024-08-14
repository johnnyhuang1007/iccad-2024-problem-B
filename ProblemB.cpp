#include <iostream>
#include <string>
#include <list>
#include <vector>
#include <map>
#include <unordered_set>
#include <unordered_map>
#include <utility>
#include <fstream>

using namespace std;

struct pair_hash {
    template <class T1, class T2>
    size_t operator () (const pair<T1,T2> &p) const {
        auto h1 = hash<T1>{}(p.first);
        auto h2 = hash<T2>{}(p.second); 
        return h1 ^ h2;  
    }
};

struct PlacementRows{
    int SiteX, SiteY, SiteWidth, SiteHeight, NumSite;
};

struct Pin{
    string pinName;
    char pinType;                  //D or Q or C for FF, I or O for Gate
    int pinX, pinY;                //Pin's relative position
};

struct Flipflop{
    string FFName;
    int bitCount, pinCount;
    int FFWidth, FFHeight;
    double Qdelay;
    double power;
    unordered_map<string, Pin> pin_Map;
};

struct Gate{    
    string gName;
    int pinCount;
    int gWidth, gHeight;
    unordered_map<string, Pin> pin_Map;
};

struct Net{
    string netName;
    int pinCount;
    unordered_map<string, unordered_set<string>> connected_pin;        //Inst name, unordered_set<pin name>
};

class InstanceG{
    private:
        string instName;
        string instType;
        int instX, instY;                                       //Instance's actual leftdown position
        Gate* Gptr = nullptr;                                   //points to specific Gate type
    public:
        InstanceG();
        ~InstanceG();
    friend class Circuit;
};

class InstanceFF{
    private:
        string instName;
        string instType;
        string clkDomain;                                       //FF's clk domain(clk pin is here)
        int instX, instY;                                       //Instance's actual leftdown position
        Flipflop* FFptr = nullptr;                              //points to specific FF type
        double slack = 0;
        map<InstanceFF*, vector<Pin*>> clustered_FFpin;         //store all the clustered FF's pin(derived form different InstFF), only QD, except clk
        vector<pair<string, string>> able_pins;                 //Q, D
        unordered_map<string, vector<pair<string, string>>> pin_mapping;              //(new pin name, vector<oldff name, oldff pin name>)
    public:
        InstanceFF();
        ~InstanceFF();
        void initial_able_pins();
        void initial_pin_mapping();
        bool clkuplow();                                        //true for up, false for low
    friend class Circuit;
};

class Circuit{
    private:
        unordered_map<string, string> Type_Lib;                              //inst(in/output) name, type(wacch out new insff name isn't in this map)
        unordered_map<string, Gate> G_Lib;                                   //Gate name, Gate
        unordered_map<string, Pin> In_Lib;                                   //Net name, Net
        unordered_map<string, Pin> Out_Lib;                                  //Net name, Net
        unordered_map<string, Flipflop> FF_Lib;                              //FF name, FF
        unordered_map<int , vector<Flipflop*>> FF_Lib_bybit;                 //FF bitCount, FF           

        unordered_map<string, InstanceFF> InstFF_Lib;                        //Instance name, Instance
        unordered_map<string, InstanceG> InstG_Lib;                          //Instance name, Instance
        unordered_map<string, map<int, map<int, InstanceFF*>>> InstFF_xmp;       //Instance(FF) <clk domain, <xcoor, <ycoor, Instance>>>
        unordered_map<string, map<int, map<int, InstanceFF*>>> InstFF_ymp;       //Instance(FF) <clk domain, <ycoor, <xcoor, Instance>>>

        unordered_map<pair<string, string>, Net*, pair_hash> PfindN_mp;      //<Instance(FF) name, "new" pin name> or <Instance(G) or "input" or "output" , pin name>, Net
        unordered_map<string, Net> N_Lib;                                    //Net name, Net

        unordered_set<string> Inst_FF_name;                       //all input Instance(FF) name
        unordered_set<string> Legal_new_Inst_name;                //Legal Instance name(starts from reg(numofFF+1) to reg(numofFF+totalbit))
        vector<PlacementRows> PlacementRows_vec;                  //PlacementRows vector

        vector<InstanceFF> InstFF_newvec;                         //store all the new instanceFF(each new instanceFF is a cluster of FFs)
        unordered_map<string, int> Instance_indexmp;              //Instance name, index in the hMetis file
        double Alpha, Beta, Gamma, Lambda;
        int Die_leftdownX, Die_leftdownY, Die_rightupX, Die_rightupY;
        int BinWidth, BinHeight, BinMaxUtil;
        double DisplacementDelay;

        int numofFF = 0, numofGate = 0;
        int maxFFbit=0;
    public:
        Circuit();
        ~Circuit();
        void get_legal_instname();
        void getK_cluster(ofstream&); //get k clusters, return all the clustered instance
        void dividearea(int, int, int, int, char, string, vector<vector<InstanceFF*>>&); //char direc 'v' or 'h'
        int countFFbit(int, int, int, int, string); //count total Instance(FF) bit in the area
        vector<InstanceFF*> findFF(int, int, int, int, string); //find all Instance(FF) in the area(with the same clk domain)
        pair<int, int> get_avgcenter(vector<InstanceFF*>); //get the average center of the cluster
        InstanceFF FF_CLustering(vector<InstanceFF*>);
        void updatePfindN_mp();

        void result_to_hMetis(ofstream&);
        void indexmap_to_hMetis(ofstream&);
        void printmapping_result(ofstream&);
        void readInput(ifstream&);
        void printFF(ofstream&);
        void printGate(ofstream&);
        void printPfindN(ofstream&);
        void printNet(ofstream&);
        void printInstanceFF(ofstream&);
};


int main(int argc, char *argv[]) {
    ifstream infile(argv[1]);

    ofstream outfile(argv[2]);
    //infile.open("sampleCase.txt");
    //infile.open("testcase1.txt");
    //outfile.open("output.txt");

    Circuit circuit;
    circuit.readInput(infile);
    infile.close();

    //circuit.printPfindN(outfile);
    //circuit.printNet(outfile);

    circuit.get_legal_instname();
    circuit.getK_cluster(outfile);
    circuit.updatePfindN_mp();
    circuit.printmapping_result(outfile);
    outfile.close();
    /*
    outfile.open("mapindex.txt");
    circuit.indexmap_to_hMetis(outfile);
    outfile.close();

    outfile.open("input.hgr");
    circuit.result_to_hMetis(outfile);
    outfile.close();
    */

    return 0;
}

Circuit::Circuit(){
}

Circuit::~Circuit(){
}

InstanceFF::InstanceFF(){
}

InstanceFF::~InstanceFF(){
}

InstanceG::InstanceG(){
}

InstanceG::~InstanceG(){
}

void InstanceFF::initial_able_pins(){
    if(FFptr->bitCount == 1)
        able_pins.push_back(make_pair("Q", "D"));
    else{
        for(int i = 0; i < FFptr->bitCount; i++)
            able_pins.push_back(make_pair("Q"+to_string(i), "D"+to_string(i)));
    }
    return;
}

void InstanceFF::initial_pin_mapping(){
    for(const auto& it: clustered_FFpin){                  //map<InstanceFF*, vector<Pin*>>
        for(const auto& it2: it.second){                   //vector<Pin*>
            if(it2->pinType == 'Q'){                       //Pin* (comes from same FF)
                if(it2->pinName.length() != 1){            //Q0, Q1, Q2...
                    string subname = it2->pinName.substr(1);
                    pin_mapping[able_pins.back().first].push_back(make_pair(it.first->instName, it2->pinName));
                    pin_mapping[able_pins.back().second].push_back(make_pair(it.first->instName, "D"+subname));
                    able_pins.pop_back();
                 }
                 else{                                     //Q, D
                    pin_mapping[able_pins.back().first].push_back(make_pair(it.first->instName, it2->pinName));
                    pin_mapping[able_pins.back().second].push_back(make_pair(it.first->instName, "D"));
                    able_pins.pop_back();
                 }
            }
            else if(it2->pinType == 'C'){
                if(clkuplow()) pin_mapping["CLK"].push_back(make_pair(it.first->instName, it2->pinName));
                else pin_mapping["clk"].push_back(make_pair(it.first->instName, it2->pinName));
            }
        }
    }
    return;
}

bool InstanceFF::clkuplow(){
    for(const auto& it:FFptr->pin_Map){
        if(it.second.pinType == 'C'){
            if(it.second.pinName[0] == 'C')
                return true;
            else if(it.second.pinName[0] == 'c')
                return false;
        }
    }
}




void Circuit::get_legal_instname(){
    int totalbit = 0;
    for(auto it = InstFF_Lib.begin(); it != InstFF_Lib.end(); it++)
        totalbit += it->second.FFptr->bitCount;
    for(int i = numofFF+1; i <= numofFF+totalbit; i++)
        Legal_new_Inst_name.insert("reg"+to_string(i));
}

void Circuit::getK_cluster(ofstream& outfile){
    vector<vector<InstanceFF*>> Cluster_vec;           //store all the clustered instanceFF(vecotr<InstanceFF*> stands for one cluster)
    for(const auto& it: InstFF_xmp){                       //cluster all the FFs in the circuit
        string clkDomain = it.first;
        auto starx = InstFF_xmp[clkDomain].begin();
        auto endx = InstFF_xmp[clkDomain].rbegin();
        auto stary = InstFF_ymp[clkDomain].begin();
        auto endy = InstFF_ymp[clkDomain].rbegin();
        dividearea(starx->first, endx->first, stary->first, endy->first, 'v', clkDomain, Cluster_vec);
    }
    //cout <<"Clusters: "<< Cluster_vec.size() << endl;
    for(const auto& it: Cluster_vec){
        //outfile << "Clk: " << it[0]->clkDomain << endl;
        //for(const auto& it2: it){
            //outfile << it2->instName << " ";
            //outfile << it2->instName << " ";
        //}
        //outfile << endl;
        InstFF_newvec.push_back(FF_CLustering(it));
        //outfile << endl;
    }
    //outfile << endl;
}

void Circuit::dividearea(int startx ,int endx, int starty,  int endy, char direc, string clk, vector<vector<InstanceFF*>>& Cluster_vec){
    int count = countFFbit(startx, endx, starty, endy, clk);
    if(count == 0)
        return;
    else if((count <= maxFFbit) && (FF_Lib_bybit.find(count) != FF_Lib_bybit.end())){
        vector<InstanceFF*> FF_vec = findFF(startx, endx, starty, endy, clk);
        Cluster_vec.push_back(FF_vec);
        return;
    }
    else{
        if(direc == 'v'){
            if(startx == endx)  //only one column, can't cut vertically
                dividearea(startx, endx, starty, endy, 'h', clk, Cluster_vec);
            else{
                int midx = (startx + endx) / 2;
                dividearea(startx, midx, starty, endy, 'h', clk, Cluster_vec);
                dividearea(midx+1, endx, starty, endy, 'h', clk, Cluster_vec);
            }
        }
        else if(direc == 'h'){
            if(starty == endy)  //only one row, can't cut horizontally
                dividearea(startx, endx, starty, endy, 'v', clk, Cluster_vec);
            else{
                int midy = (starty + endy) / 2;
                dividearea(startx, endx, starty, midy, 'v', clk, Cluster_vec);
                dividearea(startx, endx, midy+1, endy, 'v', clk, Cluster_vec);
            }
        }
    }
}

int Circuit::countFFbit(int startx, int endx, int starty, int endy, string clk){
    int count = 0;
    for(auto it: InstFF_xmp[clk]){
        if(it.first >= startx && it.first <= endx){
            for(auto it2: it.second){
                if(it2.first >= starty && it2.first <= endy)
                    count += it2.second->FFptr->bitCount;
            }
        }
    }
    return count;
}

vector<InstanceFF*> Circuit::findFF(int startx, int endx, int starty, int endy, string clk){
    vector<InstanceFF*> FF_vec;
    for(const auto& pair : InstFF_xmp[clk]){                            //map<int, map<int, InstanceFF*>> <xcoor, <ycoor, Instance>>
        if(pair.first >= startx && pair.first <= endx){
            for(const auto& pair2 : pair.second){                   //map<int, InstanceFF*> <ycoor, Instance>
                if(pair2.first >= starty && pair2.first <= endy)
                    FF_vec.push_back(pair2.second);
            }
        }
    }
    return FF_vec;
}

pair<int, int> Circuit::get_avgcenter(vector<InstanceFF*> InstFF_vec){
    int sum_x = 0, sum_y = 0, avg_x = 0, avg_y = 0;
    for(auto it = InstFF_vec.begin(); it != InstFF_vec.end(); it++){
        sum_x += (*it)->instX;
        sum_y += (*it)->instY;
    }
    avg_x = sum_x / InstFF_vec.size();
    avg_y = sum_y / InstFF_vec.size();
    return make_pair(avg_x, avg_y);
}

InstanceFF Circuit::FF_CLustering(vector<InstanceFF*> InstFF_vec){
    InstanceFF new_instFF;
    pair<int, int> avgcenter = get_avgcenter(InstFF_vec);
    for(const auto& it : InstFF_vec){                                                    //giving the new instance new name
        if(Inst_FF_name.find(it->instName) != Inst_FF_name.end()) continue;              //the name was from initial input, not available
        else Legal_new_Inst_name.insert(it->instName);                                   //the name was from Legal_set, available(release back to Legal_set)
    }
    auto it = Legal_new_Inst_name.begin();
    new_instFF.instName = *it;
    Legal_new_Inst_name.erase(it);
    
    new_instFF.instType = "flipflop";                                             //giving the new instance type
    new_instFF.instX = avgcenter.first;                                           //giving the new instance position
    new_instFF.instY = avgcenter.second;
    new_instFF.clkDomain = InstFF_vec[0]->clkDomain;                              //giving the new instance clk domain

    int totalbit = 0;
    for(const auto& it : InstFF_vec){                                            //vector<InstanceFF*>
        for(const auto& it2 : it->clustered_FFpin){                              //map<InstanceFF*, vector<Pin*>
            for(const auto& it3 : it2.second)                                    //vector<Pin*>
                new_instFF.clustered_FFpin[it2.first].push_back(it3);   
        }
        totalbit += it->FFptr->bitCount;
    }
    new_instFF.FFptr = FF_Lib_bybit[totalbit][0];                            //giving the new instance FF type(depeand on the total bit of the clustered FFs)
    new_instFF.initial_able_pins();                                          //giving the new instance able pins
    new_instFF.initial_pin_mapping();                                        //giving the new instance pin mapping
    //slack hasn't been assigned yet
    return new_instFF;
}

void Circuit::updatePfindN_mp(){
    for(const auto& it: InstFF_newvec){                                     //vector<InstanceFF>
        string newname = it.instName;
        for(const auto& it2: it.pin_mapping){                               //map<string, vector<pair<string, string>>>
            string newpin = it2.first;
            for(const auto& it3: it2.second){                               //vector<pair<string, string>>
                string oldff = it3.first;
                string oldpin = it3.second;
                Net* netptr = PfindN_mp[{it3.first, it3.second}];           //find the net the old pin is connected tos
                //cout << "old pin: " << it3.first << "/" << it3.second << " net: " << netptr->netName << endl;
                //cout << "new pin: " << it.instName << "/" << it2.first << " net: " << netptr->netName << endl;
                PfindN_mp.erase({it3.first, it3.second});                   //erase the old pin
                PfindN_mp[{it.instName, it2.first}] = netptr;               //assign the new pin
                PfindN_mp[{it.instName, it2.first}]->connected_pin[it.instName].insert(it2.first);
                PfindN_mp[{it.instName, it2.first}]->connected_pin.erase(it3.first);
            }
        }
    }
}











void Circuit::readInput(ifstream& infile){
    string object;
    while (!infile.eof()){
        infile >> object;
        if(object == "Alpha"){
            infile >> Alpha;
        }
        else if(object == "Beta"){
            infile >> Beta;
        }
        else if(object == "Gamma"){
            infile >> Gamma;
        }
        else if(object == "Lambda"){
            infile >> Lambda;
        }
        else if(object == "DieSize"){
            infile >> Die_leftdownX >> Die_leftdownY >> Die_rightupX >> Die_rightupY;
        }
        else if(object == "NumInput"){
            int numInput;
            infile >> numInput;
            for(int i = 0; i < numInput; i++){
                string temp;
                Pin pin;
                infile >> temp >> pin.pinName >> pin.pinX >> pin.pinY;
                In_Lib[pin.pinName] = pin;
                Type_Lib[pin.pinName] = "input";
            }
        }
        else if(object == "NumOutput"){
            int numOutput;
            infile >> numOutput;
            for(int i = 0; i < numOutput; i++){
                string temp;
                Pin pin;
                infile >> temp >> pin.pinName >> pin.pinX >> pin.pinY;
                Out_Lib[pin.pinName] = pin;
                Type_Lib[pin.pinName] = "output";
            }
        }
        else if(object == "FlipFlop"){
            Flipflop FF;
            infile >> FF.bitCount >> FF.FFName >> FF.FFWidth >> FF.FFHeight >> FF.pinCount;
            if(FF.bitCount > maxFFbit)
                maxFFbit = FF.bitCount;
            for(int i = 0; i < FF.pinCount; i++){
                Pin pin;
                string temp;
                infile >> temp >> pin.pinName >> pin.pinX >> pin.pinY;
                if((pin.pinName[0] == 'Q')||(pin.pinName[0] == 'q'))
                    pin.pinType = 'Q';
                else if((pin.pinName[0] == 'D')||(pin.pinName[0] == 'd'))
                    pin.pinType = 'D';
                else if((pin.pinName[0] == 'C')||(pin.pinName[0] == 'c'))
                    pin.pinType = 'C';
                FF.pin_Map[pin.pinName] = pin;
            }
            FF_Lib[FF.FFName] = FF;
            FF_Lib_bybit[FF.bitCount].push_back(&FF_Lib[FF.FFName]);
            Type_Lib[FF.FFName] = "flipflop";
        }
        else if(object == "Gate"){
            Gate G;
            infile >> G.gName >> G.gWidth >> G.gHeight >> G.pinCount;
            for(int i = 0; i < G.pinCount; i++){
                Pin pin;
                string temp;
                infile >> temp >> pin.pinName >> pin.pinX >> pin.pinY;
                if((pin.pinName[0] == 'I')||(pin.pinName[0] == 'i'))
                    pin.pinType = 'I';
                else if((pin.pinName[0] == 'O')||(pin.pinName[0] == 'o'))
                    pin.pinType = 'O';
                G.pin_Map[pin.pinName] = pin;
            }
            G_Lib[G.gName] = G;
            Type_Lib[G.gName] = "gate";
        }
        else if(object == "NumInstances"){
            int numInstances;
            infile >> numInstances;
            for(int i = 0; i < numInstances; i++){
                string temp, inst_name, cell_name;
                infile >> temp >> inst_name >> cell_name;
                if(Type_Lib[cell_name] == "flipflop"){
                    InstanceFF instFF;
                    instFF.instName = inst_name;
                    instFF.instType = "flipflop";
                    infile >> instFF.instX >> instFF.instY;
                    Inst_FF_name.insert(instFF.instName);
                    instFF.FFptr = &FF_Lib[cell_name];
                    for(auto& it : instFF.FFptr->pin_Map)
                        instFF.clustered_FFpin[&InstFF_Lib[inst_name]].push_back(&it.second);
                    InstFF_Lib[instFF.instName] = instFF;
                    Type_Lib[instFF.instName] = instFF.instType;
                    numofFF++;
                }
                else if(Type_Lib[cell_name] == "gate"){
                    InstanceG instG;
                    infile >> instG.instX >> instG.instY;
                    instG.instName = inst_name;
                    instG.instType = "gate";
                    instG.Gptr = &G_Lib[cell_name];
                    InstG_Lib[instG.instName] = instG;
                    Type_Lib[instG.instName] = instG.instType;
                    numofGate++;
                }
            }
        }
        else if(object == "NumNets"){
            int numNets;
            infile >> numNets;
            for(int i = 0; i < numNets; i++){
                Net net;
                string temp;
                infile >> temp >> net.netName >> net.pinCount;
                for(int j = 0; j < net.pinCount; j++){
                    string full_name, cell_name, pin_name;
                    infile >> temp >> full_name;
                    cell_name = full_name.substr(0, full_name.find_first_of('/'));
                    pin_name = full_name.substr(full_name.find_first_of('/')+1);
                    //cout << "cell_name: " << cell_name << " pin_name: " << pin_name << endl;
                    if(Type_Lib[cell_name] == "flipflop"){
                        net.connected_pin[cell_name].insert(pin_name);
                        PfindN_mp[{cell_name, pin_name}] = &N_Lib[net.netName];
                        if((pin_name == "CLK")||pin_name == "clk")
                            InstFF_Lib[cell_name].clkDomain = net.netName;
                    }
                    else if(Type_Lib[cell_name] == "gate"){
                        net.connected_pin[cell_name].insert(pin_name);
                        PfindN_mp[{cell_name, pin_name}] = &N_Lib[net.netName];
                    }
                    else if(Type_Lib[cell_name] == "input"){
                        net.connected_pin["input"].insert(cell_name);
                        PfindN_mp[{"input", cell_name}] = &N_Lib[net.netName];
                    }
                    else if(Type_Lib[cell_name] == "output"){
                        net.connected_pin["output"].insert(cell_name);
                        PfindN_mp[{"output", cell_name}] = &N_Lib[net.netName];
                    }
                }
                N_Lib[net.netName] = net;
            }
        }
        else if(object == "BinWidth"){
            infile >> BinWidth;
        }
        else if(object == "BinHeight"){
            infile >> BinHeight;
        }
        else if(object == "BinMaxUtil"){
            infile >> BinMaxUtil;
        }
        else if(object == "PlacementRows"){
            PlacementRows PR;
            infile >> PR.SiteX >> PR.SiteY >> PR.SiteWidth >> PR.SiteHeight >> PR.NumSite;
            PlacementRows_vec.push_back(PR);
        }
        else if(object == "QpinDelay"){
            string FF_name;
            infile >> FF_name;
            infile >> FF_Lib[FF_name].Qdelay;
        }
        else if(object == "DisplacementDelay"){
            infile >> DisplacementDelay;
        }
        else if(object == "TimingSlack"){
            string inst_name, temp;
            infile >> inst_name >> temp;
            infile >> InstFF_Lib[inst_name].slack;
        }
        else if(object == "GatePower"){
            string cell_name;
            infile >> cell_name;
            infile >> FF_Lib[cell_name].power;
        }
    }
    for(const auto& it: InstFF_Lib){
        InstFF_xmp[it.second.clkDomain][it.second.instX][it.second.instY] = &InstFF_Lib[it.second.instName];
        InstFF_ymp[it.second.clkDomain][it.second.instY][it.second.instX] = &InstFF_Lib[it.second.instName];
    }
}

void Circuit::printFF(ofstream& outfile){
    cout << "Number of FlipFlop type: "<< FF_Lib.size() << endl;
    for(const auto& it: FF_Lib){
        cout << "FlipFlop: ";
        cout << it.second.FFName << " Width: " << it.second.FFWidth << " Height: " << it.second.FFHeight << " bitCount: " << it.second.bitCount << " pinCount: " << it.second.pinCount << endl;
        for(const auto& it2: it.second.pin_Map){
            cout << "Pin:" << it2.second.pinName << " type: " << it2.second.pinType << " X: " << it2.second.pinX << " Y: " << it2.second.pinY << endl;
        }
        
    }
}

void Circuit::printGate(ofstream& outfile){
    cout << "Number of Gate type: "<< G_Lib.size() << endl;
    for(auto it = G_Lib.begin(); it != G_Lib.end(); it++){
        cout << "Gate: ";
        cout << it->second.gName << " " << it->second.gWidth << " " << it->second.gHeight << " " << it->second.pinCount << endl;
        for(auto it2 = it->second.pin_Map.begin(); it2 != it->second.pin_Map.end(); it2++){
            cout << it2->first << " " << it2->second.pinX << " " << it2->second.pinY << endl;
        }
    }
}

void Circuit::printInstanceFF(ofstream& outfile){
    cout << "Number of InstanceFF: "<< InstFF_Lib.size() << endl;
    for(auto it = InstFF_Lib.begin(); it != InstFF_Lib.end(); it++){
        cout << it->second.instName << " " << it->second.instType << " " << it->second.instX << " " << it->second.instY << endl;
        cout << "FF: " << it->second.FFptr->FFName << " " << it->second.FFptr->FFWidth << " " << it->second.FFptr->FFHeight << " " << it->second.FFptr->bitCount << " " << it->second.FFptr->pinCount << endl;
        for(auto it2 = it->second.clustered_FFpin.begin(); it2 != it->second.clustered_FFpin.end(); it2++){
            cout << "instname: " << it2->first->instName << " ";
            cout << "size: " << it2->second.size() << endl;
            for(auto it3 = it2->second.begin(); it3 != it2->second.end(); it3++)
                cout <<" pintype: "<< (*it3)->pinType << " ";
            cout << endl;
        }
        cout << "clkDomain: " << it->second.clkDomain << endl;
        cout << "slack: " << it->second.slack << endl;
        cout << endl;
    }
}

void Circuit::printPfindN(ofstream& outfile){
    cout << "Number of Pin: "<< PfindN_mp.size() << endl;
    for(const auto& it: PfindN_mp){
        cout << "Instance: " << it.first.first << " Pin: " << it.first.second << " Net: " << it.second->netName << endl;
    }
}

void Circuit::printNet(ofstream& outfile){
    cout << "Number of Net: "<< N_Lib.size() << endl;
    for(const auto& it: N_Lib){
        cout << "Net: " << it.second.netName << " " << it.second.pinCount << endl;
        for(const auto& it2: it.second.connected_pin){
            cout << "instname: " << it2.first;
            cout << " Pin: ";
            for(const auto& it3: it2.second)
                cout << it3 << " ";
            cout << endl;
        }
        cout << endl;
    }
}

void Circuit::printmapping_result(ofstream& outfile){
    outfile << "CellInst " << InstFF_newvec.size() << endl;
    //cout << "CellInst " << InstFF_newvec.size() << endl;
    for(const auto& it:InstFF_newvec){
        outfile << "Inst " << it.instName << " " << it.FFptr->FFName << " " << it.instX << " " << it.instY << endl;
        //cout << "Inst " << it.instName << " " << it.FFptr->FFName << " " << it.instX << " " << it.instY << endl;
    }
    for(const auto& it:InstFF_newvec){                                     //vector<InstanceFF>
        for(const auto& it2: it.pin_mapping){                              //map<string, vector<pair<string, string>>>
            for(const auto& it3: it2.second){                               //vector<pair<string, string>>
                outfile << it3.first << "/" << it3.second << " map " << it.instName << "/" << it2.first << endl;
                //cout << it3.first << "/" << it3.second << " map " << it.instName << "/" << it2.first << endl;
            }
        }
    }
}

void Circuit::result_to_hMetis(ofstream& outfile){
    unordered_map<string, bool> Net_containFF;
    int numofnet_connectedFF = 0;
    for(const auto& it: N_Lib){                             //Net name, Net
        bool flag = false;
        for(const auto& it2: it.second.connected_pin){       //Inst name, unordered_set<pin name>
            string type = Type_Lib[it2.first];
            if((type == "gate")||(it2.first == "input")||(it2.first == "output")) continue;
            else flag = true;
        }
        Net_containFF[it.first] = flag;
        if(flag) numofnet_connectedFF++;
    }
    outfile << numofnet_connectedFF << " " << InstFF_newvec.size() << endl;
    //cout << numofnet_connectedFF << " " << InstFF_newvec.size() << endl;

    for(const auto& it: N_Lib){                             //Net name, Net
        if(Net_containFF[it.first]){
            //outfile << it.second.netName << " ";
            for(const auto& it2: it.second.connected_pin){       //Inst name, unordered_set<pin name>
                string type = Type_Lib[it2.first];
                if((type == "gate")||(it2.first == "input")||(it2.first == "output")) continue;
                else{
                    outfile << Instance_indexmp[it2.first] << " ";
                    //cout << Instance_indexmp[it2.first] << " ";
                }
            }
            outfile << endl;
            //cout << endl;
        }
    }

}

void Circuit::indexmap_to_hMetis(ofstream& outfile){
    int count = 1;
    for(const auto& it: InstFF_newvec){
        Instance_indexmp[it.instName] = count;
        outfile << it.instName << " map to hMetis index " << count << endl;
        count++;
    }
}