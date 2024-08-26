#ifndef _MODULE_EXT_H_
#define _MODULE_EXT_H_
#include"module.h"
#include<vector>
#include<string>
#include<iostream>
#include<list>

class Inst;
struct net;


struct Pin
{

    net* belong_net = NULL;
    net* clk_belong = NULL; //net or pin? net

    std::string org_type;
    Inst* orginal_Inst = NULL;
    Point org_relative_loc;

    std::string pin_type;   //e.g. D0
    Inst* belong_Inst = NULL;
    Point relative_loc; //if belong inst is null then it's absolute

    char type; //fanin or fanout or clk I/O/C

    double slack = 0;   //slack = require_time - arrival_time
    bool slack_determined = 0;
    bool recursive = 0;
    double arrival_time = 0;
    double require_time = 0;
    double critical_weight = 1;
    Pin* matching_Pin = NULL;
    std::list<Pin*> path_seq;
    void delay_free_value_determine();  //only for Dpin
    std::vector<double> slack_propagation(Pin*);
    std::vector<double> slack_propagation(){return slack_propagation(this);}
    std::vector<double> slack_propagation(Pin* modified_Pin,bool start);
    void slack_trace_back();
    double get_slack();
    void slack_cal();
    void arrival_time_propagation();

    Pin(const Pin& copy);
    Pin(){ this->recursive = 0; }
    ~Pin();
    Point abs_loc();
    std::string get_name();
};

Pin* binding(Pin*,Pin*); //master,slave

double HPWL(Pin* a, Pin* b);

struct net
{
    double displacement_delay = 0;
    double HPWL = 0;
    double WEIGHT = 1;
    double set_HPWL();
    std::vector<Point> set_center(); //0 for froms 1 for tos
    std::vector<Point> set_weight_center();
    Point center_of_FROMs;
    Point center_of_TOs;
    std::string name;
    std::vector<Pin*> relative;    
    std::vector<Pin*> TOs;
    double TOs_weight = 1;
    std::vector<Pin*> FROMs;
    double FROMs_weight = 1;
    
    std::vector<Pin*> CLKs;
    net(){}
    ~net(){}
};

struct Inst_data
{
    int width = 0;
    int height = 0;
    int bits = 0;
    int pinCount = 0;
    double QpinDelay = 0;
    double power = 0;
    std::string name = "";
    std::vector<Pin> Pin_set;
    Inst_data(){}
    Inst_data(int,int,int,int,std::string,std::vector<Pin>);
    ~Inst_data();
};

class Inst:public Fixed_Module
{
    public:
    std::pair<int,int> corr_Lib = std::pair<int,int>(-1,-1);
    Inst_data* corr_data = NULL;
    int idx;
    //for FF
    std::vector<Pin*> INs;
    std::vector<Pin*> OUTs;
    std::vector<Pin*> CLK;
    net* clk_domain;

    //all pins
    std::vector<Pin*> Pins;

    double slack_val;
    std::list<Inst*> max_slack_sequence;

    bool is_gate(){return corr_data->name[0]!='F';}
    bool is_ff(){return corr_data->name[0]=='F';}

    bool inserted = 0;
    Inst(std::string a, Point b, int c, int d);//name ld height width    
    Inst(std::string name, Point LD, Inst_data* data,std::pair<int,int> T_Idx );
    Inst(std::string name, Point LD, Inst_data* data,std::pair<int,int> T_Idx ,std::string NOT_To_NEW);
    Inst(Inst*);
    virtual ~Inst(){}

    Point FF_RightUp(){return this->LeftDown() + Point(this->corr_data->height,this->corr_data->width);}
    Point Tile_RightUp(){return RU(get_root());}

    Point center(){return LeftDown() + Point(corr_data->height/2,corr_data->width/2);}
    void set_new_loc(Point); //it set new loc, but it isn't responsible to insertion
    void set_DQC(); // it move pins from pins to its type, and sorted in order;
};

std::pair<std::string,std::string> split_by_symbol(std::string name_org);
 
Pin* pair_D(Inst*,Pin*);
Pin* pair_Q(Inst*,Pin*);
std::pair<double,Pin*> max_arrival_time(std::vector<Pin*>);

std::ostream& operator<<(std::ostream&,Pin);
std::ostream& operator<<(std::ostream&,Inst_data);

double HPWL(net*);

int height(Inst* cur);
int width(Inst* cur);


#endif


