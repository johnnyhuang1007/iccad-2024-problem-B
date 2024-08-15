#ifndef _P_EXT_H_
#define _P_EXT_H_
#include<utility>
#include<string>
#include<vector>
#include<unordered_map>
#include "Plane.h"
#include "module_extension.h"
#define T_Idx pair<int,int> //T: 0 for FF 1 for G 
struct Bin
{
    Point left_down = Point(0,0);
    long width = 0;
    long height = 0;
    long used_area = 0;
    double smoothen_area = 0;
    std::vector<Inst*> corr_inst;
};

struct PlacementRow
{
    Point left_down;
    int siteWidth;
    int siteHeight;
    int count;
};



class Plane_E : public Plane
{
    private:
        
        double alpha;
        double beta;
        double gamma;
        double lambda;
        
        
        void SET_FF_GATE_INFO(std::ifstream&);
        void SET_FF_GATE_LIB(std::ifstream&);
        void SET_NET(std::ifstream&);
        void SET_COST_INFO(std::ifstream&);
        
        

        double BinWidth;
        double BinHeight;
        double BinMaxUtil;
        double negative_slack;
        double positive_slack;
        double HPWL;
        double area;
        double power;
        double max_util;
        double displacement_delay;
        int violated_bins_cnt = 0;
        double smoothen_bin_util = 0;
        long FF_total_area = 0;
        double FF_total_power = 0;
        long long int name_cnter = 0;
    public:
        

        std::vector<Inst_data> FF_lib;
        std::unordered_map<std::string,Inst_data*> FF_lib_u;
        std::vector<std::vector<Inst_data*>> FF_lib_bits;
        std::vector<Inst_data> G_lib;
        std::vector<Inst*> G_list;
        std::unordered_map<std::string,Inst*> G_list_u;
        
        std::unordered_map<std::string,Inst*> FF_list_u; //for seraching usage
        std::vector<Inst*> FF_list; //original FF
        std::vector<Inst*> FF_list_bank;
        std::vector<std::vector<Bin>> Bins;
        
      
        std::unordered_map<std::string,Inst*> In_list; //intput (need to be inserted)
        std::unordered_map<std::string,Inst*> Out_list;//output (need to be inserted)
        std::vector<Pin*> Inst_Pins;    //inside the FF (no need to be inserted)
        std::vector<Pin*> evaluation_Pins;

        std::vector<net*> net_list;
        //std::vector<std::vector<Pin*>> same_domain_d_pins;

        //finder
        Pin* find_pin_by_FFsPinName(std::string);   //e.g. reg0/D1
        Inst* find_inst_by_name(std::string);
        Pin* find_pin_by_name(std::string,Inst*);
        Inst_data* find_data_by_name(std::string);
        std::pair<int,int> find_lib_by_name(std::string name);
        Inst_data* find_lib_by_name_ptr(std::string name);

        //inst insertion
        bool insert_inst(Inst*);
        bool insert_inst(Inst*, Tile*);//speed up process;
        bool remove_Inst(Inst*);
        void FFs_insertion();
        
        //legality checker
        std::vector<PlacementRow> PlacementRows;
        std::vector<Inst*> get_not_legal_location();    //for location only don't care clk domain, pin data etc
        bool is_legal();
        void location_legalization(std::vector<Inst*>);
        void location_legalization(){location_legalization(this->FF_list);};

        //L.D. loc getter
        void location_to_P_Row(std::vector<Inst*>);
        void location_to_P_Row(){location_to_P_Row(this->FF_list);};
        Point random();
        Point closest_Legal_locs(Point);
        

        //L.D. loc setter
        void update_placementRow();
        void set_to_placementRow(Inst*);
        void set_to_placementRow();
        void loc_randomize();

        //slack computation
        void IOC_distinguish();
        void required_time_init();
        void all_Inst_slack_cal();
        void Inst_slack_cal(Inst*);
        void pin_slack_cal(Pin*);
        void slack_trace_back();
        void slack_propagation();
        void slack_propagation(Inst*);

        //bin computation
        void add_util(Inst*);
        void min_util(Inst*);

        void add_smooth_util(Inst*);
        void min_smooth_util(Inst*);


        //bank and debank
        Inst* bank(std::vector<Inst*>, Inst_data*);
        std::vector<Inst*> debank(Inst*, std::vector<Inst_data*>);

        //placer
        double set_on_site();
        void insert_FFs();
        
        //optimizer
        void sequence_finder();
        void sequence_finder(Inst*);
        double HPWL_optimizer(double);
        double slack_optimizer();
        void move_and_propagate(Inst*,Point);
        void set_and_propagate(Inst*, Point);
        void unit_move_and_propagate(Inst*,std::string,int);  //"UP" "DOWN" "LEFT" "RIGHT"
        void propagate(Inst*);
        
        void reduce_high_util_pin_weight();
        std::pair<std::pair<int,int>,std::pair<int,int>> occupied_bin_idx(Inst*);
        bool is_in_highly_used_bin(Inst*);
        //getter
        double get_n_slack(){return negative_slack;}
        double get_p_slack(){return positive_slack;}
        double cost();
        double optimize_cost();
        //READ_WRITE
        void read_output_format(std::string);
        void write_input_format(std::string);


        Plane_E(std::string input);
        void set_km_result(std::string input,std::string output);
        virtual ~Plane_E();
        
        void output(char*);
        void outimg();
        double eval();

        std::vector<std::vector<std::string>> mapping_FF();
};

double overlappingArea(Inst* i1,Inst* i2);
double overlappingArea_smoothen(Bin bin,Inst* i2);
int overlappingArea(Bin bin, Inst* inst);
double util(Bin cur);

#endif