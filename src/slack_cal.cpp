#include "Plane_extension.h"
#include <fstream>
#include<iomanip>
#include<stack>
#define u_map_itr(T) unordered_map<std::string,T>::const_iterator
using namespace std;

void Plane_E::slack_propagation()
{
    
    positive_slack = 0;
    negative_slack = 0;

    for(auto& IN:In_list)
    {
        Pin* input = IN.second->OUTs[0];
        input->slack_propagation();
    }

    for(int i = 0 ; i < evaluation_Pins.size() ; i++)
    {
        evaluation_Pins[i]->slack_propagation();
    }
    

    for(int i = 0 ; i < evaluation_Pins.size() ; i++)
    {
        if(evaluation_Pins[i]->slack > 0)
            positive_slack += evaluation_Pins[i]->slack;
        else
            negative_slack += evaluation_Pins[i]->slack;
    }


}

void print_seq(Pin* cur)
{
    for(auto& p:cur->path_seq)
        cout<<p->get_name()<<"  ";
    cout<<endl;
}

string Pin::get_name()
{
    string name = belong_Inst->get_name();
    name = name + "/" + pin_type;
    return name;
}

void Plane_E::required_time_init()  //it calculate initial slack value and the require time of each pin
{
    
    for(auto& IN:In_list)
    {
        Pin* input = IN.second->OUTs[0];
        input->arrival_time_propagation();
    }
    for(Pin* p:evaluation_Pins)
    {
        p->arrival_time_propagation();
    }
    for(Pin* p:evaluation_Pins)
    {
        p->require_time = p->slack + p->arrival_time;
    }
    negative_slack = 0;
    positive_slack = 0;
    for(int i = 0 ; i < evaluation_Pins.size() ; i++)
    {
        if(evaluation_Pins[i]->slack < 0)
            negative_slack+=evaluation_Pins[i]->slack;   //negative value
        else
            positive_slack+=evaluation_Pins[i]->slack;
    }
    
    HPWL = 0;
    for(int i = 0 ; i < FF_list_bank.size() ; i++)
    {
        
        for(Pin* p:FF_list_bank[i]->INs)
        {
            p->belong_net->set_center();
            Point HPWL_vec = p->belong_net->center_of_FROMs -  p->abs_loc();
            HPWL += abs(HPWL_vec.x) + abs(HPWL_vec.y);
        }
        for(Pin* p:FF_list_bank[i]->OUTs)
        {
            p->belong_net->set_center();
            Point HPWL_vec = p->belong_net->center_of_TOs -  p->abs_loc();
            HPWL += abs(HPWL_vec.x) + abs(HPWL_vec.y);
        }
    }
}


pair<double,Pin*> max_arrival_time(vector<Pin*> inputs)
{
    double value = -9999999999;
    Pin* N = NULL;
    for(int i = 0 ; i < inputs.size() ; i++)
    {
        if(inputs[i]->recursive)
        {
            value = inputs[i]->arrival_time;
            N = inputs[i];
            break;
        }
        if(inputs[i]->arrival_time > value)
        {
            value = inputs[i]->arrival_time;
            N = inputs[i];
        }
    }   
    return make_pair(value,N);
}

void Pin::arrival_time_propagation()
{


    stack<Pin*> to_prop;
    to_prop.push(this);
    
    int cnt =0 ;

    while(!to_prop.empty())
    {
        Pin* cur = to_prop.top();
        to_prop.pop();

        if(cur->recursive)
        {
            continue;
        }
        if(cur->pin_type == cur->belong_Inst->get_name() && cur->type == 'O')   //output from other chips
        {
            cur->arrival_time = 0;
            cur->path_seq.clear();
            cur->path_seq.push_back(cur);
            if(cur->belong_net==NULL)
                continue;
            for(int i = 0 ; i < cur->belong_net->TOs.size() ; i++)
                to_prop.push(cur->belong_net->TOs[i]);
        }
        else if(cur->pin_type[0] =='Q')  //FF output
        {
            double new_arrival = cur->belong_Inst->corr_data->QpinDelay;
            if(cur->arrival_time == new_arrival)
                continue;
            cur->arrival_time = new_arrival;
            cur->path_seq.clear();
            cur->path_seq.push_back(cur);
            if(cur->belong_net==NULL)
                continue;
            for(int i = 0 ; i < cur->belong_net->TOs.size() ; i++)
                to_prop.push(cur->belong_net->TOs[i]);
        }
        else if(cur->type == 'O')    //gate output
        {
            int count = 0;
            if(!cur->path_seq.empty())
            {
                for(Pin* p: cur->path_seq)
                {
                    if(p == cur)
                    {
                        count++;
                    }
                }
            }
            if(count >= 3)
            {
                vector<Pin*> p_seq;
                p_seq.resize(cur->path_seq.size());
                for(Pin* p: cur->path_seq)
                    p_seq.push_back(p);
                for(int i = p_seq.size()-1 ; i >= 0  ; i--)
                {
                    p_seq[i]->recursive = 1;
                    p_seq[i]->arrival_time = 200000000;
                    if(p_seq[i] == cur && i != p_seq.size()-1)
                        break;
                }
                continue;
            }
            
            pair<double,Pin*> new_arrival = max_arrival_time(cur->belong_Inst->INs);
            
            if(cur->arrival_time == new_arrival.first)
                continue;
            cur->arrival_time = new_arrival.first;
            cur->path_seq = new_arrival.second->path_seq;
            cur->path_seq.push_back(cur);
            if(cur->belong_net==NULL)
                continue;
            for(int i = 0 ; i < cur->belong_net->TOs.size() ; i++)
                to_prop.push(cur->belong_net->TOs[i]);
        }
        else if(cur->pin_type == cur->belong_Inst->get_name() && cur->type == 'I')
        {
            if(cur->belong_net==NULL)
                continue;
            double new_arrival = HPWL(cur->belong_net->FROMs[0],cur) + cur->belong_net->FROMs[0]->arrival_time;
            if(cur->arrival_time == new_arrival)
                continue;
            cur->path_seq = cur->belong_net->FROMs[0]->path_seq;
            cur->path_seq.push_back(cur);
            cur->arrival_time = new_arrival;
        }
        else if(cur->pin_type[0] == 'D')
        {
            if(cur->belong_net==NULL)
                continue;
            double new_arrival = HPWL(cur->belong_net->FROMs[0],cur) + cur->belong_net->FROMs[0]->arrival_time;
            if(cur->arrival_time == new_arrival)
                continue;
            cur->path_seq = cur->belong_net->FROMs[0]->path_seq;
            cur->path_seq.push_back(cur);
            cur->arrival_time = new_arrival;
            string next = cur->pin_type;
            next[0] = 'Q';
            Pin* Next = NULL;
            for(Pin* N:cur->belong_Inst->OUTs)
            {
                if(N->pin_type == next)
                {
                    Next = N;
                    break;
                }
            }
            to_prop.push(Next);
        }
        else if(cur->type == 'I')    //gate input
        {
            if(cur->belong_net==NULL)
                continue;
            double new_arrival = HPWL(cur->belong_net->FROMs[0],cur) + cur->belong_net->FROMs[0]->arrival_time;
            if(cur->arrival_time == new_arrival)
                continue;
            cur->path_seq = cur->belong_net->FROMs[0]->path_seq;
            cur->path_seq.push_back(cur);
            cur->arrival_time = new_arrival;
            
            for(int i = 0 ; i < cur->belong_Inst->OUTs.size() ; i++)
                to_prop.push(cur->belong_Inst->OUTs[i]);
        }
        else    //clk
        {
            continue;
        }
    }
}



void Plane_E::all_Inst_slack_cal()
{
    //doesn't make sense
    //for(int i = 0 ; i < evaluation_Pins.size() ; i++)
    //    evaluation_Pins[i]->slack_trace_back();
    for(int i = 0 ; i < G_list.size() ; i++)
    {
        for(Pin* p:G_list[i]->Pins)
        {
            if(p->belong_net==NULL)
            {
                p->slack = -999999999999;
                p->slack_determined = 1;
            }
        }
    }
    for(auto& INPUT_INST:In_list)
    {
        Inst* in = INPUT_INST.second;
        for(Pin* p:in->Pins)
        {
            p->slack = -999999999999;
            p->slack_determined = 1;
        }
    }
    for(auto& OUTPUT_INST:Out_list)
    {
        Inst* out = OUTPUT_INST.second;
        for(Pin* p:out->Pins)
        {
            if(p->belong_net==NULL)
            {
                p->slack = -999999999999;
                p->slack_determined = 1;
            }
        }
    }
}

void Plane_E::Inst_slack_cal(Inst* cur)
{
    for(int i = 0 ; i < cur->INs.size() ; i++)
    {
        pin_slack_cal(cur->INs[i]);
    }
}

void Plane_E::pin_slack_cal(Pin* cur)
{
    if(cur->slack_determined)
        return;
    net* belongNet = cur->belong_net;
    
    if(belongNet == NULL)//redundant pin (found in gate only)
        return;
    vector<Pin*> corrOut = belongNet->FROMs;
}

double Pin::get_slack()
{
    slack_cal();
    return slack;
}

void Plane_E::slack_propagation(Inst* cur)
{
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

vector<double> Pin::slack_propagation(Pin* modified_Pin){return slack_propagation(modified_Pin,1);}

vector<double> Pin::slack_propagation(Pin* modified_Pin,bool start) //it return the slack value difference (recursive)  0 for negative 1 for positive
{
    double cur_slack = slack;
    if(pin_type == belong_Inst->get_name() && type == 'O')   //output from other chips
    {
        arrival_time = 0;
        path_seq.clear();
        path_seq.push_back(this);
        vector<double> vals{0.0,0.0};
        if(this->belong_net==NULL)
            return vals;
        for(int i = 0 ; i < belong_net->TOs.size() ; i++)
        {
            vector<double> tmp = belong_net->TOs[i]->slack_propagation(modified_Pin,0);
            vals[0] += tmp[0];
            vals[1] += tmp[1];
        }
        return vals;
    }
    else if(pin_type[0] =='Q')  //FF output
    {
        double new_arrival = belong_Inst->corr_data->QpinDelay;
        arrival_time = new_arrival;
        if(new_arrival == arrival_time && pair_D(this->belong_Inst,this) != modified_Pin)
            return vector<double>{0.0,0.0};
        /*
        if(arrival_time == new_arrival_time)
            return vector<double>{0.0,0.0}  //half correct
        */
        path_seq.clear();
        path_seq.push_back(this);
        vector<double> vals{0.0,0.0};
        if(this->belong_net==NULL)
            return vals;
        for(int i = 0 ; i < belong_net->TOs.size() ; i++)
        {
            vector<double> tmp = belong_net->TOs[i]->slack_propagation(modified_Pin,0);
            vals[0] += tmp[0];
            vals[1] += tmp[1];
        }
        return vals;
    }
    else if(type == 'O')    //gate output
    {
        
        pair<double,Pin*> new_arrival = max_arrival_time(belong_Inst->INs);
        if(arrival_time == new_arrival.first)
            return vector<double>{0.0,0.0};

        
        arrival_time = new_arrival.first;
        path_seq = new_arrival.second->path_seq;
        path_seq.push_back(this);

        

        vector<double> vals{0.0,0.0};
        if(this->belong_net==NULL)
            return vals;
        for(int i = 0 ; i < belong_net->TOs.size() ; i++)
        {
            vector<double> tmp = belong_net->TOs[i]->slack_propagation(modified_Pin,0);
            vals[0] += tmp[0];
            vals[1] += tmp[1];
        }
        return vals;
    }
    else if(pin_type == belong_Inst->get_name() && type == 'I')
    {
        if(this->belong_net==NULL)
            return vector<double>{0.0,0.0};
        if(belong_net->FROMs.size()!=1)
            cout<<"??????????"<<endl;
        double new_arrival = HPWL(belong_net->FROMs[0],this) + belong_net->FROMs[0]->arrival_time;

        
        if(arrival_time == new_arrival)
            return vector<double>{0.0,0.0};
        path_seq = belong_net->FROMs[0]->path_seq;
        path_seq.push_back(this);
        arrival_time = new_arrival;
        return vector<double>{0.0,0.0};
    }
    else if(pin_type[0] == 'D')
    {
        if(this->belong_net==NULL)
            return vector<double>{0.0,0.0};
        if(belong_net->FROMs.size()!=1)
            cout<<"??????????"<<endl;
        double new_arrival = HPWL(belong_net->FROMs[0],this) + belong_net->FROMs[0]->arrival_time;
        if(new_arrival == arrival_time && !start)
            return vector<double>{0.0,0.0};
        path_seq = belong_net->FROMs[0]->path_seq;
        path_seq.push_back(this);
        arrival_time = new_arrival;
        slack = require_time - arrival_time;
        
        vector<double> vals = {0.0,0.0};
        if(slack >= 0.0 && cur_slack <= 0.0)
        {
            vals[0] = -cur_slack;
            vals[1] = slack;
        }
        if(slack <= 0.0 && cur_slack <= 0.0)
        {
            vals[0] = (slack - cur_slack);
            vals[1] = 0.0;
        }
        if(slack <= 0.0 && cur_slack >= 0.0)
        {
            vals[0] = slack;
            vals[1] = -cur_slack;
        }
        if(slack >= 0.0 && cur_slack >= 0.0)
        {
            vals[0] = 0.0;
            vals[1] = (slack - cur_slack);
        }
        vector<double> tmp = pair_Q(belong_Inst,this)->slack_propagation(modified_Pin,0);
        
        vals[0] += tmp[0];
        vals[1] += tmp[1];
        
        return vals; 
    }
    else if(type == 'I')    //gate input
    {
        if(recursive)
            vector<double>{0.0,0.0};
        if(this->belong_net==NULL)
            return vector<double>{0.0,0.0};
        if(belong_net->FROMs.size()!=1)
            cout<<"??????????"<<endl;
        double new_arrival = HPWL(belong_net->FROMs[0],this) + belong_net->FROMs[0]->arrival_time;
        if(arrival_time == new_arrival)
            return vector<double>{0,0};

        
        path_seq = belong_net->FROMs[0]->path_seq;
        path_seq.push_back(this);
        arrival_time = new_arrival;
        vector<double> vals{0.0,0.0};
        for(int i = 0 ; i < belong_Inst->OUTs.size() ; i++)
        {
            vector<double> tmp = belong_Inst->OUTs[i]->slack_propagation(modified_Pin,0);
            vals[0] += tmp[0];
            vals[1] += tmp[1];
        }
        return vals;

    }
    else    //clk
    {
        return vector<double>{0.0,0.0};
    }
}

void Pin::slack_trace_back()//it is not recursive function, it trace back to its relative source only
{
    for(int i = 0 ; i < belong_net->FROMs.size() ; i++)
    {
        double newVal = slack - HPWL(belong_net->FROMs[i],this);
        if(!belong_net->FROMs[i]->slack_determined)
            belong_net->FROMs[i]->slack = newVal;
        else if(belong_net->FROMs[i]->slack != newVal)
        {
            cout<<"TRACE BACK AND GET DIFFERENT VALUE: "<<belong_net->FROMs[i]->slack<<"   "<<newVal<<endl;
            cout<<" "<<belong_net->FROMs[i]->belong_Inst->get_name()<<"/"<<belong_net->FROMs[i]->pin_type<<endl;
        }
        else
            cout<<"TRACE BACK AND GET SAME VALUE: "<<belong_net->FROMs[i]->slack<<"   "<<newVal<<endl;
        belong_net->FROMs[i]->slack_determined = 1;
    }
}

void Pin::slack_cal()
{
    if(slack_determined)
        return;
    if(pin_type[0] =='Q')  //FF_type
    {   
        string find_type = pin_type;
        find_type[0] = 'D';
        Pin* corr_pin = NULL;
        for(int i = 0 ; i < belong_Inst->INs.size() ; i++)
            if(belong_Inst->INs[i]->pin_type == find_type)
            {
                corr_pin = belong_Inst->INs[i];
                break;
            }
        slack = corr_pin->get_slack() + belong_Inst->corr_data->QpinDelay;
    }
    else if(type =='O')  // gate/"IN" output
    {   
        Pin* corr_pin = NULL;
        double max_slack = -999999999999;
        for(int i = 0 ; i < belong_Inst->INs.size() ; i++)
            if(belong_Inst->INs[i]->get_slack() > max_slack)
                corr_pin = belong_Inst->INs[i];
        
        if(corr_pin == NULL)    //it is input of the chip
        {
            slack = 0;
            cout<<"UNDETERMINED SLACK VALUE FOR INPUT PIN"<<endl;
        }
        else
            slack = corr_pin->get_slack();
    }
    else  //FF_type
    {   
        Pin* corr_pin = NULL;
        double max_slack = -999999999999;
        for(int i = 0 ; i < belong_net->FROMs.size() ; i++)
        {
            double delay = belong_net->FROMs[i]->get_slack() + HPWL(belong_net->FROMs[i],this);
            if(delay > max_slack)
                max_slack = delay;
        }
        slack = max_slack;
    }
    slack_determined = 1;
}

Point Pin::abs_loc()
{
    
    return belong_Inst->LeftDown() + relative_loc;
}

double HPWL(Pin* a, Pin* b)
{
    if(a->belong_net!=b->belong_net)
    {
        cout<<"THEIR CORRISPONDED NET ARE DIFFERENT"<<endl;
        cout<<a->get_name()<<"  "<<b->get_name()<<endl;
        exit(0);
    }
    Point diff = a->abs_loc() - b->abs_loc();

    return a->belong_net->displacement_delay*(abs(diff.x) + abs(diff.y));
}

double HPWL(net* cur)
{
    int x1,y1,x2,y2;
    x1 = 1215752192;
    y1 = x1;
    x2 = -1215752192;
    y2 = x2;
    for(int i = 0 ; i < cur->relative.size() ; i++)
    {
        Point p = cur->relative[i]->abs_loc();
        if(p.x > x2)
            x2 = p.x;
        if(p.y > y2)
            y2 = p.y;
        if(p.x < x1)
            x1 = p.x;
        if(p.y < y1)
            y1 = p.y;
    }
    return (x2 - x1) + (y2 - y1);
}

double net::set_HPWL()
{
    int x1,y1,x2,y2;
    x1 = 1215752192;
    y1 = x1;
    x2 = -1215752192;
    y2 = x2;
    for(int i = 0 ; i < relative.size() ; i++)
    {
        Point p = relative[i]->abs_loc();
        if(p.x > x2)
            x2 = p.x;
        if(p.y > y2)
            y2 = p.y;
        if(p.x < x1)
            x1 = p.x;
        if(p.y < y1)
            y1 = p.y;
    }
    this->HPWL = (x2 - x1) + (y2 - y1);
    return this->HPWL;
};


double HPWL_without_corr_pins(net* cur,vector<Pin*> withouts)
{
    double x1,y1,x2,y2;
    x1 = 1215752192;
    y1 = x1;
    x2 = -1215752192;
    y2 = x2;

    vector<Pin*> to_cal = cur->relative;
    for(int i = 0 ; i < to_cal.size() ; i++)
    {
        for(int j = 0 ; j < withouts.size() ; j++)
        {
            if(withouts[j] == to_cal[i])
            {
                withouts[j] = withouts.back();
                withouts.pop_back();
                to_cal[i] = to_cal.back();
                to_cal.pop_back();
                j-=1;
            }
        }
    }

    for(int i = 0 ; i < to_cal.size() ; i++)
    {
        Point p = to_cal[i]->abs_loc();
        if(p.x > x2)
            x2 = p.x;
        if(p.y > y2)
            y2 = p.y;
        if(p.x < x1)
            x1 = p.x;
        if(p.y < y1)
            y1 = p.y;
    }
    return (x2 - x1) + (y2 - y1);
}

double HPWL_without_D_pins(net* cur)
{
    double x1,y1,x2,y2;
    x1 = 1215752192;
    y1 = x1;
    x2 = -1215752192;
    y2 = x2;

    vector<Pin*> to_cal = cur->relative;

    for(int i = 0 ; i < to_cal.size() ; i++)
    {
        if(to_cal[i]->pin_type[0]=='D')
            continue;
        Point p = to_cal[i]->abs_loc();
        if(p.x > x2)
            x2 = p.x;
        if(p.y > y2)
            y2 = p.y;
        if(p.x < x1)
            x1 = p.x;
        if(p.y < y1)
            y1 = p.y;
    }
    return max({0.0,(x2 - x1) + (y2 - y1)});
}

vector<Point> net::set_center()
{

    center_of_FROMs = Point(0,0);
    for(int i = 0 ; i < FROMs.size() ; i++)
    {
        center_of_FROMs = center_of_FROMs + FROMs[i]->abs_loc();
    }
    center_of_FROMs = center_of_FROMs/double(FROMs.size());
    center_of_TOs = Point(0,0);
    for(int i = 0 ; i < TOs.size() ; i++)
    {
        center_of_TOs = center_of_TOs + TOs[i]->abs_loc();
    }
    center_of_TOs = center_of_TOs/double(TOs.size());
    return vector<Point> {center_of_FROMs,center_of_TOs};
}

vector<Point> net::set_weight_center()
{
    
    center_of_FROMs = Point(0,0);
    double weight_F = 0;
    double Fx = 0;
    double Fy = 0;
    
    for(int i = 0 ; i < FROMs.size() ; i++)
    {
        Fx = Fx + FROMs[i]->abs_loc().x * FROMs[i]->critical_weight;
        Fy = Fy + FROMs[i]->abs_loc().y * FROMs[i]->critical_weight;
        weight_F += FROMs[i]->critical_weight;
    }

    center_of_FROMs.x = Fx/weight_F;
    center_of_FROMs.y = Fy/weight_F;
    FROMs_weight = weight_F;

    center_of_TOs = Point(0,0);
    double weight_FFrom = weight_F;
    weight_F = 0;
    Fx = 0;
    Fy = 0;
    for(int i = 0 ; i < TOs.size() ; i++)
    {
        if(TOs[i]->type == 'C')
            continue;
        Fx = Fx + TOs[i]->abs_loc().x * TOs[i]->critical_weight;
        Fy = Fy + TOs[i]->abs_loc().y * TOs[i]->critical_weight;
        weight_F += TOs[i]->critical_weight;
    }
    center_of_TOs.x = Fx/weight_F;
    center_of_TOs.y = Fy/weight_F;
    if(weight_FFrom <= 0 && weight_F <= 0)
    {
        set_center();
    }
    if(FROMs.size() == 0 || weight_FFrom <= 0)
    {
        center_of_FROMs = center_of_TOs;
    }
    if(TOs.size() == 0 || weight_F <= 0)
    {
        center_of_TOs = center_of_FROMs;
    }
    TOs_weight = weight_F;
    return vector<Point> {center_of_FROMs,center_of_TOs};
}