#include<iostream>
#include<fstream>
#include<vector>
#include<string>
#include<list>
using namespace std;

int main(int argc, char** argv)
{
    ifstream module_list;
    ifstream org_in;
    ofstream fout;
    module_list.open(argv[1]);
    org_in.open(argv[2]);
    fout.open("Fix_mat.txt");
    
    int w,h;
    string dummy;
    for(int i = 0 ; i < 4 ; i++)    
       org_in>>dummy>>dummy;
    org_in>>dummy>>dummy>>dummy;
    org_in>>w>>h;
    org_in.close();
    fout<<"CHIP "<<w<<" "<<h<<endl;
    fout<<"SOFTMODULE 0"<<endl;
    list<string> modules;
    while(!module_list.eof())
    {
        string name, PX,PY,W,H;
        module_list>>dummy>>name>>PX>>PY>>dummy>>W>>H;
        string info;
        info = name+" "+PX+" "+PY+" "+W+" "+H;
        modules.push_back(info);
    } 
    if(modules.back().size()==0)
        modules.pop_back();
    fout<<"FIXEDMODULE "<<modules.size()<<endl;
    for(auto& info : modules)
        fout<<info<<endl;
    
    fout<<"CONNECTION 0"<<endl;

    return 0;
} 