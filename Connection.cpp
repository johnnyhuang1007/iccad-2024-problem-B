#include <cstdlib>
#include<iostream>
#include<string>
#include<fstream>
using namespace std;

int main(int argc, char**argv)
{
    string connectfile;
    connectfile = "sIVNCJNWAINI.txt";
    string command;
    command = "./sub/ProblemB ";
    command = command+argv[1] + " "+connectfile;
    system(command.c_str());

    command = "./sub/placer ";
    command = command+argv[1] +" "+connectfile+ " "+argv[2];
    system(command.c_str());
    system("rm ./sIVNCJNWAINI.txt");
    return 0;
} 