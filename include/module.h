#ifndef _MODULE_H_
#define _MODULE_H_
#include"corner_stitch.h"

#include<iostream>
#include<list>
#include<queue>
#include<vector>

class Module;

Point LD(Module*);
Point RU(Module*);
Point RD(Module*);
Point LU(Module*);

class Module
{
protected:
	int min_area;
	std::string name;
	Tile* bound_Tile[4];
	std::list<Wire*> wireSet;
public:
	Module(std::string, int);
	virtual Point center();
	int getMinArea() { return min_area; }
	std::string getName() { return name; }
	void getWire(Wire* newWire) { wireSet.push_back(newWire); }
	Point optimPos();
	double PosPerformCheck(Point);
	void callWire();
	virtual Tile* get_root() { return bound_Tile[0]; }
	virtual bool is_fix() { return 1; }
	virtual void ripOff(Tile* a) {};
	virtual void update(Tile* a) {};
	virtual void update() {};
	Tile* getLeft() { return bound_Tile[0]; }
	Tile* getRight() { return bound_Tile[2]; }
	Tile* getUp() { return bound_Tile[1]; }
	Tile* getDown() { return bound_Tile[3]; }
	virtual ~Module() = default;
	friend int width(Module*);
	friend int height(Module*);

};

class Soft_Module :public Module
{
private:
	int cur_area = 0;

	void updateState();
public:
	std::vector<Tile*> component;
	Soft_Module(std::string, int);
	bool checkLegal();
	bool LegalifAdd(Tile*);
	bool is_fix() { return 0; }
	void obtainNewTile(Tile*);

	Point pseudoCenter(Tile*);
	Point HPWLdir();
	Point estHPWL(Point);

	void update();
	void update(Tile*);
	void updateRoot(Tile*);
	void ripTillRoot();
	void ripOff(Tile*);
	void ripOff();

	void get_Tile(Tile*);
	void print();
	Tile* get_root() { return *(component.begin()); }


	int getCurArea() { return cur_area; }
	int leftArea() { return min_area - cur_area; }
	void ripoffALL();
	std::vector<Tile*> getComp() { return component; }

	std::vector<Point> listPoint(Tile* (*point_finding)(Point, Tile*));
	~Soft_Module();
};

class Fixed_Module :public Module
{
private:
	Tile* root; 
public:
	Fixed_Module(std::string, Point, int, int);
	Tile* get_root() { return root; }
	bool set_root(Tile cur){
		if(root!=NULL) return 0;
		return root = new Tile(&cur,0);}
	bool set_root(Tile* cur){
		if(root!=NULL) return 0;
		return root = cur;}
	void set_root_to_NULL()
	{
		root = NULL;
	}
	std::string get_name() { return name; }
	void set_name(std::string in){name = in;}
	bool is_fix() { return 1; }
	virtual ~Fixed_Module();
	Point LeftDown(){return LD(root);}
	Point RightUp(){return RU(root) + Point(1,1);}
	int area(){ return (RightUp().x - LeftDown().x)*(RightUp().y - LeftDown().y); }
};

std::ostream& operator<<(std::ostream&, Fixed_Module);

#endif