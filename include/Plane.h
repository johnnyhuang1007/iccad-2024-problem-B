#ifndef _PLANE_H_
#define _PLANE_H_
#include"module.h"


#define origin Point(0,0)
#define f_m_set Fixed_Module_set

#define point_left_down(a) (a)->coord[0]
#define point_right_up(a) (a)->coord[1]


#define UD 1
#define RL 0

#define US(a) (a)->stitch[3]
#define LS(a) (a)->stitch[2]
#define DS(a) (a)->stitch[1]
#define RS(a) (a)->stitch[0]

#define RootOf(a) (a)->get_root()

#define Extand			0b000001
#define BothSide		0b000110
#define RLside			0b000100
#define UDside			0b000010
#define FixCenter		0b001000
#define One				0b010000
#define MinAreaLegal	0b100000


#include <vector>
#include <fstream>
#include <list>
#include<utility>
#include<algorithm>
#include<cmath>

 
class Plane
{
private:
	std::vector<Soft_Module*>	ordered_set;

public:
	Tile* boundary[4]; 
	int Height;
	int Width;
	std::vector<Fixed_Module*>	Fixed_Module_set;
	std::vector<Soft_Module*>	Soft_Module_set;
	std::vector<Wire*> Wire_set;
	//corner stitching core function
	void insert(Tile*);
	void h_split(Tile*, Point, int);
	void updateStitch(std::vector<Tile*>, Tile*);
	void v_split(Tile*, Tile*);

	bool merge(Tile*, Tile*);

	//utility
	std::vector<Tile*> neighborFinding(Tile*);
	static void findRightNeighbor(Tile*, std::vector<Tile*>&);
	static void findLeftNeighbor(Tile*, std::vector<Tile*>&);
	static void findDownNeighbor(Tile*, std::vector<Tile*>&);
	static void findUpNeighbor(Tile*, std::vector<Tile*>&);

	std::vector<Tile*> getAllSpace(void (*fp)(Tile*, std::vector<Tile*>&), Tile*);

	void remove(Tile*);
	void ripTillRoot(Soft_Module*);
	void ripTillRoot(int);

	bool checkAllSpace(Tile*);
	bool checkAllSpace(Tile*, Tile*);
	std::vector<Tile*> getSpaceTileInRegion(Tile*);
	std::vector<Tile*> getSolidTileInRegion(Tile*);
	bool checkAllSpace(int idx){return checkAllSpace(Fixed_Module_set[idx]->get_root());}
	void insertFix(Fixed_Module*);
	void minHPWLStretch(Soft_Module*);
	void minHPWLStretch(Soft_Module*,int);
	void shrinkRoot(Soft_Module*);
	void shrinkRoot(int);

	bool filled(Soft_Module* a) { return (a->getCurArea() - a->getMinArea()) >= 0; }
	double all_HPWL();

	Point generateSeed(Soft_Module*, Point&);
	Tile findMaxUsableRect(Point);
	bool patch(Soft_Module*, int, Point&);
	bool patchOne(Soft_Module* curSoft, int side, int Constraint);
	bool fillUp(Soft_Module*, int);
 

	Plane(std::string);
	Plane() {}
	Tile* point_finding(Point);			
	static Tile* point_finding(Point, Tile*);	
	std::vector<Tile*> findOccupiedSpace(Tile*);

	void insert_fix(size_t idx) { insertFix(Fixed_Module_set[idx]); }
	size_t fixedModule_size() { return Fixed_Module_set.size(); }
	size_t softModule_size() { return Soft_Module_set.size(); }
	bool is_boundary(Tile* a) { return a->belong == boundary[0]->belong; }

	std::vector<Point> getBound();
	int getWidth() { return Width; }
	int getHeight() { return Height; }
	std::vector<Tile*> getInserted();
	std::vector<Soft_Module*> getToInsert();
	std::vector<Wire*> getWires();
	Soft_Module* getSoft(int i) { return Soft_Module_set[i]; }
	std::vector<std::vector<Tile>> getAllTile();
	void obtainPreResult(std::vector<std::vector<Tile>> softSet);
	void obtainPreResult(std::vector<Tile> softSet);

	Point generateSeed(int,Point&);
	Point generateSeed(int);

	Tile decideInitTile();
	Tile generateRand(Soft_Module*);
	Tile generateRand(int);
	Tile moveCenter(int);
	Tile moveCenter(Soft_Module*);
	bool patch(int, int, Point&);
	void softPatch(int); 
	bool fillUp(int, int);

	
	bool ripoffALL();
	bool ripoff(int);
	bool ripoff(Soft_Module*); 
	int ripoffRecent(int);

	double getArea();
	double minLegalArea();
	
	int checkAllLegal();
	bool Legal(int);

	virtual void output(char*);
	virtual void outimg();
	void print();

	void operator=(Plane&);
	
	virtual ~Plane(); 
	void cleanUp();
};
#endif