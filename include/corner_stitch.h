#ifndef _CORNER_STITCH_
#define _CORNER_STITCH_


#include<string>
#include<iostream>
#include<vector>
struct Tile;
struct Point;
class Module;

bool in_range(Tile*, Point);
bool y_in_range(Tile*, Point);
bool x_in_range(Tile*, Point);
bool adjcent(Tile*, Tile*);
bool areaCompare(Tile*, Tile*);
bool xCompare(const Point, const Point);
bool yCompare(const Point, const Point);

int height(Tile*);
int width(Tile*);
int area(Tile*);
int height(Module*);
int width(Module*);

std::ostream& operator<<(std::ostream&, Point);
std::ostream& operator<<(std::ostream&, Tile);

template <class T>
void randomize(std::vector<T>& toRand)
{
	for (int i = 0; i < toRand.size()-1; i++)
		std::swap(toRand[i],toRand[i + rand() % (toRand.size()-i)]);
}

struct Point
{
	int y ;
	int x ;
	Point(double y, double x) { this->y = y; this->x = x; }
	Point() { y = -1; x = -1; }
	Point operator+(Point p)const { return Point(y + p.y, x + p.x); }
	Point operator-(Point p)const { return Point(y - p.y, x - p.x); }
	Point operator/(double scalar)const { return Point(y / scalar, x / scalar); }
	Point operator*(double scalar)const { return Point(y * scalar, x * scalar); }
	void toInt() { x = int(x); y = int(y); }
	bool operator==(Point p);
};


struct Tile
{
		Point coord[2];	//left_down right_up
		Tile* stitch[4];
		Module* belong = NULL;
		Tile(Point,int,int);
		Tile(Point, Point);
		Tile(Tile*);
		Tile(Tile*, bool);
		Tile(Point, Point, Module*);
		Tile(Point, int, int, Module*);
		Tile(int, int, Point, Module*);
		Tile(int, int, Point);
		Tile();
		~Tile(); 
		bool operator==(Tile);
		bool operator>=(Tile);
		bool is_space() { return belong == NULL; }
		void rotate();
		void clearStitch();
};

struct Wire
{
	Module* a;
	Module* b;
	int weight;
	Wire(Module*, Module*, int);
	double HPWL();
};

Point& LD(Tile* a);
Point& RU(Tile* a);
Point LU(Tile* a);
Point RD(Tile* a);
Point Center(Tile* a);
int& U(Tile* a);
int& D(Tile* a);
int& L(Tile* a);
int& R(Tile* a);


Tile* getByMinWidth(std::vector<Tile*>);
Tile* getByMaxHeight(std::vector<Tile*>);
Tile* getByMinHeight(std::vector<Tile*>);
Tile* getByMaxWidth(std::vector<Tile*>);
#endif