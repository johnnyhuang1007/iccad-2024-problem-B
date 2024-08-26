#include"Plane.h"

Point& LD(Tile* a) { return (a)->coord[0]; }
Point& RU(Tile* a) { return (a)->coord[1]; }
Point LU(Tile* a) { return Point((a)->coord[1].y, (a)->coord[0].x); }
Point RD(Tile* a) { return Point((a)->coord[0].y, (a)->coord[1].x); }
Point Center(Tile* a) { return Point(U(a) - D(a), R(a) - L(a)); }
int& U(Tile* a) { return (a)->coord[1].y; }
int& D(Tile* a) { return (a)->coord[0].y; }
int& L(Tile* a) { return (a)->coord[0].x; }
int& R(Tile* a) { return (a)->coord[1].x; }

Tile* getByMinWidth(std::vector<Tile*> Tlist)
{
	Tile* cur = Tlist[0];
	for (size_t i = 1; i < Tlist.size(); i++)
	{
		if (width(cur) > width(Tlist[i]))
			cur = Tlist[i];
	}
	return cur;
}

Tile* getByMaxWidth(std::vector<Tile*> Tlist)
{
	Tile* cur = Tlist[0];
	for (size_t i = 1; i < Tlist.size(); i++)
	{
		if (width(cur) < width(Tlist[i]))
			cur = Tlist[i];
	}
	return cur;
}

Tile* getByMaxHeight(std::vector<Tile*> Tlist)
{
	Tile* cur = Tlist[0];
	for (size_t i = 1; i < Tlist.size(); i++)
	{
		if (height(cur) < height(Tlist[i]))
			cur = Tlist[i];
	}
	return cur;
}

Tile* getByMinHeight(std::vector<Tile*> Tlist)
{
	Tile* cur = Tlist[0];
	for (size_t i = 1; i < Tlist.size(); i++)
	{
		if (height(cur) > height(Tlist[i]))
			cur = Tlist[i];
	}
	return cur;
}

std::ostream& operator<<(std::ostream& fout, Point p) 
{
	fout << p.x << " " << p.y; 
	return fout; 
}

std::ostream& operator<<(std::ostream& fout, Tile t)
{
	fout << t.coord[0] << " " << t.coord[1];
	return fout;
}

Tile::Tile(Tile* cp)
{
	this->coord[0] = cp->coord[0];
	this->coord[1] = cp->coord[1];
	for (size_t i = 0; i < 4; i++)
		this->stitch[i] = cp->stitch[i];
	this->belong = cp->belong;

}

Tile::Tile(Tile* cp,bool copyStitch)
{
	this->coord[0] = cp->coord[0];
	this->coord[1] = cp->coord[1];
	this->belong = cp->belong;
	if (copyStitch == 1)
	{
		for (size_t i = 0; i < 4; i++)
			this->stitch[i] = cp->stitch[i];
	}
	else
	{
		for (int i = 0; i < 4; i++)
			this->stitch[i] = NULL;
	}
}

Tile::Tile(Point coord, int height, int width)
{
	this->coord[0] = coord;
	this->coord[1] = coord + Point(height, width) - Point(1,1);
	for (int i = 0; i < 4; i++)
		this->stitch[i] = NULL;
}

Tile::Tile(Point coord, int height, int width, Module* belong)
{
	this->coord[0] = coord;
	this->coord[1] = coord + Point(height, width) - Point(1, 1);
	this->belong = belong;
	for (int i = 0; i < 4; i++)
		this->stitch[i] = NULL;
}

Tile::Tile(Point pLeftDown, Point pRightUp, Module* belong)
{
	this->coord[0] = pLeftDown;
	this->coord[1] = pRightUp;
	this->belong = belong;
	for (int i = 0; i < 4; i++)
		this->stitch[i] = NULL;
}

Tile::Tile(Point pLeftDown, Point pRightUp)
{
	this->coord[0] = pLeftDown;
	this->coord[1] = pRightUp;
	for (int i = 0; i < 4; i++)
		this->stitch[i] = NULL;
	this->belong = NULL;
}

Tile::Tile(int h, int w, Point pRU, Module* B)
{
	this->coord[1] = pRU;
	this->coord[0] = pRU - Point(h, w) + Point(1, 1);
	for (int i = 0; i < 4; i++)
		this->stitch[i] = NULL;
	this->belong = B;
}

Tile::Tile()
{
	this->coord[1] = Point(0, 0);
	this->coord[0] = Point(0, 0);
	for (int i = 0; i < 4; i++)
		this->stitch[i] = NULL;
	this->belong = NULL;
}

bool in_range(Tile* tile, Point coord)
{
	
	if (!x_in_range(tile, coord))
		return 0;
	if (!y_in_range(tile, coord))
		return 0;

	return 1;
}

bool x_in_range(Tile* tile, Point coord)
{
	if (LD(tile).x > coord.x)
		return 0;
	if (RU(tile).x < coord.x)
		return 0;
	return 1;
}

bool y_in_range(Tile* tile, Point coord)
{
	if (tile->coord[0].y > coord.y)
		return 0;
	if (tile->coord[1].y < coord.y)
		return 0;
	return 1;
}

bool Point::operator==(Point a)
{
	if (y != a.y)
		return 0;
	if (x != a.x)
		return 0;
	return 1;
}

bool Tile::operator==(Tile b)
{
	if (!(coord[0] == b.coord[0]))
		return 0;
	if (!(coord[1] == b.coord[1]))
		return 0;
	return 1;
}

bool Tile::operator>=(Tile b)
{
	if (!in_range(this, b.coord[0]))
		return 0;
	if (!in_range(this, b.coord[1]))
		return 0;
	return 1;
}

bool areaCompare(Tile* a, Tile* b)
{
	int Area1 = width(a) * height(b);
	int Area2 = width(b) * height(b);
	return Area1 < Area2;
}

bool xCompare(const Point a, const Point b) { return a.x < b.x; }
bool yCompare(const Point a, const Point b) { return a.y < b.y; } 

Tile::~Tile()
{}

void Tile::rotate()
{
	int tmpWidth = height(this);
	int tmpHeight = width(this);
	coord[1].x = coord[0].x + tmpHeight - 1;
	coord[1].y = coord[0].y + tmpWidth - 1;
	return;
}



void Tile::clearStitch()
{
	for (int i = 0; i < 4; i++)
		stitch[i] = NULL;
}

int height(Tile* a)
{
	return (a)->coord[1].y - (a)->coord[0].y + 1;
}

int width(Tile* a)
{
	return (a)->coord[1].x - (a)->coord[0].x + 1;
}

int area(Tile* a)
{
	return width(a) * height(a);
}

int width(Module* a)
{
	return RU(a->bound_Tile[2]).x - LD(a->bound_Tile[0]).x + 1;
}

int height(Module* a)
{
	return RU(a->bound_Tile[1]).y - LD(a->bound_Tile[3]).y + 1;
}

Wire::Wire(Module* a, Module* b, int weight)
{
	this->a = a;
	this->b = b;
	this->weight = weight;
}

double Wire::HPWL()
{
  if(a->getUp()==NULL || b->getUp()==NULL)
    exit(0);

	Point A = a->center();
	Point B = b->center();
	double x1 = std::max({ A.x,B.x });
	double x2 = std::min({ A.x,B.x });
	double y1 = std::max({ A.y,B.y });
	double y2 = std::min({ A.y,B.y }); 
	double lx = x1 - x2;
	double ly = y1 - y2; 
	double wl = lx + ly;
	return double(weight) * wl;
}
