#include "Plane.h"
using namespace std;

Point LD(Module* a) { return Point(LD(a->getDown()).y, LD(a->getLeft()).x); }
Point RU(Module* a) { return Point(RU(a->getUp()).y, RU(a->getRight()).x); }
Point RD(Module* a) { return Point(LD(a->getDown()).y, RU(a->getRight()).x); }
Point LU(Module* a) { return Point(RU(a->getUp()).y, LD(a->getLeft()).x); }

Fixed_Module::Fixed_Module(string name, Point coord, int height, int width) :Module(name, height* width)
{
	root = new Tile(coord, height, width, this);
	for (int i = 0; i < 4; i++)
		bound_Tile[i] = root;
}

Module::Module(string name, int area)
{
	this->name = name;
	min_area = area;
	for (int i = 0; i < 4; i++)
		bound_Tile[i] = NULL;
}

void Module::callWire()
{
	for (auto wire : this->wireSet)
		cout << wire->a->getName() << "	" << wire->b->getName() << "	" << wire->weight << endl;
}

struct PW
{
	Point Cen;
	double Weight;
};
bool xComp(PW a, PW b) { return a.Cen.x < b.Cen.x; }
bool yComp(PW a, PW b) { return a.Cen.y < b.Cen.y; }

Point Module::optimPos()
{
	vector<PW> otherModules;
	otherModules.clear();
	otherModules.reserve(wireSet.size());
	int s = 0;
	int weightCount = 0;
	for (auto wire : this->wireSet)
	{
		s++;
		Module* others;
		if (wire->a == this)
			others = wire->b;
		else
			others = wire->a;
		if (others->getUp() == NULL)
			continue;
		PW tmp;
		tmp.Cen = others->center();
		tmp.Weight = wire->weight;
		otherModules.push_back(tmp);
		weightCount += wire->weight;
	}

	int halfcount = 0;
	Point opt;
	sort(otherModules.begin(), otherModules.end(), xComp);
	for (int i = 0; i < otherModules.size(); i++)
	{
		halfcount += otherModules[i].Weight;
		if (halfcount * 2 >= weightCount)
		{
			opt.x = otherModules[i].Cen.x;
			break;
		}
	}
	halfcount = 0;
	sort(otherModules.begin(), otherModules.end(), yComp);
	for (int i = 0; i < otherModules.size(); i++)
	{
		halfcount += otherModules[i].Weight;
		if (halfcount * 2 >= weightCount)
		{
			opt.y = otherModules[i].Cen.y;
			break;
		}
	}
	return opt;
}

Point Soft_Module::HPWLdir()
{
	Point dir = Point(0, 0);
	for (auto& wire : wireSet)
	{
		Point A = wire->a->center();
		Point B = wire->b->center();
		double x1 = std::max({ A.x,B.x });
		double x2 = std::min({ A.x,B.x });
		double y1 = std::max({ A.y,B.y });
		double y2 = std::min({ A.y,B.y });
		double lx = x1 - x2;
		double ly = y1 - y2;
		dir.y += ly * double(wire->weight);
		dir.x += lx * double(wire->weight);
	}
	return dir;
}
Point Soft_Module::estHPWL(Point opt)
{
	Point dir = Point(0, 0);
	for (auto& wire : wireSet)
	{
		Point A = wire->a->center();
		Point B = wire->b->center();
		if (wire->a == this)
			A = opt;
		else
			B = opt;
		double x1 = std::max({ A.x,B.x });
		double x2 = std::min({ A.x,B.x });
		double y1 = std::max({ A.y,B.y });
		double y2 = std::min({ A.y,B.y });
		double lx = x1 - x2;
		double ly = y1 - y2;
		dir.y += ly * double(wire->weight);
		dir.x += lx * double(wire->weight);
	}
	return dir;
}

Point Module::center()		//����A�p�G�Ogrid base�Acenter�n�A -(0.5,0.5)
{
	return Point((RU(bound_Tile[1]).y + LD(bound_Tile[3]).y +1) / 2, (RU(bound_Tile[2]).x + LD(bound_Tile[0]).x +1) / 2);
}
 
Point Soft_Module::pseudoCenter(Tile* toInst)
{
	Point psuLD, psuRU;
	psuLD = LD(this);
	psuRU = RU(this);
	if (RU(toInst).x > psuRU.x)
		psuRU.x = RU(toInst).x;
	if (RU(toInst).y > psuRU.y)
		psuRU.y = RU(toInst).y;
	if (LD(toInst).x < psuLD.x)
		psuLD.x = LD(toInst).x;
	if (LD(toInst).y < psuLD.y)
		psuLD.y = LD(toInst).y;
	return Point((psuRU.y + psuLD.y) / 2 + 0.5, (psuRU.x + psuLD.x) / 2 + 0.5);
}

Soft_Module::Soft_Module(string name, int area) :Module(name, area)
{
	component.resize(0);
}

void Soft_Module::updateRoot(Tile* newRoot)
{
	component[0] = newRoot;
	update();
}

void Soft_Module::update()
{
	cur_area = 0;
	for (int i = 0; i < 4; i++)
		bound_Tile[i] = NULL;
	for (auto& comp : component)
	{
		if (bound_Tile[0] == NULL || LD(comp).x <= LD(bound_Tile[0]).x)
			bound_Tile[0] = comp;
		if (bound_Tile[1] == NULL || RU(comp).y >= RU(bound_Tile[1]).y)
			bound_Tile[1] = comp;
		if (bound_Tile[2] == NULL || RU(comp).x >= RU(bound_Tile[2]).x)
			bound_Tile[2] = comp;
		if (bound_Tile[3] == NULL || LD(comp).y <= LD(bound_Tile[3]).y)
			bound_Tile[3] = comp;
		cur_area += area(comp);
	}

}

bool Soft_Module::LegalifAdd(Tile* toInst)
{
	if (component.size() == 0)
		return 0;
	Point newLD = LD(this);
	Point newRU = RU(this);
	if (newLD.y > LD(toInst).y)
		newLD.y = LD(toInst).y;
	if (newLD.x > LD(toInst).x)
		newLD.x = LD(toInst).x;
	if (newRU.y < RU(toInst).y)
		newRU.y = RU(toInst).y;
	if (newRU.x < RU(toInst).x)
		newRU.x = RU(toInst).x;


	double cover = (double)(newRU.x - newLD.x + 1) * (newRU.y - newLD.y + 1);
	if (cur_area + area(toInst) < min_area)
		return 0;
	if (ceil(cover) > ((cur_area + area(toInst)) * 5) / 4)
		return 0;
	
	if ((newRU.x - newLD.x + 1) * 2 < (newRU.y - newLD.y + 1))
		return 0;
	if ((newRU.y - newLD.y + 1) * 2 < (newRU.x - newLD.x + 1))
		return 0;
		
	return 1;
}

bool Soft_Module::checkLegal()
{
	if (component.size() == 0)
		return 0;
	double cover = (double)(width(this)) * height(this);
	if (cur_area < min_area)
		return 0;
	if (ceil(cover) > (cur_area * 5) / 4)
		return 0;
	if (width(this) * 2 < height(this))
		return 0;
	if (height(this) * 2 < width(this))
		return 0;
	return 1;
}

void Soft_Module::update(Tile* newTile)
{
	if (bound_Tile[0] == NULL || LD(newTile).x <= LD(bound_Tile[0]).x)
		bound_Tile[0] = newTile;
	if (bound_Tile[1] == NULL || RU(newTile).y >= RU(bound_Tile[1]).y)
		bound_Tile[1] = newTile;
	if (bound_Tile[2] == NULL || RU(newTile).x >= RU(bound_Tile[2]).x)
		bound_Tile[2] = newTile;
	if (bound_Tile[3] == NULL || LD(newTile).y <= LD(bound_Tile[3]).y)
		bound_Tile[3] = newTile;
	component.push_back(newTile);
	cur_area += area(newTile);
}

void Soft_Module::ripOff(Tile* toRemove)
{
	if (toRemove->belong != this)
		return;
	for (int i = 0; i < component.size(); i++)
	{
		if (component[i] == toRemove)
		{
			component.erase(component.begin() + i, component.begin() + i + 1);
			break;
		}
	}
	update();
}

void Soft_Module::ripTillRoot()
{
	component.resize(1);
	update();
}

void Soft_Module::ripoffALL()
{
	component.clear();
	component.resize(0);
	for (int i = 0; i < 4; i++)
		bound_Tile[i] = NULL;
	cur_area = 0;
}

void Soft_Module::get_Tile(Tile* newTile) { component.push_back(newTile); }

void Soft_Module::print()
{
	cout << "MIN_AREA: ";
	cout << min_area << endl;
	cout << "CUR_AREA: ";
	cout << cur_area << endl;
	cout << "Width and Height: ";
	if (bound_Tile[0] != NULL)
		cout << width(this) << "	" << height(this);
	cout << endl;
	for (auto& t : component)
		cout << *t << "  " << (t->belong == this) << endl;
}

struct Line
{
	Point start;
	Point end;
	Line(Point A, Point B) { start = A; end = B; }
	Line() {}
};

bool between(Line l, Point p)
{
	Point a, b;
	if (l.start == l.end)
		return 0;
	if (l.start.x == l.end.x && p.x == l.end.x)	//vertical
	{
		if (l.start.y < l.end.y)	//low to high
		{
			a = l.start;
			b = l.end;
		}
		else	//high to low
		{
			a = l.end;
			b = l.start;
		}
		if ((a.y <= p.y) && (p.y <= b.y))
			return 1;
		else
			return 0;
	}
	else if (l.start.y == l.end.y && p.y == l.end.y)
	{
		if (l.start.x < l.end.x)
		{
			a = l.start;
			b = l.end;
		}
		else
		{
			a = l.end;
			b = l.start;
		}
		if ((a.x <= p.x) && (p.x <= b.x))
			return 1;
		else
			return 0;
	}
	return 0;
}

void updateLine(vector<Line>& Lset)
{
	for (int i = 0; i < Lset.size(); i++)
	{
		for (int j = 0; j < Lset.size(); j++)
		{
			if (Lset[i].end == Lset[j].start)
			{
				Lset[i].end = Lset[j].end;
				Lset[j] = Lset.back();
				Lset.pop_back();
				i = -1;
				break;
			}
		}
	}

	for (int i = 0; i < Lset.size(); i++)
	{
		if (Lset[i].start == Lset[i].end)
		{
			Lset[i] = Lset.back();
			i--;
			Lset.pop_back();
		}
	}

	for (int i = 0; i < Lset.size(); i++)
	{

		for (int j = 0; j < Lset.size(); j++)
		{
			if (i == j)
				continue;
			if (between(Lset[i], Lset[j].end) && between(Lset[i], Lset[j].start))	//j is shorter
			{
				swap(Lset[i].end, Lset[j].end);
				if (Lset[i].start == Lset[i].end)
				{
					Lset[i] = Lset.back();
					Lset.pop_back();
				}
				if (Lset[j].start == Lset[j].end)
				{
					Lset[j] = Lset.back();
					Lset.pop_back();
				}
				i = -1;
				break;
			}
			else if (between(Lset[i], Lset[j].end))
			{
				swap(Lset[i].end, Lset[j].end);
				if (Lset[i].start == Lset[i].end)
				{
					Lset[i] = Lset.back();
					Lset.pop_back();
				}
				if (Lset[j].start == Lset[j].end)
				{
					Lset[j] = Lset.back();
					Lset.pop_back();
				}
				i = -1;
				break;
			}
			else if (between(Lset[i], Lset[j].start))
			{
				swap(Lset[i].start, Lset[j].start);
				if (Lset[i].start == Lset[i].end)
				{
					Lset[i] = Lset.back();
					Lset.pop_back();
				}
				if (Lset[j].start == Lset[j].end)
				{
					Lset[j] = Lset.back();
					Lset.pop_back();
				}
				i = -1;
				break;
			}
		}
	}

	for (int i = 0; i < Lset.size(); i++)
	{
		if (Lset[i].start == Lset[i].end)
		{
			Lset[i] = Lset.back();
			i--;
			Lset.pop_back();
		}
	}

}

vector<Point> Soft_Module::listPoint(Tile* (*point_finding)(Point p, Tile* t))
{

	vector<Line> LsetX, LsetY;
	for (int i = 0; i < component.size(); i++)
	{
		LsetY.push_back(Line(LD(component[i]), LU(component[i]) + Point(1, 0)));
		LsetY.push_back(Line(RU(component[i]) + Point(1, 1), RD(component[i]) + Point(0, 1)));
		LsetX.push_back(Line(LU(component[i]) + Point(1, 0), RU(component[i]) + Point(1, 1)));
		LsetX.push_back(Line(RD(component[i]) + Point(0, 1), LD(component[i])));
	}

	updateLine(LsetX);
	updateLine(LsetY);



	vector<Point> solP;
	solP.resize(1, LsetY[0].start);
	for (int i = 0; i < LsetY.size(); i++)
	{
		if (solP[0].x > LsetY[i].start.x)
			solP[0] = LsetY[i].start;
		else if ((solP[0].y > LsetY[i].start.y) && (solP[0].x == LsetY[i].start.x))
			solP[0] = LsetY[i].start;
	}

	bool dir = 1;
	while (1)
	{
		solP.resize(solP.size() + 1);
		if (dir == 0)
		{
			Point cur = solP[solP.size() - 2];
			for (int i = 0; i < LsetX.size(); i++)
			{
				if (cur == LsetX[i].start)
				{
					cur = LsetX[i].end;
					LsetX[i] = LsetX.back();
					LsetX.pop_back();
					i = -1;
				}
			}
			solP[solP.size() - 1] = cur;
			dir = 1;
		}
		else
		{
			Point cur = solP[solP.size() - 2];
			for (int i = 0; i < LsetY.size(); i++)
			{
				if (cur == LsetY[i].start)
				{
					cur = LsetY[i].end;
					LsetY[i] = LsetY.back();
					LsetY.pop_back();
					i = -1;
				}
			}
			solP[solP.size() - 1] = cur;
			dir = 0;
		}
		if (solP.back() == solP[0])
			break;
	}

	solP.pop_back();
	return solP;
}

Soft_Module::~Soft_Module()
{}

Fixed_Module::~Fixed_Module()
{
}