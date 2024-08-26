#include "Plane.h"
#include <set>
using namespace std;


void Plane::ripTillRoot(int cur)
{
	ripTillRoot(Soft_Module_set[cur]);
}

void Plane::ripTillRoot(Soft_Module* curSoft)
{
	vector<Tile*> comp = curSoft->getComp();
	for (int i = 1; i < comp.size(); i++)
		remove(comp[i]);
	
	curSoft->ripTillRoot();
}

void Plane::remove(Tile* toRemove)
{

	//�ಾ��space set
	toRemove->belong = NULL;
	vector<Tile*> left;
	//	left.reserve(height(toRemove));
	findLeftNeighbor(toRemove, left);
	
	vector<Tile*> right;
	//	right.reserve(height(toRemove));
	findRightNeighbor(toRemove, right);
	if(left.size()==0 && right.size()==0)
		return;
	Point Upper = RU(toRemove);
	Point Lower = LD(toRemove);
	for (int i = 0; i < left.size(); i++)
	{	
		if (RU(left[i]).y <= Upper.y)
		{
			if (RS(left[i])->is_space())
				h_split(RS(left[i]), RU(left[i]) + Point(0, 1), 0);
			if (RS(RS(left[i]))->is_space())
				h_split(RS(RS(left[i])), RU(RS(left[i])) + Point(0, 1), 0);
		}
	}

	for (int i = 0; i < right.size(); i++)
	{
		if (LD(right[i]).y >= Lower.y)
		{
			if (LS(right[i])->is_space())
				h_split(LS(right[i]), LD(right[i]) + Point(0, -1), 1);
			if (LS(LS(right[i]))->is_space())
				h_split(LS(LS(right[i])), LD(LS(right[i])) + Point(0, -1), 1);
		}
	}

	Tile* top = point_finding(Upper, left[0]);
	while(LD(top).y >= Lower.y)
	{
		Tile* tmp = DS(top);
		merge(top, RS(top));
		merge(top, LS(top));
		if (LS(top) == LS(US(top)) && RS(top) == RS(US(top)))
			merge(top, US(top));

		top = tmp;
	}
	if (LS(top) == LS(US(top)) && RS(top) == RS(US(top)))
		merge(top, US(top));
}

bool Plane::merge(Tile* Main, Tile* Sub)
{
	if (Sub->belong!= Main->belong || !Main->is_space())
		return 0;

	int spaceMergeType = -1;
	vector<Tile*> neighbors;
	if (LD(Main).y == LD(Sub).y && RU(Main).y == RU(Sub).y)
	{
		if (RU(Main).x < LD(Sub).x)
		{
			RS(Main) = RS(Sub);
			US(Main) = US(Sub);
			RU(Main) = RU(Sub);
			spaceMergeType = 0;
		}
		else
		{
			LS(Main) = LS(Sub);
			DS(Main) = DS(Sub);
			LD(Main) = LD(Sub);
			spaceMergeType = 1;
		}
	}
	else if (LD(Main).x == LD(Sub).x && RU(Main).x == RU(Sub).x)
	{
		if (Main->is_space())
		{
			if (LS(Main) != LS(Sub) || RS(Main) != RS(Sub))
				return 0;
		}
		if (RU(Main).y +1 == LD(Sub).y)
		{
			RS(Main) = RS(Sub);
			US(Main) = US(Sub);
			RU(Main) = RU(Sub);
			spaceMergeType = 2;
		}
		else if(LD(Main).y - 1 == RU(Sub).y)
		{
			LS(Main) = LS(Sub);
			DS(Main) = DS(Sub);
			LD(Main) = LD(Sub);
			spaceMergeType = 3;
		}
	}
	else
		return 0;

	neighbors = neighborFinding(Main);
	updateStitch(neighbors, Main);
	delete Sub;
	Sub = NULL;
	if (Main->belong == NULL)
		return 1;
	else
	{
		Main->belong->update();
		if (spaceMergeType == 2 && RS(Main)->is_space())
			merge(RS(Main), DS(RS(Main)));
		if (spaceMergeType == 2 && LS(Main)->is_space())
			merge(LS(Main), US(LS(Main)));
		if (spaceMergeType == 3 && LS(Main)->is_space())
			merge(LS(Main), US(LS(Main)));
		if (spaceMergeType == 3 && RS(Main)->is_space())
			merge(RS(Main), DS(RS(Main)));
		return 1;
	}
}

vector<Tile*> Plane::getAllSpace(void (*findNeighbor)(Tile*, std::vector<Tile*>&), Tile* cur)
{
	vector<Tile*> neighbor;
	neighbor.clear();
	neighbor.reserve(max({ width(cur),height(cur) })/10);
	findNeighbor(cur, neighbor);
	vector<Tile*> space;
	space.clear();
	space.reserve(neighbor.size());
	for (int i = 0; i < neighbor.size(); i++)
	{
		if (neighbor[i]->belong==NULL)
			space.push_back(neighbor[i]);
	}
	neighbor.clear();
	return space;
}



bool Plane::checkAllSpace(Tile* inTile,Tile* start)
{
	if (LD(inTile).x < 0 || LD(inTile).y < 0)
		return 0;
	if (RU(inTile).x >= Width || RU(inTile).y >= Height)
		return 0;
	Tile* curSpace = point_finding(LD(inTile),start);
	
	bool fail = 0;
	while (1)
	{
		
		if (!curSpace->is_space())
			return 0;
		if (RU(curSpace).x < RU(inTile).x)
			return 0;
		if (RU(curSpace).y >= RU(inTile).y)
			return 1;
		Point tmp(RU(curSpace).y, LD(inTile).x);
		curSpace = point_finding(tmp + Point(1, 0), curSpace);
	}

	return !fail;
}

vector<Tile*> Plane::getSpaceTileInRegion(Tile* inTile,Tile* start)
{
	set<Tile*> return_list;
	Tile* curSpace = point_finding(LD(inTile),start);
	
	int current_height = RU(inTile).y;
	int current_lower = LD(inTile).y;
	//cout<<"SEARCHING SPACE"<<endl;
	while (1)
	{
		//cout<<*curSpace<<endl;
		if (curSpace->is_space())
			return_list.insert(curSpace);
		if(current_height > RU(curSpace).y)
			current_height = RU(curSpace).y;
		
		Point tmp; 
		if(RU(curSpace).x < RU(inTile).x)
		{
			tmp.x = RU(curSpace).x + 1;
			tmp.y = current_lower;
		}
		else if(RU(curSpace).y < RU(inTile).y)	//this mean this row has done searching
		{
			tmp.y = ++current_height;
			current_lower = current_height;
			tmp.x = LD(inTile).x;
		}
		else
		{
			break;
		}
		curSpace = point_finding(tmp, curSpace);
	}
	//cout<<"END OF SEARCH"<<endl;
	vector<Tile*> res{return_list.begin(),return_list.end()};
	return res;
}

vector<Tile*> Plane::getSpaceTileInRegion(Tile* inTile)
{
	set<Tile*> return_list;
	Tile* curSpace = point_finding(LD(inTile));
	
	int current_height = RU(inTile).y;
	int current_lower = LD(inTile).y;
	//cout<<"SEARCHING SPACE"<<endl;
	while (1)
	{
		//cout<<*curSpace<<endl;
		if (curSpace->is_space())
			return_list.insert(curSpace);
		if(current_height > RU(curSpace).y)
			current_height = RU(curSpace).y;
		
		Point tmp; 
		if(RU(curSpace).x < RU(inTile).x)
		{
			tmp.x = RU(curSpace).x + 1;
			tmp.y = current_lower;
		}
		else if(RU(curSpace).y < RU(inTile).y)	//this mean this row has done searching
		{
			tmp.y = ++current_height;
			current_lower = current_height;
			tmp.x = LD(inTile).x;
		}
		else
		{
			break;
		}
		curSpace = point_finding(tmp, curSpace);
	}
	//cout<<"END OF SEARCH"<<endl;
	vector<Tile*> res{return_list.begin(),return_list.end()};
	return res;
}

std::vector<Tile*> Plane::getSolidTileInRegion(Tile* inTile,Tile* start)
{
	set<Tile*> return_list;
	Tile* curSpace = point_finding(LD(inTile),start);
	
	int current_height = RU(inTile).y;
	int current_lower = LD(inTile).y;
	//cout<<"SEARCHING SOLID"<<endl;
	while (1)
	{
		//cout<<*curSpace<<endl;
		if (!curSpace->is_space())
			return_list.insert(curSpace);
		if(current_height > RU(curSpace).y)
			current_height = RU(curSpace).y;
		
		Point tmp;
		if(RU(curSpace).x < RU(inTile).x)
		{
			tmp.x = RU(curSpace).x + 1;
			tmp.y = current_lower;
		}
		else if(RU(curSpace).y < RU(inTile).y)	//this mean this row has done searching
		{
			tmp.y = ++current_height;
			current_lower = current_height;
			tmp.x = LD(inTile).x;
		}
		else
		{
			break;
		}
		curSpace = point_finding(tmp, curSpace);
	}

	//cout<<"END OF SEARCH"<<endl;
	vector<Tile*> res{return_list.begin(),return_list.end()};
	return res;
}

std::vector<Tile*> Plane::getSolidTileInRegion(Tile* inTile)
{
	set<Tile*> return_list;
	Tile* curSpace = point_finding(LD(inTile));
	
	int current_height = RU(inTile).y;
	int current_lower = LD(inTile).y;
	//cout<<"SEARCHING SOLID"<<endl;
	while (1)
	{
		//cout<<*curSpace<<endl;
		if (!curSpace->is_space())
			return_list.insert(curSpace);
		if(current_height > RU(curSpace).y)
			current_height = RU(curSpace).y;
		
		Point tmp;
		if(RU(curSpace).x < RU(inTile).x)
		{
			tmp.x = RU(curSpace).x + 1;
			tmp.y = current_lower;
		}
		else if(RU(curSpace).y < RU(inTile).y)	//this mean this row has done searching
		{
			tmp.y = ++current_height;
			current_lower = current_height;
			tmp.x = LD(inTile).x;
		}
		else
		{
			break;
		}
		curSpace = point_finding(tmp, curSpace);
	}

	//cout<<"END OF SEARCH"<<endl;
	vector<Tile*> res{return_list.begin(),return_list.end()};
	return res;
}

bool Plane::checkAllSpace(Tile* inTile)
{
	if (LD(inTile).x < 0 || LD(inTile).y < 0)
		return 0;
	if (RU(inTile).x >= Width || RU(inTile).y >= Height)
		return 0;
	Tile* curSpace = point_finding(LD(inTile));
	
	bool fail = 0;
	while (1)
	{
		
		if (!curSpace->is_space())
			return 0;
		if (RU(curSpace).x < RU(inTile).x)
			return 0;
		if (RU(curSpace).y >= RU(inTile).y)
			return 1;
		Point tmp(RU(curSpace).y, LD(inTile).x);
		curSpace = point_finding(tmp + Point(1, 0), curSpace);
	}

	return !fail;
}

vector<Point> Plane::getBound() 
{
	vector<Point> pset;
	for (int i = 0; i < Fixed_Module_set.size(); i++)
	{
		Tile* curT = Fixed_Module_set[i]->get_root();
		int presize = pset.size();
		pset.reserve(presize + 2 * (width(curT) + height(curT) - 4));
		for (int j = LD(curT).x; j <= RU(curT).x; j++)
		{
			pset.push_back(Point(LD(curT).y, j));
			pset.push_back(Point(RU(curT).y, j));
		}
		for (int j = LD(curT).y +1; j < RU(curT).y; j++)
		{
			pset.push_back(Point(j, LD(curT).x));
			pset.push_back(Point(j, RU(curT).x));
		}
	}
	return pset;
}

double Plane::getArea()
{
	int cur = 0;
	for (int i = 0; i < Soft_Module_set.size(); i++)
		cur += Soft_Module_set[i]->getCurArea();
	return cur;
}

vector<Tile*> Plane::getInserted()
{
	vector<Tile*> insertedList;
	insertedList.reserve(Fixed_Module_set.size());
	for (int i = 0; i < Fixed_Module_set.size(); i++)
		insertedList.emplace_back(Fixed_Module_set[i]->get_root());
	for (int i = 0; i < Soft_Module_set.size(); i++)
	{
		if (Soft_Module_set[i]->getCurArea() == 0)
			continue;
		vector<Tile*> comp = Soft_Module_set[i]->getComp();
		for (int j = 0; j < comp.size(); j++)
			insertedList.emplace_back(comp[j]);
	}
	return insertedList;
}


vector<Soft_Module*> Plane::getToInsert()
{
	vector<Soft_Module*> toInsertList;
	toInsertList.reserve(Soft_Module_set.size());
	for (int i = 0; i < Soft_Module_set.size(); i++)
	{
		if (Soft_Module_set[i]->getCurArea() != 0)
			continue;
		toInsertList.emplace_back(Soft_Module_set[i]);
	}
	return toInsertList;
}

vector<Wire*> Plane::getWires()
{
	return Wire_set;
}

double Plane::minLegalArea()
{
	int cur = 0;
	for (int i = 0; i < Soft_Module_set.size(); i++)
		cur += Soft_Module_set[i]->getMinArea();
	return cur;
}

int Plane::ripoffRecent(int idx)
{
	Soft_Module* curSoft = Soft_Module_set[idx];
	vector<Tile*> sub = curSoft->getComp();
	vector<Tile*> neighbor;
	neighbor.resize(0);
	for (int i = 0; i < sub.size(); i++)
	{
		vector<Tile*> subNeighbor = neighborFinding(sub[i]);
		neighbor.insert(neighbor.end(), subNeighbor.begin(), subNeighbor.end());
	}
	for (int i = 0; i < neighbor.size(); i++)
	{
		if (neighbor[i]->is_space() || is_boundary(neighbor[i]))
		{
			neighbor[i] = neighbor.back();
			neighbor.pop_back();
			i--;
			continue;
		}
		if (neighbor[i]->belong->is_fix() || neighbor[i]->belong == curSoft)
		{
			neighbor[i] = neighbor.back();
			neighbor.pop_back();
			i--;
			continue;
		}
	}
	if (neighbor.size() == 0)
		return -1;
	Soft_Module* minSoft = dynamic_cast<Soft_Module*>(neighbor[0]->belong);
	int toReturn = 0;
	for (int i = 0; i < neighbor.size(); i++)
	{
		if (minSoft->getCurArea() > dynamic_cast<Soft_Module*>(neighbor[i]->belong)->getCurArea())
			minSoft = dynamic_cast<Soft_Module*>(neighbor[i]->belong);
	}
	for (int i = 0; i < Soft_Module_set.size(); i++)
		if (minSoft == Soft_Module_set[i])
			toReturn = i;
	ripoff(toReturn);
	return toReturn;
}

bool Plane::ripoffALL()
{
	for (int i = Soft_Module_set.size()-1; i >=0 ; i--)
		ripoff(i);
	return 1;
}

bool Plane::ripoff(Soft_Module* curSoft)
{
	vector<Tile*> Comp = curSoft->getComp();
	for (int j = 0; j < Comp.size(); j++)
		remove(Comp[j]);
	curSoft->ripoffALL();
	return 1; 
}

bool Plane::ripoff(int idx) 
{
	return ripoff(Soft_Module_set[idx]);
} 