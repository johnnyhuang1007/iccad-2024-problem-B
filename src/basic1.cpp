#include"Plane.h"

/*
* stitch ����
				 3
	--------------
	|			 |  0   
	|			 |  
	|			 |
	|			 |
  2 |			 |
	--------------
	1 


* boundary ����
		   1
	--------------
	|			 |
	|			 |
  0 |			 | 2
	|			 |
	|			 |
	--------------
		  3
*/

using namespace std;

bool moduleAreaCompare(Soft_Module* a, Soft_Module* b) { return b->getMinArea() > a->getMinArea(); }

Plane::Plane(string file)
{
	ifstream fin;
	fin.open(file);
	string type;
	fin >> type; 
	fin >> Width;
	fin >> Height;
	
	Module* dummyModule = new Module("boundary", 0);
	boundary[0] = new Tile(Point(0, -1), Point(Height, -1), dummyModule);
	boundary[1] = new Tile(Point(Height, 0), Point(Height, Width), dummyModule);
	boundary[2] = new Tile(Point(-1, Width), Point(Height - 1, Width), dummyModule);
	boundary[3] = new Tile(Point(-1, -1), Point(-1, Width - 1), dummyModule);

	Tile* Space = new Tile(Point(0, 0), Height, Width);
	boundary[0]->stitch[0] = boundary[1];
	boundary[0]->stitch[1] = boundary[3];
	boundary[1]->stitch[1] = Space;
	boundary[1]->stitch[2] = boundary[0];
	boundary[2]->stitch[2] = boundary[3];
	boundary[2]->stitch[3] = boundary[1];
	boundary[3]->stitch[0] = boundary[2];
	boundary[3]->stitch[3] = Space;



	for (size_t i = 0; i < 4; i++)
		Space->stitch[i] = boundary[(i + 2) % 4];
	int size;

	//get soft
	fin >> type;
	fin >> size;
	Soft_Module_set.resize(size);
	ordered_set.resize(size);
	for (int i = 0; i < size; i++)
	{
		string name;
		int A;
		fin >> name >> A;
		Soft_Module_set[i] = new Soft_Module(name, A);
		ordered_set[i] = Soft_Module_set[i];
	}

	//get fixed
	fin >> type;
	fin >> size;
	Fixed_Module_set.resize(size);
	for (size_t i = 0; i < size; i++)
	{
		string name;
		int x, y, w, h;
		fin >> name >> x >> y >> w >> h;
		Fixed_Module_set[i] = new Fixed_Module(name, Point(y, x), h, w);
	}
	
	//get wire
	fin >> type;
	fin >> size;
	Wire_set.resize(size);
	for (size_t i = 0; i < size; i++)
	{
		string name;
		fin >> name;
		Module* a = NULL;
		Module* b = NULL;
		for (size_t j = 0; j < Fixed_Module_set.size() && a == NULL; j++)
			if (Fixed_Module_set[j]->getName() == name)
			{
				a = Fixed_Module_set[j];
				break;
			}
		for (size_t j = 0; j < Soft_Module_set.size() && a == NULL; j++)
			if (Soft_Module_set[j]->getName() == name)
			{
				a = Soft_Module_set[j];
				break;
			}
		fin >> name;
		for (size_t j = 0; j < Fixed_Module_set.size() && b == NULL; j++)
			if (Fixed_Module_set[j]->getName() == name)
			{
				b = Fixed_Module_set[j];
				break;
			}
		for (size_t j = 0; j < Soft_Module_set.size() && b == NULL; j++)
			if (Soft_Module_set[j]->getName() == name)
			{
				b = Soft_Module_set[j];
				break;
			}

		
		int Weight;
		fin >> Weight;
		Wire_set[i] = new Wire(a, b, Weight);
		a->getWire(Wire_set[i]);
		b->getWire(Wire_set[i]);
	}
	sort(Soft_Module_set.begin(), Soft_Module_set.end(), moduleAreaCompare);
}

void Plane::obtainPreResult(vector<Tile> softSet)
{
	ripoffALL();
	for (int i = 0; i < softSet.size(); i++)
	{
		Tile* toInst = new Tile(&softSet[i], 0);
		toInst->clearStitch();
		toInst->belong = Soft_Module_set[i];
		insert(toInst);
		Soft_Module_set[i]->update(toInst);
	}
}

void Plane::obtainPreResult(vector<vector<Tile>> softSet)
{
	ripoffALL();
	for (int i = 0; i < softSet.size(); i++)
	{
		for (int j = 0; j < softSet[i].size(); j++)
		{
			Tile* toInst = new Tile(&softSet[i][j], 0);
			toInst->clearStitch();
			toInst->belong = Soft_Module_set[i];
			insert(toInst);
			Soft_Module_set[i]->update(toInst);
		}
	}
}

Tile* Plane::point_finding(Point coord) { return point_finding(coord, boundary[0]); }
Tile* Plane::point_finding(Point coord, Tile* start)
{
	Tile* cur = start;
	while (!in_range(cur, coord))
	{
		if (cur->coord[1].x < coord.x)
			cur = cur->stitch[0];
		else if (cur->coord[0].x > coord.x)
			cur = cur->stitch[2];
		if (cur->coord[1].y < coord.y)
			cur = cur->stitch[3];
		else if (cur->coord[0].y > coord.y)
			cur = cur->stitch[1];
	}
	return cur;
}

void Plane::insertFix(Fixed_Module* F_Module)
{
	insert(F_Module->get_root());
	return;
}

void Plane::insert(Tile* tile)
{
	Tile* space;
	
	space = point_finding(point_right_up(tile));

	h_split(space, point_right_up(tile), 0);
	space = point_finding(point_left_down(tile));
	h_split(space, point_left_down(tile), 1);

	vector<Tile*> occupiedSpace;
	occupiedSpace = findOccupiedSpace(tile);
	DS(tile) = point_finding(LD(tile) + Point(-1, 0), occupiedSpace[0]);
	LS(tile) = point_finding(LD(tile) + Point(0, -1), occupiedSpace[0]);

	for (size_t i = 0; i < occupiedSpace.size(); i++)
	{
		RU(tile).y = RU(occupiedSpace[i]).y;
		if (i != occupiedSpace.size() - 1)
			US(tile) = occupiedSpace[i + 1];
		else
			US(tile) = point_finding(RU(tile) + Point(1, 0), occupiedSpace[i]);
		v_split(occupiedSpace[i], tile);
	}
	
	for (Tile* t = RS(tile); LS(t) == tile; t = DS(t))
	{
		if (LS(t) == LS(US(t)) && RS(t) == RS(US(t)))
			merge(t, US(t));
	}
	for (Tile* t = LS(tile); RS(t) == tile; t = US(t))
	{
		if (LS(t) == LS(DS(t)) && RS(t) == RS(DS(t)))
			merge(t, DS(t));
	}
	
	return;
}

vector<Tile*> Plane::findOccupiedSpace(Tile* oTile)		//from lower to top
{
	Tile* up, * down;
	up = point_finding(RU(oTile));
	down = point_finding(LD(oTile));

	vector<Tile*> occupiedSpace;
	occupiedSpace.push_back(down);
	if (down == up)
	{
		return occupiedSpace;
	}
	occupiedSpace.reserve((RU(up) - LD(down)).y);

	Tile* cur = down;
	Point findPoint = LD(oTile);
	findPoint.y = RU(cur).y + 1;

	while (findPoint.y <= RU(oTile).y)
	{
		cur = point_finding(findPoint, cur);
		findPoint.y = RU(cur).y + 1;
		occupiedSpace.push_back(cur);
	}

	return occupiedSpace;
}

void Plane::v_split(Tile* leftSpace, Tile* inTile)
{
	vector<Tile*> Neighbors;

	//�����л\�ɧR��space_tile_set������
	if (*inTile >= *leftSpace)
	{
		if (RU(inTile) == RU(leftSpace))
		{
			RS(inTile) = RS(leftSpace);
			US(inTile) = US(leftSpace);
		}
		if (LD(inTile) == LD(leftSpace))
		{
			LS(inTile) = LS(leftSpace);
			DS(inTile) = DS(leftSpace);
		}
		Neighbors = neighborFinding(inTile);
		updateStitch(Neighbors, inTile);
		delete leftSpace;
		leftSpace = NULL;
		return;
	}

	//�ˬd�O�_�ͦ��sspace
	Tile* rightSpace;
	if (LD(inTile).x == LD(leftSpace).x)
	{
		rightSpace = leftSpace;
		leftSpace = NULL;
	}
	else if (RU(inTile).x == RU(leftSpace).x)
	{
		rightSpace = NULL;
	}
	else
	{
		rightSpace = new Tile(leftSpace);
	}

	//general case
	Tile* cur;
	if (rightSpace != NULL)
	{
		LD(rightSpace).x = RU(inTile).x + 1;
		LS(rightSpace) = inTile;
		if (RS(inTile) != NULL)
			cur = RS(inTile);
		else
			cur = DS(inTile);
		while (!x_in_range(cur, LD(rightSpace)))
			cur = RS(cur);
		DS(rightSpace) = cur;
	}
	if (leftSpace != NULL)
	{
		RU(leftSpace).x = LD(inTile).x - 1;
		RS(inTile) = RS(leftSpace);
		RS(leftSpace) = inTile;
		cur = US(inTile);
		while (!x_in_range(cur, RU(leftSpace)))
			cur = LS(cur);
		US(leftSpace) = cur;
	}

	//RS(inTile)�̫�~��s�O�]���e���ݭn�ª�RS
	if (rightSpace != NULL)
		RS(inTile) = rightSpace;

	if (leftSpace != NULL)
	{
		Neighbors = neighborFinding(leftSpace);
		updateStitch(Neighbors, leftSpace);
	}
	if (rightSpace != NULL)
	{
		Neighbors = neighborFinding(rightSpace);
		updateStitch(Neighbors, rightSpace);
	}
	Neighbors = neighborFinding(inTile);
	updateStitch(Neighbors, inTile);
}

void Plane::h_split(Tile* space, Point split_point, int bound_side)
{
	if (abs(RU(space).y - split_point.y) < 1e-10 && bound_side == 0)
		return;
	if (abs(LD(space).y - split_point.y) < 1e-10 && bound_side == 1)
		return;
	if (!space->is_space())
	{
		return;
	}
	//set coord
	Point orgRightUp(point_right_up(space));
	Point orgLeftDown(point_left_down(space));
	orgRightUp.toInt();
	orgLeftDown.toInt();
	// 0 �N���U�����tile����split_point�o���I
	point_right_up(space).y = split_point.y - bound_side;

	Point new_ld_point(point_right_up(space).y + 1, point_left_down(space).x);
	Tile* new_space = new Tile(new_ld_point, orgRightUp);
	/*
	--------------
	|			 |
	|			 |
	|	new  	 |
	--------------
	|			 |
	|	old  	 |
	--------------
	*/


	//set stitch

	new_space->stitch[0] = space->stitch[0];
	new_space->stitch[3] = space->stitch[3];
	new_space->stitch[1] = space;
	space->stitch[3] = new_space;

	Tile* cur = RS(new_space);
	while (!y_in_range(cur, point_right_up(space)))
		cur = DS(cur);
	RS(space) = cur;

	cur = LS(space);
	while (!y_in_range(cur, point_left_down(new_space)))
		cur = cur->stitch[3];
	LS(new_space) = cur;

	vector<Tile*> Neighbors;
	Neighbors = neighborFinding(space);
	updateStitch(Neighbors, space);
	Neighbors = neighborFinding(new_space);
	updateStitch(Neighbors, new_space);
}

void Plane::updateStitch(vector<Tile*> neighbors, Tile* tile)
{
	for (size_t i = 0; i < neighbors.size(); i++)
	{
		if ((LD(tile).x - RU(neighbors[i]).x == 1) && y_in_range(tile, RU(neighbors[i])))	//nei �b tile ����
			RS(neighbors[i]) = tile;
		if ((LD(neighbors[i]).y - RU(tile).y == 1) && x_in_range(tile, LD(neighbors[i])))	//nei �b tile �W��
			DS(neighbors[i]) = tile;
		if ((LD(neighbors[i]).x - RU(tile).x == 1) && y_in_range(tile, LD(neighbors[i])))	//nei �b tile �k��
			LS(neighbors[i]) = tile;
		if ((LD(tile).y - RU(neighbors[i]).y == 1) && x_in_range(tile, RU(neighbors[i])))
			US(neighbors[i]) = tile;
	}

}


vector<Tile*> Plane::neighborFinding(Tile* tile)
{
	vector<Tile*> neighbors;
	neighbors.clear();
	neighbors.reserve(width(tile) + height(tile));
	findRightNeighbor(tile, neighbors);

	findDownNeighbor(tile, neighbors);

	findLeftNeighbor(tile, neighbors);

	findUpNeighbor(tile, neighbors);
	return neighbors;
}

void Plane::findRightNeighbor(Tile* tile, vector<Tile*>& neighbors)
{
	Tile* cur = RS(tile);
	while (RU(cur).y >= LD(tile).y)
	{
		neighbors.push_back(cur);
		cur = DS(cur);
		if (cur == NULL)
			break;
	}
	return;
}

void Plane::findDownNeighbor(Tile* tile, vector<Tile*>& neighbors)
{
	Tile* cur = DS(tile);					//findleftneighbor
	while (LD(cur).x <= RU(tile).x)
	{
		neighbors.push_back(cur);
		cur = RS(cur);
		if (cur == NULL)
			break;
	}
	return;
}

void Plane::findLeftNeighbor(Tile* tile, vector<Tile*>& neighbors)
{
	Tile* cur = LS(tile);
	while (LD(cur).y <= RU(tile).y)
	{
		neighbors.push_back(cur);
		cur = US(cur);
		if (cur == NULL)
			break;
	}
	return;
}

void Plane::findUpNeighbor(Tile* tile, vector<Tile*>& neighbors)
{
	Tile* cur = US(tile);
	while (RU(cur).x >= LD(tile).x)
	{
		neighbors.push_back(cur);
		cur = LS(cur);
		if (cur == NULL)
			break;
	}
	return;
}

void Plane::print()
{

	cout << endl;
	for (size_t i = 0; i < Fixed_Module_set.size(); i++)
	{
		cout << "Fixed_Module" << i << endl;
		if (Fixed_Module_set[i]->get_root() == NULL)
			cout << "NULL" << endl;
		else
			cout << *(Fixed_Module_set[i]->get_root()) << endl;
	}

	cout << endl;
	for (size_t i = 0; i < Soft_Module_set.size(); i++)
	{
		cout << "Soft_Module" << i << endl;
		Soft_Module_set[i]->print();
	}

}

void Plane::operator=(Plane& other)
{
	ripoffALL();
	vector<Tile*> othersRoot;
	othersRoot.resize(Soft_Module_set.size());
	for (int i = 0; i < othersRoot.size(); i++)
	{
		othersRoot[i] = other.getSoft(i)->get_root();
		Tile* newTile = new Tile(othersRoot[i],0);
		insert(newTile);
		getSoft(i)->update(newTile);
	}
}

Plane::~Plane()
{
	ripoffALL();
	cleanUp();
}

void Plane::cleanUp()
{
	
	for (int i = 0; i < Soft_Module_set.size(); i++)
		delete Soft_Module_set[i];
	for (int i = 0; i < Fixed_Module_set.size(); i++)
	{
		remove(Fixed_Module_set[i]->get_root());
		delete Fixed_Module_set[i];
	}
	for (int i = 0; i < Wire_set.size(); i++)
		delete Wire_set[i];

	
	delete DS(boundary[1]);
	delete boundary[1]->belong;
	for (int i = 0; i < 4; i++) 
		delete boundary[i];
}

