#include"Plane.h"
#include<iomanip>
#define Belong(a) a->belong

using namespace std;


int Plane::checkAllLegal()
{
	int cnt = 0;
	for (int i = 0; i < Soft_Module_set.size(); i++)
	{
		if (!Soft_Module_set[i]->checkLegal())
			cnt++;
	}
	return cnt;
}

bool Plane::Legal(int idx) { return Soft_Module_set[idx]->checkLegal(); }

Point Plane::generateSeed(int idx)
{
	Point newP;
	newP = Point(rand() % Height, rand() % Width);
	while (!point_finding(newP)->is_space())
		newP = Point(rand() % Height, rand() % Width);

	cout << newP << endl;
	Tile* newTile = new Tile(newP, 1, 1, Soft_Module_set[idx]);
	insert(newTile);
	Soft_Module_set[idx]->update(newTile);
	return newP;
}

Tile Plane::generateRand(int idx) {
	return generateRand(Soft_Module_set[idx]);
}

double Sig(int t)
{
	return 1.0 / (1 + exp(-((t - 500) / 100)));
}

Tile Plane::generateRand(Soft_Module* curSoft)
{

	Point curLD(rand() % Height, rand() % Width);
	Tile usable;
	Point opt = curSoft->optimPos();
	while (1)
	{
		do
		{
			double ratio = double(rand() % 100) / 100.0;
			curLD = Point(rand() % Height, rand() % Width) * ratio + opt * (1.0 - ratio);
			curLD.toInt();
		} while (!(point_finding(curLD)->is_space()));

		usable = findMaxUsableRect(curLD);
		if (area(&usable) >= 0.75 * curSoft->getMinArea() || rand() % 10 >= 9)
			break;
	}

	Tile* curRoot;
	if (area(&usable) <= curSoft->getMinArea())
	{
		if (!checkAllSpace(&usable))
			cout << "False" << endl;
		curRoot = new Tile(&usable);
		curRoot->belong = curSoft;
		curRoot->clearStitch();
	}
	else
	{
		int maxLen = floor(sqrt(2 * curSoft->getMinArea()));
		int minLen = ceil(sqrt(double(curSoft->getMinArea()) / 2));
		int choseLen;	//height
		if (maxLen != minLen)
			choseLen = rand() % (maxLen - minLen) + minLen;
		else
			choseLen = minLen;

		if (choseLen > height(&usable))
			choseLen = height(&usable);

		int choseWidth = curSoft->getMinArea() / choseLen;
		if (double(curSoft->getMinArea()) / choseLen - curSoft->getMinArea() / choseLen != 0)
			choseWidth++;
		if (choseWidth > width(&usable))
		{
			choseWidth = width(&usable);
			choseLen = curSoft->getMinArea() / choseWidth;
			if (double(curSoft->getMinArea()) / choseWidth - curSoft->getMinArea() / choseWidth != 0)
				choseLen++;
		}
		Point givenP = curLD - Point(choseLen / 2, choseWidth / 2);
		if (givenP.x < 0)
			givenP.x = 0;
		if (givenP.y < 0)
			givenP.y = 0;
		curRoot = new Tile(givenP, choseLen, choseWidth, curSoft);
		if (RU(curRoot).x > RU(&usable).x)
		{
			int movement = RU(curRoot).x - RU(&usable).x;
			RU(curRoot).x -= movement;
			LD(curRoot).x -= movement;
		}
		if (LD(curRoot).x < LD(&usable).x)
		{
			int movement = LD(&usable).x - LD(curRoot).x;
			RU(curRoot).x += movement;
			LD(curRoot).x += movement;
		}
		if (RU(curRoot).y > RU(&usable).y)
		{
			int movement = RU(curRoot).y - RU(&usable).y;
			RU(curRoot).y -= movement;
			LD(curRoot).y -= movement;
		}
		if (LD(curRoot).y < LD(&usable).y)
		{
			int movement = LD(&usable).y - LD(curRoot).y;
			RU(curRoot).y += movement;
			LD(curRoot).y += movement;
		}
	}

	if (curSoft->getComp().size() != 0)
		ripoff(curSoft);
	insert(curRoot);
	curSoft->update(curRoot);

	return curRoot;
}

Tile Plane::moveCenter(int idx) { return moveCenter(Soft_Module_set[idx]); }

Tile Plane::moveCenter(Soft_Module* curSoft)
{
	Tile* cur = new Tile(curSoft->get_root(), 0);
	Point LDorg = LD(cur);
	Point RUorg = RU(cur);
	ripoff(curSoft);
	while (1)
	{
		Point movement = Point(rand() % 3 - 1, rand() % 3 - 1);
		LD(cur) = LDorg + movement;
		RU(cur) = RUorg + movement;
		if (checkAllSpace(cur))
		{
			insert(cur);
			curSoft->update(cur);
			break;
		}
	}
	Tile update(cur);
	update.clearStitch();
	return update;
}

void Plane::minHPWLStretch(Soft_Module* curSoft, int Vertical)
{
	Point optPos = curSoft->optimPos();
	Point selfVal = curSoft->HPWLdir();
	Point estimate = curSoft->estHPWL(optPos);
	static void (*findNei)(Tile*, vector<Tile*>&);
	if (abs(selfVal.x - estimate.x) == 0 && abs(selfVal.y - estimate.y) == 0)
		return;

	bool TDorLR = 1;
	if (Vertical == 1)
	{
		if (optPos.x > curSoft->center().x)
			findNei = &findRightNeighbor;
		else
			findNei = &findLeftNeighbor;
		TDorLR = 0;
	}
	else if (Vertical == 0)
	{
		if (optPos.y > curSoft->center().y)
			findNei = &findUpNeighbor;
		else
			findNei = &findDownNeighbor;
	}
	else
		return;
	int startComp = curSoft->component.size() - 1;
	for (int i = 0; i < curSoft->component.size(); i++)
	{
		vector<Tile*> neiSpace = getAllSpace(findNei, curSoft->component[i]);
		for (int j = 0; j < neiSpace.size(); j++)
		{
			Tile* toInst = new Tile(neiSpace[j]);
			toInst->clearStitch();
			if (TDorLR)
			{
				if (LD(toInst).x < max({ LD(curSoft).x , LD(curSoft->component[i]).x }))
					LD(toInst).x = max({ LD(curSoft).x , LD(curSoft->component[i]).x });
				if (RU(toInst).x > min({ RU(curSoft).x , RU(curSoft->component[i]).x }))
					RU(toInst).x = min({ RU(curSoft).x , RU(curSoft->component[i]).x });
				while (!curSoft->LegalifAdd(toInst) && area(toInst) >= 0)
				{
					if (findNei == &findUpNeighbor)
						RU(toInst).y--;
					else
						LD(toInst).y++;
				}
				if (findNei == &findUpNeighbor && curSoft->pseudoCenter(toInst).y > optPos.y)
					RU(toInst).y = 2 * optPos.y - 1 - LD(toInst).y;
				else if (findNei == &findDownNeighbor && curSoft->pseudoCenter(toInst).y < optPos.y)
					LD(toInst).y = 2 * optPos.y - 1 - RU(toInst).y;

				if (area(toInst) <= 0 || !curSoft->LegalifAdd(toInst))
				{
					delete toInst;
					toInst = NULL;
				}
			}
			else
			{
				if (LD(toInst).y < LD(curSoft).y)
					LD(toInst).y = LD(curSoft).y;
				if (RU(toInst).y > RU(curSoft).y)
					RU(toInst).y = RU(curSoft).y;
				while (!curSoft->LegalifAdd(toInst) && area(toInst) >= 0)
				{
					if (findNei == &findUpNeighbor)
						RU(toInst).x--;
					else
						LD(toInst).x++;
				}
				if (findNei == &findRightNeighbor && curSoft->pseudoCenter(toInst).x > optPos.x)
					RU(toInst).x = 2 * optPos.x - 1 - LD(toInst).x;
				else if (findNei == &findLeftNeighbor && curSoft->pseudoCenter(toInst).x < optPos.x)
					LD(toInst).x = 2 * optPos.x - 1 - RU(toInst).x;

				if (area(toInst) <= 0 || !curSoft->LegalifAdd(toInst))
				{
					delete toInst;
					toInst = NULL;
				}
			}
			if (toInst != NULL && checkAllSpace(toInst))
			{
				toInst->belong = curSoft;
				insert(toInst);
				curSoft->update(toInst);
			}
			else
			{
				delete toInst;
				toInst = NULL;
			}
			if (i == 0)
				i += startComp;
		}
	}
}

vector<vector<Tile>> Plane::getAllTile()
{
	vector<vector<Tile>> outTile;
	outTile.resize(Soft_Module_set.size());
	for (int i = 0; i < Soft_Module_set.size(); i++)
	{
		vector<Tile*> currentSoft = Soft_Module_set[i]->getComp();
		outTile[i].resize(currentSoft.size());
		for (int j = 0; j < currentSoft.size(); j++)
		{
			outTile[i][j] = *currentSoft[j];
			outTile[i][j].clearStitch();
		}
	}
	return outTile;
}


void Plane::shrinkRoot(int idx) { shrinkRoot(Soft_Module_set[idx]); }

void Plane::shrinkRoot(Soft_Module* curSoft)
{
	if (curSoft->getComp().size() <= 1)
		return;

	int w;
	string shrinkSide;
	if (LD(curSoft).x != LD(curSoft->get_root()).x)
		shrinkSide = "right";
	else if (LD(curSoft).y != LD(curSoft->get_root()).y)
		shrinkSide = "top";
	else if (RU(curSoft).y != RU(curSoft->get_root()).y)
		shrinkSide = "down";
	else if (RU(curSoft).x != RU(curSoft->get_root()).x)
		shrinkSide = "left";
	else
		return;

	Tile* newRoot = new Tile(curSoft->get_root());
	if (shrinkSide == "right" || shrinkSide == "left")
		w = min({ int(5 * curSoft->getCurArea() - 4 * height(curSoft) * width(curSoft)) / height(curSoft),int(width(curSoft) - height(curSoft) / 2),int((curSoft->getCurArea() - curSoft->getMinArea()) / height(curSoft)) });
	else
		w = min({ int(5 * curSoft->getCurArea() - 4 * height(curSoft) * width(curSoft)) / width(curSoft),int(height(curSoft) - width(curSoft) / 2 - 1),int((curSoft->getCurArea() - curSoft->getMinArea()) / width(curSoft)) });
	if (w <= 0)
	{
		delete newRoot;
		return;
	}
	newRoot->belong = curSoft;
	newRoot->clearStitch();
	if (shrinkSide == "right")	//shrink from right
		RU(newRoot).x = RU(curSoft->get_root()).x + 1 - w;
	else if (shrinkSide == "left")
		LD(newRoot).x = w - 1 + LD(curSoft->get_root()).x;
	else if (shrinkSide == "down")
		LD(newRoot).y = w - 1 + LD(curSoft->get_root()).y;
	else
		RU(newRoot).y = RU(curSoft->get_root()).y + 1 - w;
	if (area(newRoot) <= 0)
	{
		delete newRoot;
		return;
	}
	remove(curSoft->get_root());
	insert(newRoot);
	curSoft->updateRoot(newRoot);
}


void Plane::minHPWLStretch(Soft_Module* curSoft)
{
	Point optPos = curSoft->optimPos();
	Point selfVal = curSoft->HPWLdir();
	Point estimate = curSoft->estHPWL(optPos);
	static void (*findNei)(Tile*, vector<Tile*>&);
	if (abs(selfVal.x - estimate.x) == 0 && abs(selfVal.y - estimate.y) == 0)
		return;

	bool TDorLR = 1;
	if (rand() % 10000 + 1 > 10000 * abs(selfVal.y - estimate.y) / (abs(estimate.x - selfVal.x) + abs(selfVal.y - estimate.y)))
	{
		if (optPos.x > curSoft->center().x)
			findNei = &findRightNeighbor;
		else
			findNei = &findLeftNeighbor;
		TDorLR = 0;
	}
	else
	{
		if (optPos.y > curSoft->center().y)
			findNei = &findUpNeighbor;
		else
			findNei = &findDownNeighbor;
	}
	for (int i = 0; i < curSoft->component.size(); i++)
	{
		vector<Tile*> neiSpace = getAllSpace(findNei, curSoft->component[i]);

		for (int j = 0; j < neiSpace.size(); j++)
		{
			Tile* toInst = new Tile(neiSpace[j]);
			toInst->clearStitch();
			if (TDorLR)
			{
				if (LD(toInst).x < LD(curSoft).x)
					LD(toInst).x = LD(curSoft).x;
				if (RU(toInst).x > RU(curSoft).x)
					RU(toInst).x = RU(curSoft).x;
				if (!curSoft->LegalifAdd(toInst))
				{
					if (findNei == &findUpNeighbor)
						RU(toInst).y = width(curSoft) * 2 - height(curSoft) - 1 + LD(toInst).y;
					else
						LD(toInst).y = RU(toInst).y + 1 - width(curSoft) * 2 + height(curSoft);
				}
				if (!curSoft->LegalifAdd(toInst))
				{
					if (findNei == &findUpNeighbor)
						RU(toInst).y = int((width(toInst) - LD(toInst).y * width(toInst) - 0.8 * width(curSoft) * height(curSoft) + 0.8 * width(curSoft) * RU(curSoft).y + curSoft->getCurArea()) / (0.8 * width(curSoft) - width(toInst)));
					else
						LD(toInst).y = int((0.8 * width(curSoft) * height(curSoft) + 0.8 * width(curSoft) * LD(curSoft).y - curSoft->getCurArea() - RU(toInst).y * width(toInst) - width(toInst)) / (0.8 * width(curSoft) - width(toInst)));
					cout << RU(toInst).y << endl << LD(toInst).y << endl;
				}
				if (area(toInst) <= 0)
				{
					delete toInst;
					toInst = NULL;
				}
			}
			else
			{
				if (LD(toInst).y < LD(curSoft).y)
					LD(toInst).y = LD(curSoft).y;
				if (RU(toInst).y > RU(curSoft).y)
					RU(toInst).y = RU(curSoft).y;
				if (!curSoft->LegalifAdd(toInst))
				{
					if (findNei == &findUpNeighbor)
						RU(toInst).x = height(curSoft) * 2 - width(curSoft) - 1 + LD(toInst).x;
					else
						LD(toInst).x = RU(toInst).x + 1 - height(curSoft) * 2 + width(curSoft);
				}
				if (!curSoft->LegalifAdd(toInst))
				{
					if (findNei == &findRightNeighbor)
						RU(toInst).x = int((height(toInst) - LD(toInst).x * height(toInst) - 0.8 * height(curSoft) * width(curSoft) + 0.8 * height(curSoft) * RU(curSoft).x + curSoft->getCurArea()) / (0.8 * height(curSoft) - height(toInst)));
					else
						LD(toInst).x = int((0.8 * height(curSoft) * width(curSoft) + 0.8 * height(curSoft) * LD(curSoft).x - curSoft->getCurArea() - RU(toInst).x * height(toInst) - height(toInst)) / (0.8 * height(curSoft) - height(toInst)));
					cout << LD(toInst).x << " " << RU(toInst).x << endl;
				}
				if (area(toInst) <= 0)
				{
					delete toInst;
					toInst = NULL;
				}
			}
			if (toInst != NULL && checkAllSpace(toInst))
			{
				toInst->belong = curSoft;
				insert(toInst);
				curSoft->update(toInst);
			}
			else
			{
				cout << ">" << endl;
				delete toInst;
				toInst = NULL;
			}
		}
	}

	Tile* newRoot = new Tile(curSoft->get_root());
	int w;
	if (findNei == &findLeftNeighbor || findNei == &findRightNeighbor)
		w = min({ int(5 * curSoft->getCurArea() - 4 * height(curSoft) * width(curSoft)) / height(curSoft),int(width(curSoft) - height(curSoft) / 2 - 1),int(curSoft->getCurArea() - curSoft->getMinArea()) / height(curSoft) });
	else
		w = min({ int(5 * curSoft->getCurArea() - 4 * height(curSoft) * width(curSoft)) / width(curSoft),int(height(curSoft) - width(curSoft) / 2 - 1),int(curSoft->getCurArea() - curSoft->getMinArea()) / width(curSoft) });

	if (w <= 0)
	{
		delete newRoot;
		return;
	}
	newRoot->belong = curSoft;
	newRoot->clearStitch();
	if (findNei == &findLeftNeighbor)	//shrink from right
		RU(newRoot).x = RU(curSoft->get_root()).x + 1 - w;
	else if (findNei == &findRightNeighbor)
		LD(newRoot).x = w - 1 + LD(curSoft->get_root()).x;
	else if (findNei == &findUpNeighbor)
		LD(newRoot).y = w - 1 + LD(curSoft->get_root()).y;
	else
		RU(newRoot).y = RU(curSoft->get_root()).y + 1 - w;

	remove(curSoft->get_root());
	insert(newRoot);
	curSoft->updateRoot(newRoot);

}

void findCurLeftRight(vector<Tile*> WspaceSet, int givenWhiteNo, int& curMaxLeft, int& curMinRight)
{
	curMaxLeft = 0;
	curMinRight = 0;
	for (int i = 1; i <= givenWhiteNo; i++)
	{
		if (LD(WspaceSet[i]).x >= LD(WspaceSet[curMaxLeft]).x)
			curMaxLeft = i;
		if (RU(WspaceSet[i]).x <= RU(WspaceSet[curMinRight]).x)
			curMinRight = i;
	}
	for (int i = WspaceSet.size() - 1; i >= givenWhiteNo; i--)
	{
		if (LD(WspaceSet[i]).x >= LD(WspaceSet[curMaxLeft]).x)
			curMaxLeft = i;
		if (RU(WspaceSet[i]).x <= RU(WspaceSet[curMinRight]).x)
			curMinRight = i;
	}

}

#include<stack>
Tile Plane::findUsableRect(Tile* included, Tile* objective)
{
	list<Tile> accepted_list;
	//include : the tile that is found must include this tile;
	//objective : the height/width of the tile
	stack<Tile> searching_list;
	searching_list.push(*included);
	while(!searching_list.empty())
	{
		Tile header = searching_list.top();
		searching_list.pop();
		Tile upper = Tile(Point(RU(&header).y+1,LD(&header).x), Point(RU(&header).y+1,RU(&header).x));
		vector<Tile*> neighbor = getSpaceTileInRegion(&upper);
		for(int i = 0 ; i < neighbor.size() ; i++)
		{
			Tile to_find = *neighbor[i];
			if(RU(&to_find).x > RU(&header).x)
				RU(&to_find).x = RU(&header).x;
			if(LD(&to_find).x < LD(&header).x)
				LD(&to_find).x = LD(&header).x;
			/*make a sudo tile*/
			if(width(&to_find) < width(objective))
				continue;
			Tile pseudo = Tile(Point(RU(&to_find).y-height(objective)+1,LD(&to_find).x),RU(&to_find));
			if(checkAllSpace(&pseudo,included))
			{
				accepted_list.push_back(pseudo);
				continue;
			}
			searching_list.push(to_find);
		}
	}
	searching_list.push(*included);
	while(!searching_list.empty())
	{
		Tile header = searching_list.top();
		searching_list.pop();
		Tile lower = Tile(Point(LD(&header).y-1,LD(&header).x), Point(LD(&header).y-1,RU(&header).x));
		vector<Tile*> neighbor = getSpaceTileInRegion(&lower);
		for(int i = 0 ; i < neighbor.size() ; i++)
		{
			Tile to_find = *neighbor[i];
			if(RU(&to_find).x > RU(&header).x)
				RU(&to_find).x = RU(&header).x;
			if(LD(&to_find).x < LD(&header).x)
				LD(&to_find).x = LD(&header).x;
			/*make a sudo tile*/
			if(width(&to_find) < width(objective))
				continue;
			Tile pseudo = Tile(LD(&to_find),Point(LD(&to_find).x + height(objective)-1,RU(&to_find).x));
			if(checkAllSpace(&pseudo,included))
			{
				accepted_list.push_back(pseudo);
				continue;
			}
			searching_list.push(to_find);
		}
	}
	if(searching_list.size() == 0)
	{
		return Tile(Point(-999999999,-999999999),Point(-999999999,-999999999));
	}
	double dist = 100000000000000000;
	Tile to_return;
	for(Tile& T : accepted_list)
	{
		Point vec = objective->coord[0] + objective->coord[1] - T.coord[0] - T.coord[1];
		double cur_dist = (abs(vec.x) + abs(vec.y))/2;
		if(dist >= cur_dist)
		{
			dist = cur_dist;
			to_return = T;
		}
	}
	return to_return;
}

Tile Plane::findMaxUsableRect(Point given)
{
	list<Tile*> Wspacels;
	Wspacels.push_back(point_finding(given));
	Tile* givenWhite = Wspacels.front();

	Tile* upperWhite = point_finding(Point(RU(Wspacels.back()).y + 1, given.x), Wspacels.back());
	while (upperWhite->belong == NULL)
	{
		Wspacels.push_back(upperWhite);
		upperWhite = point_finding(Point(RU(upperWhite).y + 1, given.x), upperWhite);
	}

	Tile* lowerWhite = point_finding(Point(LD(Wspacels.front()).y - 1, given.x), Wspacels.front());
	while (lowerWhite->belong == NULL)
	{
		Wspacels.push_front(lowerWhite);
		lowerWhite = point_finding(Point(LD(lowerWhite).y - 1, given.x), lowerWhite);
	}

	vector<Tile*> WspaceSet;
	WspaceSet.reserve(Wspacels.size());
	for (Tile* Wspace : Wspacels)
		WspaceSet.push_back(Wspace);

	int givenWhiteNo;
	for (int i = 0; i < WspaceSet.size(); i++)
	{
		if (WspaceSet[i] == givenWhite)
		{
			givenWhiteNo = i;
			break;
		}
	}


	int boundLeft = LD(givenWhite).x;
	int boundRight = RU(givenWhite).x;

	int curMaxLeft;		//Tile no. not value
	int curMinRight;

	findCurLeftRight(WspaceSet, givenWhiteNo, curMaxLeft, curMinRight);
	int curWidth = RU(WspaceSet[curMinRight]).x - LD(WspaceSet[curMaxLeft]).x + 1;
	int curHeight = RU(WspaceSet.back()).y - LD(WspaceSet.front()).y + 1;
	Tile largest = Tile(Point(LD(WspaceSet.front()).y, LD(WspaceSet[curMaxLeft]).x), curHeight, curWidth, 0);

	if (curWidth * 2 < curHeight)
		largest = Tile(Point(LD(WspaceSet.front()).y, LD(WspaceSet[curMaxLeft]).x), curWidth * 2, curWidth, 0);
	else if (curHeight * 2 < curWidth)
		largest = Tile(Point(LD(WspaceSet.front()).y, LD(WspaceSet[curMaxLeft]).x), curHeight, curHeight * 2, 0);
	else
		largest = Tile(Point(LD(WspaceSet.front()).y, LD(WspaceSet[curMaxLeft]).x), curHeight, curWidth, 0);
	
	 
	while (curMaxLeft != givenWhiteNo && curMinRight != givenWhiteNo)
	{

		if (curMaxLeft < givenWhiteNo && curMaxLeft >= 0)
		{
			int cnt = 0;
			for (int i = curMaxLeft + 1; i < WspaceSet.size(); i++)
				WspaceSet[cnt++] = WspaceSet[i];
			givenWhiteNo = givenWhiteNo - curMaxLeft - 1;
			WspaceSet.resize(cnt);
		}
		else if (curMaxLeft > givenWhiteNo && curMaxLeft < WspaceSet.size())
			WspaceSet.resize(curMaxLeft);

		findCurLeftRight(WspaceSet, givenWhiteNo, curMaxLeft, curMinRight);
		int newWidth = RU(WspaceSet[curMinRight]).x - LD(WspaceSet[curMaxLeft]).x + 1;
		int newHeight = RU(WspaceSet.back()).y - LD(WspaceSet.front()).y + 1;
		Tile newLargest;
		if (newWidth * 2 < newHeight)
			newLargest = Tile(Point(LD(WspaceSet.front()).y, LD(WspaceSet[curMaxLeft]).x), newWidth * 2, newWidth, 0);
		else if (newHeight * 2 < newWidth)
			newLargest = Tile(Point(LD(WspaceSet.front()).y, LD(WspaceSet[curMaxLeft]).x), newHeight, newHeight * 2, 0);
		else
			newLargest = Tile(Point(LD(WspaceSet.front()).y, LD(WspaceSet[curMaxLeft]).x), newHeight, newWidth, 0);

		if (area(&newLargest) >= area(&largest))
			largest = newLargest;

		if (curMinRight < givenWhiteNo && curMinRight >= 0)
		{
			int cnt = 0;
			for (int i = curMinRight + 1; i < WspaceSet.size(); i++)
				WspaceSet[cnt++] = WspaceSet[i];
			givenWhiteNo = givenWhiteNo - curMinRight - 1;
			WspaceSet.resize(cnt);
		}
		else if (curMinRight > givenWhiteNo && curMinRight < WspaceSet.size())
			WspaceSet.resize(curMinRight);

		findCurLeftRight(WspaceSet, givenWhiteNo, curMaxLeft, curMinRight);
		newWidth = RU(WspaceSet[curMinRight]).x - LD(WspaceSet[curMaxLeft]).x + 1;
		newHeight = RU(WspaceSet.back()).y - LD(WspaceSet.front()).y + 1;

		if (newWidth * 2 < newHeight)
			newLargest = Tile(Point(LD(WspaceSet.front()).y, LD(WspaceSet[curMaxLeft]).x), newWidth * 2, newWidth, 0);
		else if (newHeight * 2 < newWidth)
			newLargest = Tile(Point(LD(WspaceSet.front()).y, LD(WspaceSet[curMaxLeft]).x), newHeight, newHeight * 2, 0);
		else
			newLargest = Tile(Point(LD(WspaceSet.front()).y, LD(WspaceSet[curMaxLeft]).x), newHeight, newWidth, 0);

		if (area(&newLargest) >= area(&largest))
			largest = newLargest;

	}

	if (width(&largest) * 2 < height(&largest))
		largest = Tile(LD(&largest), width(&largest) * 2, width(&largest), 0);
	else if (height(&largest) * 2 < width(&largest))
		largest = Tile(LD(&largest), height(&largest), height(&largest) * 2, 0);


	vector<Tile*> rightNei;
	rightNei.reserve(Wspacels.size());
	Tile* curRight = point_finding(RU(&largest), givenWhite);
	rightNei.push_back(curRight);
	while (LD(curRight).y > LD(&largest).y)
	{
		curRight = point_finding(Point(LD(curRight).y - 1, RU(&largest).x), curRight);
		rightNei.push_back(curRight);
	}


	int rightLimitPos = 10000000;
	for (int i = 0; i < rightNei.size(); i++)
	{
		if (RU(rightNei[i]).x < rightLimitPos)
			rightLimitPos = RU(rightNei[i]).x;
	}
	int maxMovement = rightLimitPos - RU(&largest).x;
	int movement = max({ 0,int(given.x - Center(&largest).x - 0.5) });
	if (movement < maxMovement)
	{
		RU(&largest).x += movement;
		LD(&largest).x += movement;
	}
	else
	{
		RU(&largest).x += maxMovement;
		LD(&largest).x += maxMovement;
	}

	if (!checkAllSpace(&largest))
		cout << "False" << endl;

	return largest;
}


Point Plane::generateSeed(int idx, Point& best) { return generateSeed(Soft_Module_set[idx], best); }
Point Plane::generateSeed(Soft_Module* curSoft, Point& best)
{
	Point newLD;
	newLD.x = floor(best.x);
	newLD.y = floor(best.y);
	int len = 1 + floor(sqrt(curSoft->getMinArea() / 2));

	Tile* New = new Tile(newLD, len, len, curSoft);
	int cnt = 0;
	while (!checkAllSpace(New))
	{
		newLD = LD(point_finding(newLD));
		*New = Tile(newLD, len, len, curSoft);
		if (!checkAllSpace(New))
		{
			newLD = Point(rand() % Height, rand() % Width);
			*New = Tile(newLD, len, len, curSoft);
		}
		if (cnt % 50 == 0 && len > 1)
			len *= 0.975;
		cnt++;
	}
	insert(New);
	curSoft->update(New);
	return newLD;
}



bool Plane::fillUp(int idx, int mode) { return fillUp(Soft_Module_set[idx], mode); }
bool Plane::fillUp(Soft_Module* curSoft, int mode)
{
	bool success = 0;
	if (mode && RLside != 0)
	{
		for (auto& subcomp : curSoft->getComp())
		{
			vector<Tile*> space;
			space = getAllSpace(findRightNeighbor, subcomp);
			for (int i = 0; i < space.size(); i++)
			{
				int NewWidth;
				NewWidth = floor((curSoft->getMinArea() - curSoft->getCurArea()) / height(space[i]));

				if (LD(space[i]).x + NewWidth - 1 > RU(space[i]).x)
					NewWidth = width(space[i]);
				if (LD(space[i]).x + NewWidth - 1 > RU(curSoft).x)
					NewWidth = RU(curSoft).x - LD(space[i]).x + 1;

				if (NewWidth <= 0)
					continue;
				success = 1;
				Tile* newOne = new Tile(LD(space[i]), height(space[i]), NewWidth, curSoft);
				insert(newOne);
				curSoft->update(newOne);
			}
		}
		for (auto& subcomp : curSoft->getComp())
		{
			vector<Tile*> space;
			space = getAllSpace(findLeftNeighbor, subcomp);
			for (int i = 0; i < space.size(); i++)
			{
				int NewWidth;
				NewWidth = floor((curSoft->getMinArea() - curSoft->getCurArea()) / height(space[i]));

				if (RU(space[i]).x - NewWidth + 1 < LD(space[i]).x)
					NewWidth = width(space[i]);
				if (RU(space[i]).x - NewWidth + 1 < LD(curSoft).x)
					NewWidth = RU(space[i]).x - LD(curSoft).x + 1;
				if (NewWidth <= 0)
					continue;
				success = 1;
				Tile* newOne = new Tile(height(space[i]), NewWidth, RU(space[i]), curSoft);
				insert(newOne);
				curSoft->update(newOne);
			}
		}
	}
	return success;
}

void Plane::softPatch(int idx)
{
	bool insertable = 1;
	bool fail = 0;
	Point dummy(0, 0);
	while (insertable)
	{
		insertable = 0;
		if (fillUp(idx, RLside | MinAreaLegal))
			insertable = 1;
		else if (patch(idx, One, dummy))
			insertable = 1;
		else if (!Legal(idx))
			break;
	}
}

bool Plane::patch(int idx, int mode, Point& best) { return patch(Soft_Module_set[idx], mode, best); }
bool Plane::patch(Soft_Module* curSoft, int mode, Point& best)
{
	bool success = 0;
	if (curSoft->getCurArea() >= curSoft->getMinArea())
		return 0;

	if (mode == One)
	{
		int constrain = floor(0.1 * sqrt(curSoft->getMinArea() - curSoft->getCurArea()));
		if (constrain < 1)
			constrain = 1;
		if (width(curSoft) > height(curSoft))
		{
			if (width(curSoft->getUp()) > width(curSoft->getDown()))
			{
				success = patchOne(curSoft, 1, constrain);
				if (!success)
					success = patchOne(curSoft, 3, constrain);
				if (!success)
					success = patchOne(curSoft, 2, constrain);
				if (!success)
					success = patchOne(curSoft, 0, constrain);
			}
			else
			{
				success = patchOne(curSoft, 3, constrain);
				if (!success)
					success = patchOne(curSoft, 1, constrain);
				if (!success)
					success = patchOne(curSoft, 0, constrain);
				if (!success)
					success = patchOne(curSoft, 2, constrain);
			}
		}
		else
		{
			if (height(curSoft->getLeft()) > height(curSoft->getRight()))
			{
				success = patchOne(curSoft, 0, constrain);
				if (!success)
					success = patchOne(curSoft, 2, constrain);
				if (!success)
					success = patchOne(curSoft, 3, constrain);
				if (!success)
					success = patchOne(curSoft, 1, constrain);
			}
			else
			{
				success = patchOne(curSoft, 2, constrain);
				if (!success)
					success = patchOne(curSoft, 0, constrain);
				if (!success)
					success = patchOne(curSoft, 1, constrain);
				if (!success)
					success = patchOne(curSoft, 3, constrain);
			}
		}
	}

	return success;
}

bool Plane::patchOne(Soft_Module* curSoft, int side, int Constraint)
{
	Tile* originalUp = curSoft->getUp();
	Tile* originalDown = curSoft->getDown();
	Tile* originalRight = curSoft->getRight();
	Tile* originalLeft = curSoft->getLeft();
	bool success = 0;
	if (side == 0)
	{
		Tile* leftOne = curSoft->getLeft();
		vector<Tile*> lspace;
		lspace = getAllSpace(findLeftNeighbor, leftOne);
		if (lspace.size() != 0)
		{
			Tile* lsp = getByMaxWidth(lspace);
			int MinWidth = width(lsp);
			int expandSize = min({ Constraint, MinWidth });
			if (expandSize == 0)
				return 0;

			for (auto& lsp : lspace)
			{
				if (curSoft->getMinArea() - curSoft->getCurArea() <= 0)
					break;
				int len = min({ expandSize ,width(lsp) });
				int hei = min({ height(lsp) ,(curSoft->getMinArea() - curSoft->getCurArea() / len) });
				if (hei == 0)
				{
					len = 1;
					hei = 1;
				}
				Tile* newLeft = new Tile(hei, len, RU(lsp), curSoft);
				if (area(newLeft) > curSoft->getMinArea() - curSoft->getCurArea())
					LD(newLeft) = RU(newLeft) + Point(0, -1);
				insert(newLeft);
				curSoft->update(newLeft);
			}
			success = 1;
		}
	}
	if (side == 2)
	{
		Tile* rightOne = curSoft->getRight();
		vector<Tile*> rspace;
		rspace = getAllSpace(findRightNeighbor, rightOne);
		if (rspace.size() != 0)
		{
			Tile* rsp = getByMaxWidth(rspace);
			int MinWidth = width(rsp);
			int expandSize = min({ Constraint, MinWidth });
			if (expandSize == 0)
				return 0;

			for (auto& rsp : rspace)
			{
				if (curSoft->getMinArea() - curSoft->getCurArea() <= 0)
					break;
				int len = min({ expandSize ,width(rsp) });
				int hei = min({ height(rsp) ,(curSoft->getMinArea() - curSoft->getCurArea() / len) });
				if (hei == 0)
				{
					len = 1;
					hei = 1;
				}
				Tile* newRight = new Tile(LD(rsp), height(rsp), len, curSoft);
				if (area(newRight) > curSoft->getMinArea() - curSoft->getCurArea())
					RU(newRight) = LD(newRight);
				insert(newRight);
				curSoft->update(newRight);

			}
			success = 1;
		}
	}

	if (side == 1)
	{
		Tile* UpOne = curSoft->getUp();
		vector<Tile*> uspace;
		uspace = getAllSpace(findUpNeighbor, UpOne);
		if (uspace.size() != 0)
		{
			Tile* usp = getByMaxHeight(uspace);
			int MaxHeight = height(usp);
			int expandSize = min({ Constraint, MaxHeight });
			if (expandSize == 0)
				return 0;
			for (auto& usp : uspace)
			{
				if (curSoft->getMinArea() - curSoft->getCurArea() <= 0)
					break;
				int h = min({ expandSize,height(usp) });
				Point newLD = LD(usp);
				newLD.x = max({ newLD.x, LD(UpOne).x });
				Point newRU = RU(usp);
				newRU.x = min({ newRU.x, RU(UpOne).x });
				newRU.y = newLD.y + h - 1;
				Tile* newUp = new Tile(newLD, newRU, curSoft);
				if (area(newUp) > curSoft->getMinArea() - curSoft->getCurArea())
					RU(newUp) = LD(newUp);
				insert(newUp);
				curSoft->update(newUp);
				success = 1;
			}
		}
	}
	if (side == 3)
	{
		Tile* DownOne = curSoft->getDown();
		vector<Tile*> dspace;
		dspace = getAllSpace(findDownNeighbor, DownOne);
		if (dspace.size() != 0)
		{
			Tile* dsp = getByMaxHeight(dspace);
			int MaxHeight = height(dsp);
			int expandSize = min({ Constraint, MaxHeight });
			if (expandSize == 0)
				return 0;
			for (auto& dsp : dspace)
			{
				if (curSoft->getMinArea() - curSoft->getCurArea() <= 0)
					break;
				int h = min({ expandSize,height(dsp) });
				Point newLD = LD(dsp);
				newLD.x = max({ newLD.x, LD(DownOne).x });
				Point newRU = RU(dsp);
				newRU.x = min({ newRU.x, RU(DownOne).x });
				newLD.y = newRU.y - h + 1;
				Tile* newDown = new Tile(newLD, newRU, curSoft);
				if (area(newDown) > curSoft->getMinArea() - curSoft->getCurArea())
					LD(newDown) = RU(newDown);

				insert(newDown);
				curSoft->update(newDown);
				success = 1;
			}
		}
	}
	return success;
}

int calArea(vector<Point> Pset)
{
	int curArea = 0;
	for (int i = 0; i < Pset.size() - 1; i++)
	{
		curArea += Pset[i].x * Pset[i + 1].y;
		curArea -= Pset[i + 1].x * Pset[i].y;
	}
	curArea += Pset[Pset.size() - 1].x * Pset[0].y;
	curArea -= Pset[0].x * Pset[Pset.size() - 1].y;

	return (abs(curArea) / 2);
}

void Plane::output(char* file)
{
	ofstream fout;
	fout.open(file);
	fout << "HPWL " << fixed << setprecision(1) << all_HPWL() << endl;
	fout << setprecision(0);
	fout << "SOFTMODULE " << Soft_Module_set.size() << endl;
	for (int i = 0; i < ordered_set.size(); i++)
	{
		vector<Point> Pset;
		Pset = ordered_set[i]->listPoint(point_finding);
		fout << ordered_set[i]->getName() << " " << Pset.size() << endl;
		for (int j = 0; j < Pset.size(); j++)
			fout << Pset[j].x << " " << Pset[j].y << endl;
		//		cout << calArea(Pset) << endl;
	}
}

double Plane::all_HPWL()
{
	double sol = 0;
	for (int i = 0; i < Wire_set.size(); i++)
		sol += Wire_set[i]->HPWL();
	return sol;
}

void Plane::outimg()
{
	ofstream fout;
	fout.open("./IMG/img.txt");
	fout <<fixed<<setprecision(0)<< Width << " " << Height << endl;
	fout << "FIXEDMODULE " << Fixed_Module_set.size() << endl;
	for (int i = 0; i < Fixed_Module_set.size(); i++)
	{
		Tile* tmp = Fixed_Module_set[i]->get_root();
		fout << LD(tmp) << " " << width(tmp) << " " << height(tmp) << endl;
	}

	fout << "SOFTMODULE " <<fixed<< Soft_Module_set.size() << endl;
	for (int i = 0; i < Soft_Module_set.size(); i++)
	{
		fout << Soft_Module_set[i]->getName();
		vector<Tile*> Comp = Soft_Module_set[i]->getComp();
		fout << " " <<fixed<< Comp.size() << endl;
		for (int j = 0; j < Comp.size(); j++)
			fout <<fixed<< LD(Comp[j]) << " " << width(Comp[j]) << " " << height(Comp[j]) << endl;
	}

	fout << "Wire " <<fixed<< Wire_set.size() << endl;
	for (int i = 0; i < Wire_set.size(); i++)
	{
		fout <<fixed<< Wire_set[i]->weight << "	" << Wire_set[i]->a->center() << "	" << Wire_set[i]->b->center() << endl;
	}
	fout.close();

}