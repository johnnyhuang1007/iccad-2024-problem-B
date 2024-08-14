#ifndef _FLOW_FUNC_H_
#define _FLOW_FUNC_H_

#include"Plane.h"
#include"PlacerInOut.h"
#include <unistd.h>
#include<ctime>

#define save 1
#define dontsave 0
#define V 1
#define H 0
#define doShrink 1
using namespace std;


void FindBetterPlaceAP(Plane&, vector<Tile>&, vector<int>&, double&);
void wireReduceSA(Plane&, vector<Tile>&, vector<int>&);

void generateSeedGreedy(Plane&, vector<Tile>&, vector<int>&, double&);
void fillInStage(Plane&, vector<int>&);
void handlingFail(Plane& chip, vector<int>& chosen, double&);


ofstream file;
void writeCSV(Plane&, double, int);
void Initializing(Plane&, vector<Tile>&, vector<int>&, double&);
bool wireReduceSA(Plane&, vector<Tile>&, vector<int>&, double, double, double, int, int, bool, void(*operatorFunc)(vector<int>, vector<Tile>&, Plane&));
void generateSeedGreedy(Plane&, vector<Tile>&, vector<int>&, double&);

vector<Tile> resultSoft;
double bestHPWL = 100000000000000000;
vector<vector<Tile>> resultSoftShape;
double hightestTmp = 60000;

void handlingFail(Plane& chip, vector<int>& chosen, double& score)
{
	
	chosen.clear();
	for (int i = 0; i < chip.softModule_size(); i++)
	{
		if (chip.Legal(i))
			continue;
		chosen.push_back(i);
		if (chip.getSoft(i)->getCurArea() != 0)
			chip.ripoff(i);
	}
	int para = chip.softModule_size();
	if (chosen.size() == 0)
	{
		chosen.reserve(0);
		return;
	}
	
	for (int i = 0; i < chip.softModule_size(); i++)
	{
		if (chip.Legal(i) && (rand() % para == 1))
		{
			chosen.push_back(i);
			chip.ripoff(i);
		}
	}
	
	for (int i = 0; i < chosen.size() - 1; i++)
		for (int j = i + 1; j < chosen.size(); j++)
			if (chosen[i] == chosen[j])
			{
				chosen[j] = chosen.back();
				chosen.pop_back();
				j--;
			}
	
}

void FindBetterPlaceAP(Plane& chip, vector<Tile>& currentSoft, vector<int>& initChosen, double& score)
{
	vector<Tile> newSoft;
	newSoft.resize(currentSoft.size());
	vector<int> chosen;
	chosen.resize(0.25 * initChosen.size());
	if (chosen.size() < 2 && initChosen.size() > 1)
		chosen.resize(2);
	else if (chosen.size() < 2)
		chosen.resize(1);
	bool AreaPrior = 0;
	int cnt = 0, repeat = 0;
	while (1)
	{

		if (repeat > 100 || score >= 0.5)
			break;
		cnt++;
		repeat++; 
		
		double newScore = 2;
		double preHPWL = chip.all_HPWL();
		newSoft = currentSoft;

		for (size_t j = 0; j < chosen.size(); j++)
		{
			while (1)
			{
				chosen[j] = initChosen[rand() % initChosen.size()];
				bool success = 1;
				for (size_t k = 0; k < j; k++)
					if (chosen[k] == chosen[j])
						success = 0;
				if (success)
					break;
			}

		}

		for (int& idx : chosen)
			chip.ripoff(idx);
		for (int& idx : chosen)
		{
			newSoft[idx] = chip.generateRand(idx);
			newSoft[idx].clearStitch();
		}

		for (size_t j = 0; j < chip.softModule_size(); j++)
			newScore = min({ double(area(&newSoft[j])) / chip.getSoft(j)->getMinArea(), newScore });
		if (newScore >= score)
		{
			if (newScore > score)
				repeat = 0; 
			currentSoft = newSoft;
			score = newScore;
		}
		else
		{
			for (int& idx : chosen)
				chip.ripoff(idx);
			for (int& idx : chosen)
			{
				Tile* newTile = new Tile(&currentSoft[idx]);
				chip.insert(newTile);
				chip.getSoft(idx)->update(newTile);
			}
		}
	}
}


void fillInStage(Plane& chip, vector<int>& chosen)
{
	bool insertable = 1;
	bool fail = 0;
	Point dummy(0, 0);
	while (insertable)
	{
		insertable = 0;
		for (int i : chosen)
		{
			if (chip.fillUp(i, RLside | MinAreaLegal))
				insertable = 1;
			else if (chip.patch(i, One, dummy))
				insertable = 1;
			else if (!chip.Legal(i))
			{
				fail = 1;
				break;
			}
		}
		if (fail)
			break;
	}
}



vector<Tile> getPreRoot(char* fileName)
{
	ifstream rootFile;
	rootFile.open(fileName);
	int Size;
	rootFile >> Size;
	vector<Tile> preSoft;
	preSoft.resize(Size);
	for (int cur = 0; cur < Size; cur++)
	{
		int LDx, LDy, RUx, RUy;
		rootFile >> LDx >> LDy >> RUx >> RUy;
		LD(&preSoft[cur]) = Point(LDy, LDx);
		RU(&preSoft[cur]) = Point(RUy, RUx);
		preSoft[cur].clearStitch();
		preSoft[cur].belong = NULL;
	}
	return preSoft;
}

void outRoot(vector<Tile> currentSoft, char* outfileName)
{
	string outName = outfileName;
	outName[0] = 'r';
	outName[1] = 'o';
	outName[2] = 'o';
	outName[3] = 't';
	ofstream rootFile;
	rootFile.open(outName.c_str());
	rootFile << currentSoft.size() << endl;
	for (int i = 0; i < currentSoft.size(); i++)
	{
		rootFile << LD(&currentSoft[i]) << " " << RU(&currentSoft[i]) << endl;
	}
}

double CSVstart = clock();

void writeCSV(Plane& chip, double temperature, int iterTime)
{
	double preArea = 0;
	for (int idx = 0; idx < chip.softModule_size(); idx++)
		preArea += min({ double(chip.getSoft(idx)->getCurArea()) / chip.getSoft(idx)->getMinArea(),1.0 });
	file << iterTime << ',' << temperature << "," << chip.all_HPWL() << "," << preArea<< (CSVstart - clock())/1000000 << endl;
}

void Initializing(Plane& chip, vector<Tile>& currentSoft, vector<int>& chosen, double& score)
{

	bestHPWL = 100000000000000000;
	resultSoft.resize(chip.softModule_size());
	for (int i = 0; i < chip.fixedModule_size(); i++)
		chip.insert_fix(i);
	resultSoftShape.clear();
	currentSoft.resize(chip.softModule_size(), Tile(Point(0, 0), 0, 0));
	chosen.resize(chip.softModule_size());
	for (int i = 0; i < chosen.size(); i++)
		chosen[i] = i;

	score = 2;
}


void generateSeedGreedy(Plane& chip, vector<Tile>& currentSoft, vector<int>& chosen, double& score)
{
	for (int i : chosen)
	{
		if (chip.getSoft(i)->getCurArea() == 0)
			chip.generateRand(i);
		currentSoft[i] = *(chip.getSoft(i)->get_root());
		currentSoft[i].clearStitch();
		score = min({ double(area(&currentSoft[i])) / chip.getSoft(i)->getMinArea(),score });
	}
}


double Omega(double temperature)
{
	if (temperature >= 2000) return 0.85;
	return 0.98;
	if (temperature >= 1000) return 0.95;
	if (temperature >= 500) return 0.95;
	if (temperature >= 200) return 0.96;
	if (temperature >= 10) return 0.95;
	return 0.98;
}

void randBase(vector<int> chosen, vector<Tile>& newSoft, Plane& chip)
{
	for (int& idx : chosen)
		chip.ripoff(idx);

	for (int& idx : chosen)
	{
		newSoft[idx] = chip.generateRand(idx);
		newSoft[idx].clearStitch();
	}
}

void moveBase(vector<int> chosen, vector<Tile>& newSoft, Plane& chip)
{
	for (int& idx : chosen)
	{
		newSoft[idx] = chip.moveCenter(idx);
		newSoft[idx].clearStitch();
	}
}

void shapeBase(vector<int> chosen, vector<Tile>& newSoft, Plane& chip)
{
	for (int& idx : chosen)
		chip.ripoff(idx);
	for (int& idx : chosen)
	{
		//	newSoft[idx] = chip.reshape(idx);
		newSoft[idx].clearStitch();
	}
}

void rectiLinearSA(Plane& chip, vector<Tile>& currentSoft, char* out, bool Shrink, vector<vector<Tile>>& prevTiles)
{

	vector<vector<Tile>> InitState = prevTiles;
	chip.obtainPreResult(prevTiles);
	double Tmp = 1000;
	double outputHPWL = chip.all_HPWL();
	vector<int> order;
	order.resize(chip.softModule_size());
	for (int i = 0; i < order.size(); i++)
		order[i] = i;

	vector<int>dir;
	dir.resize(chip.softModule_size(), V);
	for (int i = 0; i < order.size(); i++)
		chip.minHPWLStretch(chip.getSoft(order[i]), dir[i]);
	double preHPWL = chip.all_HPWL();


	while (Tmp >= 0.001)
	{
		for (int i = 0; i < 3 * chip.softModule_size(); i++)
		{

			vector<int> old_order = order;
			vector<int>old_dir = dir;
			bool changeDir = rand() % 2;
			if (changeDir)
				dir[rand() % dir.size()] = rand() % 3;
			else
			{
				int toChange1 = rand() % dir.size();
				int toChange2 = rand() % dir.size();
				swap(order[toChange1], order[toChange2]);
				swap(dir[toChange1], dir[toChange2]);
			}

			chip.obtainPreResult(InitState);
			for (int i = 0; i < order.size(); i++)
			{
				chip.minHPWLStretch(chip.getSoft(order[i]), dir[i]);
				if (Shrink)
					chip.shrinkRoot(order[i]);
			}
			double D = (chip.all_HPWL() - preHPWL) / preHPWL;
			if (D < 0 || double(rand() % 100) / 100.0 < exp(-1 * D * hightestTmp / Tmp))
			{
				preHPWL = chip.all_HPWL();
				if (outputHPWL > chip.all_HPWL() && chip.checkAllLegal() == 0)
				{
					outputHPWL = chip.all_HPWL();
					bestHPWL = outputHPWL;
					chip.outimg();
					chip.output(out);
					prevTiles = chip.getAllTile();
					resultSoftShape = prevTiles;
					writeCSV(chip, Tmp, 1);
				}
			}
			else
			{
				order = old_order;
				dir = old_dir;
			}
		}
		Tmp *= 0.95;
	}
	chip.obtainPreResult(resultSoftShape);
	prevTiles = resultSoftShape;

}

bool wireReduceSA(Plane& chip, vector<Tile>& currentSoft, vector<int>& initChosen, double initTmp, double endTmp, \
	double areaWeight, int tryTime, int adjsize, void(*operatorFunc)(vector<int>, vector<Tile>&, Plane&))
{
	vector<Tile> newSoft;
	newSoft.resize(currentSoft.size());
	vector<int> chosen;
	if (adjsize > 0)
		chosen.resize(adjsize);
	if (initChosen.size() < adjsize)
		chosen.resize(1);
	if (tryTime > 1000 * 32)
		tryTime = 32000;


	//init
	double requiredArea = 0;
	for (int i = 0; i < initChosen.size(); i++)
		requiredArea += chip.getSoft(i)->getMinArea();
	double preHPWL = chip.all_HPWL();
	double preArea = 1;
	bool success = 0;
	for (int idx = 0; idx < initChosen.size(); idx++)
		preArea *= min({ double(chip.getSoft(idx)->getCurArea()) / chip.getSoft(idx)->getMinArea(),1.0 });
	preArea = pow(preArea, 1.0 / double(currentSoft.size()));

	//prestate
	double initVal = preHPWL;
	double trueBest = initVal;
	bool currentlyLegal = chip.checkAllLegal() == 0;


	//SA
	double Tmp = initTmp;
	cout << preArea << "	" << preHPWL << endl;
	int cnt = -1;
	while (Tmp > endTmp)
	{
		preHPWL = chip.all_HPWL();
		preArea = 0;
		for (int idx = 0; idx < initChosen.size(); idx++)
			preArea += min({ double(chip.getSoft(idx)->getCurArea()) / chip.getSoft(idx)->getMinArea(),1.0 });
		for (int count = 0; count < tryTime; count++)
		{
			newSoft = currentSoft;
			if (adjsize < 0)
				chosen.resize(rand() % initChosen.size());
			randomize<int>(initChosen);
			for (size_t j = 0; j < chosen.size(); j++)
				chosen[j] = initChosen[j];

			operatorFunc(chosen, newSoft, chip);

			double avgOccupied = 0;
			for (int idx = 0; idx < initChosen.size(); idx++)
				avgOccupied += min({ double(chip.getSoft(idx)->getCurArea()) / chip.getSoft(idx)->getMinArea(),1.0 });

			double D = areaWeight * (1.0 / avgOccupied - 1.0 / preArea) / (1.0 / preArea) + (1.0 - areaWeight) * (chip.all_HPWL() - preHPWL) / preHPWL;
			if (D < 0 || double(rand() % 100) / 100.0 < exp(-1 * D * hightestTmp / Tmp))
			{

				currentSoft = newSoft;
				if ((chip.all_HPWL() < bestHPWL) && chip.checkAllLegal() == 0)
				{
					//					cout << "Update" << endl;
						//				cout << avgOccupied << "	" << chip.all_HPWL() << "	" << Tmp << endl;
					resultSoft = currentSoft;
					bestHPWL = chip.all_HPWL();
					writeCSV(chip, Tmp, cnt);
				}
				if (chip.checkAllLegal() == 0)
					success = 1;
				preArea = avgOccupied;
				preHPWL = chip.all_HPWL();
			}
			else
			{
				for (int& idx : chosen)
				{
					chip.ripoff(idx);
				}
				for (int& idx : chosen)
				{
					Tile* newTile = new Tile(&currentSoft[idx]);
					chip.insert(newTile);
					chip.getSoft(idx)->update(newTile);
				}
			}

		}
		cnt++;
		preArea = 0;
		Tmp = Tmp * Omega(Tmp);
	}
	if ((bestHPWL != initVal || currentlyLegal) && Tmp < 4)
	{
		currentSoft = resultSoft;
		chip.obtainPreResult(resultSoft);
	}
	return success;
}

#endif 