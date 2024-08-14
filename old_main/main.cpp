#include"FlowFunc.hpp"
#include <omp.h>
#include <thread>
using namespace std;
double start = clock();

double Omega(double);

void extremeCase(char* argv[])
{
	Plane chip(argv[1]);
	for (int i = 0; i < chip.fixedModule_size(); i++)
		chip.insert_fix(i);

	vector<Tile> currentSoft;
	currentSoft.resize(chip.softModule_size(), Tile(Point(0, 0), 0, 0));

	vector<int> chosen;
	chosen.resize(chip.softModule_size());
	for (int i = 0; i < chosen.size(); i++)
		chosen[i] = i;

	while (1)
	{
		double score = 2;
		generateSeedGreedy(chip, currentSoft, chosen, score);
		FindBetterPlaceAP(chip, currentSoft, chosen, score);
		fillInStage(chip, chosen);
		handlingFail(chip, chosen, score);
		if (chip.checkAllLegal() == 0)
			break;
	}

	chip.outimg();
	chip.output(argv[2]);
}

int main(int argc, char* argv[])
{
	

	Plane chip(argv[1]);
	vector<Tile> currentSoft;
	vector<int> chosen;
	double score;
	string CSVname = argv[1];
	CSVname = CSVname + +".csv";
	file.open(CSVname);

	Initializing(chip, currentSoft, chosen, score);
	if (chip.softModule_size() == 15)
		srand(12);
	else if (chip.softModule_size() == 16 && chip.getHeight() == 2300)
		srand(46);
	else if (chip.softModule_size() == 28)
		srand(20);
	else if (chip.softModule_size() == 20)
		srand(32);
	else if (chip.softModule_size() == 16)
		srand(17);
	else if (chip.softModule_size() == 21)
		srand(11);
	else
		srand(3);


	vector<double> weightRatio = { 0.65 ,0.65 ,0.6 ,0.6 ,0.6 ,0.6 ,0.6 ,0.65 };
	vector<double> TmpSet = { 1000 ,200,20,5,1,0.5,0.2,0.08,0.01 };
	generateSeedGreedy(chip, currentSoft, chosen, score);
	
	wireReduceSA(chip, currentSoft, chosen, 60000, 10000, 0.2, 3 * currentSoft.size(), 2, randBase);
	
	wireReduceSA(chip, currentSoft, chosen, 10000, 1000, 0.2, 3 * currentSoft.size(), 2, randBase);
	wireReduceSA(chip, currentSoft, chosen, TmpSet[0], TmpSet[1], weightRatio[0], 3 * currentSoft.size(), 2, randBase);
	int failTime = 0;
	bool successBefore = 0;
	int tryTime = 3 * chip.softModule_size();
	for (int i = 1; i < 8; i++)
	{
		bool UpdateSuccess = 0;
		UpdateSuccess = wireReduceSA(chip, currentSoft, chosen, TmpSet[i], TmpSet[i + 1], weightRatio[i], tryTime, 2, randBase);
		if (UpdateSuccess == 1)
			successBefore = 1;

		if (!UpdateSuccess)
		{
			for (int j = i; j < 8; j++)
				weightRatio[j] = (weightRatio[j] + 1.0) / 2;
			i--;
		}
		if ((clock() - start) / 1000000.0 > 1300)
		{
			cout << "REDUCE TRYTIME" << endl;
			tryTime = 100 * chip.softModule_size();
		}
		if ((clock() - start) / 1000000.0 > 1500 && successBefore)
		{
			cout << "TAKE TOO LONG TIME" << endl;
			break;
		}
		if ((clock() - start) / 1000000.0 > 1500 && !successBefore)
		{
			extremeCase(argv);
			exit(0);
		}
	}
	//		cout << "Set function to movement" << endl;
	if(tryTime == 1000 * chip.softModule_size())
		wireReduceSA(chip, currentSoft, chosen, 0.08, 0.01, 0, 3 * currentSoft.size(), -1, moveBase);
	else
		wireReduceSA(chip, currentSoft, chosen, 0.08, 0.01, 0, 3 * currentSoft.size(), -1, moveBase);
	
	if (chip.checkAllLegal() == 0)
	{
		chip.output(argv[2]);
		chip.outimg();
	}
	double HPWL = chip.all_HPWL();
	cout << HPWL << endl;
	vector<vector<Tile>> prevTiles = chip.getAllTile();
	rectiLinearSA(chip, currentSoft, argv[2], doShrink, prevTiles);
	rectiLinearSA(chip, currentSoft, argv[2], !doShrink, prevTiles);
	for (int i = 0; i < currentSoft.size(); i++)
	{
		currentSoft[i] = Tile(chip.getSoft(i)->get_root());
		currentSoft[i].clearStitch();
	}
	bestHPWL = chip.all_HPWL();
	resultSoft = currentSoft;
	HPWL = chip.all_HPWL();


	if (chip.checkAllLegal() == 0)
		cout << "PASS!" << endl;

	for (int i = 0; i < chip.softModule_size(); i++)
		if (!chip.Legal(i))
			cout << chip.getSoft(i)->getName() + " " << height(chip.getSoft(i)) << "	" << width(chip.getSoft(i)) << endl;

	cout << HPWL << endl;
	cout << (clock() - start) / 100000 << endl;
	cout << endl;
	file.close();
	return 0;
}



