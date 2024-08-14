#include<fstream>
#include<iostream>
#include<sstream>
#include<vector>

struct Tile;
class Soft_Module;
struct Wire;
class PlacerInOut
{
	private:
		std::vector<Tile*> inserted;
		std::vector<Soft_Module*> toInsert;
		std::vector<Wire*> wires;
		int chipX = 0;
		int chipY = 0;
		int minHeight = 9999999;
	public:
		void readInsertedTile(std::vector<Tile*>);
		void readWire(std::vector<Wire*>);
		void readtoInsertTile(std::vector<Soft_Module*>);
		void setY(int);
		void setX(int);

		void generateData();
		void pl();
		void wts();
		void scl();
		void nodes();
		void nets();
};