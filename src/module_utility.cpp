#include "module_extension.h"
using namespace std;

ostream& operator<<(ostream& fout,Pin p)
{
    fout<<p.pin_type<<" ("<<p.relative_loc<<")  ";
    return fout;
}
