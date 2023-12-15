#include <iostream>
#include "phydb/phydb.h"

using namespace phydb;
using namespace std;

int main(int argc, char **argv) {
	PhyDB *db = new PhyDB();
	string defname = "./routed.def";
	string lefname = "./output.lef";

	db->ReadLef(lefname);
	db->ReadDef(defname);

	db->GenerateRCNetwork();
	db->PrintRCNetwork(std::cout);
}
