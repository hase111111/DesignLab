#include"vectorFunc.h"

void myvector::VectorOutPut(SVector v1, std::ofstream& fout) 
{
	fout << "( " << v1.x << "," << v1.y << "," << v1.z << " )\n";
}

void myvector::VectorOutPut(SVector v1) 
{
	std::cout << "( " << (int)v1.x << "," << (int)v1.y << "," << v1.z << " )\n";
}


bool myvector::VECTOREqual(const SVector& In1, const SVector& In2) 
{
	if ((abs(In1.x - In2.x) < 1) && (abs(In1.y - In2.y) < 1) && (abs(In1.z - In2.z) < 1)) 
	{
		return true;
	}

	return false;
}
