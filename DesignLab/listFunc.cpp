#include "listFunc.h"
#include <iostream>


int GetRouteDepth(LNODE* My_p) 
{
	int Depth = 0;

	while (My_p->parent != NULL) 
	{
		Depth++;
		My_p = My_p->parent;
	}

	return Depth;
}

int GetRoute(LNODE* My_p, LNODE** Root2My)
{
	int Depth; //深さ
	if (My_p -> parent != NULL)
	{					//parentがNULLでない∴根ノードでなかったら
		Depth = 1+GetRoute(My_p -> parent, Root2My);//深さに1を足す
		Root2My[Depth] = My_p;						//自身のポインタを格納 Root2My[自身の深さ] 
		return Depth;
	}
	else
	{											//parentがNULL∴根ノードだったら
		Depth = 0;
		Root2My[Depth] = My_p;						//Root2My[0]
		return Depth;
	}
}

bool isLNODEEqual(const LNODE& node1, const LNODE& node2)
{
	// isEqualVectorの許容誤差の値が 1である理由は分からない．もともとマジックナンバーだった．

	if (node1.leg_state == node2.leg_state && myvector::isEqualVector(node1.global_center_of_mass, node2.global_center_of_mass, 1.0) == true)
	{
		for (int i = 0; i < Define::LEG_NUM; ++i)
		{
			if (myvector::isEqualVector(node1.Leg[i], node2.Leg[i], 1.0) == false)
			{
				return false;
			}
		}

		return true;
	}

	return false;
}
