#include "listFunc.h"
#include <iostream>
#include "MapConst.h"
#include "MyMath.h"

int GetRouteDepth(SNode* My_p) 
{
	int Depth = 0;

	while (My_p->parent != NULL) 
	{
		Depth++;
		My_p = My_p->parent;
	}

	return Depth;
}

int GetRoute(SNode* My_p, SNode** Root2My)
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

bool isNodeEqual(const SNode& node1, const SNode& node2)
{
	// isEqualVectorの許容誤差の値が 1である理由は分からない．もともとマジックナンバーだった．

	if (node1.leg_state == node2.leg_state && myvector::isEqualVector(node1.global_center_of_mass, node2.global_center_of_mass, 1.0) == true)
	{
		for (int i = 0; i < HexapodConst::LEG_NUM; ++i)
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

void initNode(SNode& _node, const bool _do_random)
{
	const double COM_Z = 130 + MapConst::MAX_Z_BASE;	// ロボットの重心のZ座標

	const double _angle = my_math::generateRandomNumber(0.0, 2.0 * Define::MY_PI);
	const double _r = my_math::generateRandomNumber(0.0, MapConst::START_RANDOM_R);

	//重心位置グローバル
	_node.global_center_of_mass = myvector::VGet(_r * cos(_angle), _r * sin(_angle), COM_Z);

	//付け根から脚先の位置
	_node.Leg[0] = myvector::VGet(120, 100, -COM_Z);
	_node.Leg[1] = myvector::VGet(130, 0, -COM_Z);
	_node.Leg[2] = myvector::VGet(120, -100, -COM_Z);
	_node.Leg[3] = myvector::VGet(-120, -100, -COM_Z);
	_node.Leg[4] = myvector::VGet(-130, 0, -COM_Z);
	_node.Leg[5] = myvector::VGet(-120, 100, -COM_Z);

	//脚の位置(z方向固定)
	_node.Leg2[0] = myvector::VGet(120, 100, -COM_Z);
	_node.Leg2[1] = myvector::VGet(130, 0, -COM_Z);
	_node.Leg2[2] = myvector::VGet(120, -100, -COM_Z);
	_node.Leg2[3] = myvector::VGet(-120, -100, -COM_Z);
	_node.Leg2[4] = myvector::VGet(-130, 0, -COM_Z);
	_node.Leg2[5] = myvector::VGet(-120, 100, -COM_Z);

	//姿勢テイトブライアン角グローバル
	_node.pitch = 0.0;			//x軸回転
	_node.roll = 0.0;			//y軸回転
	_node.yaw = 0.0;			//z軸回転
	_node.leg_state = 0b00000000110011001100110011001100;
	_node.parent = NULL;		//親ノードのポインタ
	_node.node_height = 1;		//ノード高さ
	_node.debug = 24;			//現在運動履歴として使用,前回の脚上下ノード(上下運動をした場合)2桁,前回の動作1桁,前々回の動作1桁
	_node.delta_comz = 0;
}
