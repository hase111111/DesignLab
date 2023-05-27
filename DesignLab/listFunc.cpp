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

	if (node1.leg_state == node2.leg_state && myvector::isEqualVector(node1.global_center_of_mass, node2.global_center_of_mass, 1.0f) == true)
	{
		for (int i = 0; i < HexapodConst::LEG_NUM; ++i)
		{
			if (myvector::isEqualVector(node1.Leg[i], node2.Leg[i], 1.0f) == false)
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
	const float COM_Z = 130.0f + MapConst::MAX_Z_BASE;	// ロボットの重心のZ座標

	const float _angle = _do_random ? my_math::generateRandomNumber(0.0f, 2.0f * Define::MY_PI) : 0;
	const float _ex = _do_random ? my_math::generateRandomNumber(0.0f, 1.0f) : 0;

	const float _x = ((float)MapConst::MAP_X_MAX - MapConst::MAP_X_MIN) / 2.0f * 0.8f;
	const float _y = ((float)MapConst::START_ROUGH_TARRAIN_Y - MapConst::MAP_Y_MIN) * 0.25f;

	//脚状態
	_node.leg_state = 0b00000000110011001100110011001100;

	//付け根から脚先の位置
	_node.Leg[0] = myvector::VGet(120.0f ,  100.0f, -COM_Z);
	_node.Leg[1] = myvector::VGet(130.0f ,    0.0f, -COM_Z);
	_node.Leg[2] = myvector::VGet(120.0f , -100.0f, -COM_Z);
	_node.Leg[3] = myvector::VGet(-120.0f, -100.0f, -COM_Z);
	_node.Leg[4] = myvector::VGet(-130.0f,    0.0f, -COM_Z);
	_node.Leg[5] = myvector::VGet(-120.0f,  100.0f, -COM_Z);

	//脚の位置(z方向固定)
	_node.Leg2[0] = myvector::VGet( 120.0f,  100.0f, -COM_Z);
	_node.Leg2[1] = myvector::VGet( 130.0f,    0.0f, -COM_Z);
	_node.Leg2[2] = myvector::VGet( 120.0f, -100.0f, -COM_Z);
	_node.Leg2[3] = myvector::VGet(-120.0f, -100.0f, -COM_Z);
	_node.Leg2[4] = myvector::VGet(-130.0f,    0.0f, -COM_Z);
	_node.Leg2[5] = myvector::VGet(-120.0f,  100.0f, -COM_Z);

	//重心位置グローバル
	_node.global_center_of_mass = myvector::VGet(_ex * _x * cos(_angle), _ex * _y * sin(_angle), COM_Z);

	//ロールピッチヨーで回転を表現する．ロボットの重心を中心にして回転する． https://watako-lab.com/2019/01/23/roll_pitch_yaw/
	_node.roll = 0.0f;			// x軸回転
	_node.pitch = 0.0f;			// y軸回転
	_node.yaw = 0.0f;			// z軸回転

	_node.next_move = EHexapodMove::COM_UP_DOWN;
	_node.parent_num = -1;
	_node.depth = 0;

	_node.parent = nullptr;		//親ノードのポインタ
	_node.node_height = 1;		//ノード高さ
	_node.debug = 24;			//現在運動履歴として使用,前回の脚上下ノード(上下運動をした場合)2桁,前回の動作1桁,前々回の動作1桁

	_node.last_node_num = 0;
	_node.time = 0;

	_node.delta_comz = 0.0f;
	_node.target_delta_comz = 0;
}
