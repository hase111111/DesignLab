#include "Node.h"
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

	if (node1.leg_state == node2.leg_state && my_vec::isEqualVector(node1.global_center_of_mass, node2.global_center_of_mass, 1.0f) == true)
	{
		for (int i = 0; i < HexapodConst::LEG_NUM; ++i)
		{
			if (my_vec::isEqualVector(node1.leg_pos[i], node2.leg_pos[i], 1.0f) == false)
			{
				return false;
			}
		}

		return true;
	}

	return false;
}


SNode::SNode()
{
	leg_state = 0;

	leg_pos[0] = my_vec::VGet(0, 0, 0);
	leg_pos[1] = my_vec::VGet(0, 0, 0);
	leg_pos[2] = my_vec::VGet(0, 0, 0);
	leg_pos[3] = my_vec::VGet(0, 0, 0);
	leg_pos[4] = my_vec::VGet(0, 0, 0);
	leg_pos[5] = my_vec::VGet(0, 0, 0);

	Leg2[0] = my_vec::VGet(0, 0, 0);
	Leg2[1] = my_vec::VGet(0, 0, 0);
	Leg2[2] = my_vec::VGet(0, 0, 0);
	Leg2[3] = my_vec::VGet(0, 0, 0);
	Leg2[4] = my_vec::VGet(0, 0, 0);
	Leg2[5] = my_vec::VGet(0, 0, 0);

	global_center_of_mass = my_vec::VGet(0, 0, 0);

	rot.roll = rot.pitch = rot.yaw = 0;

	roll = 0.0f;	pitch = 0.0f;	yaw = 0.0f;

	next_move = EHexapodMove::COM_UP_DOWN;
	parent_num = -1;
	depth = 0;

	parent = nullptr;		//親ノードのポインタ
	node_height = 1;		//ノード高さ
	debug = 24;			//現在運動履歴として使用,前回の脚上下ノード(上下運動をした場合)2桁,前回の動作1桁,前々回の動作1桁

	last_node_num = 0;
	time = 0;

	delta_comz = 0.0f;
	target_delta_comz = 0;
}

SNode::SNode(const SNode& _other)
{
	leg_state = _other.leg_state;

	leg_pos[0] = _other.leg_pos[0];
	leg_pos[1] = _other.leg_pos[1];
	leg_pos[2] = _other.leg_pos[2];
	leg_pos[3] = _other.leg_pos[3];
	leg_pos[4] = _other.leg_pos[4];
	leg_pos[5] = _other.leg_pos[5];

	Leg2[0] = _other.Leg2[0];
	Leg2[1] = _other.Leg2[1];
	Leg2[2] = _other.Leg2[2];
	Leg2[3] = _other.Leg2[3];
	Leg2[4] = _other.Leg2[4];
	Leg2[5] = _other.Leg2[5];

	global_center_of_mass = _other.global_center_of_mass;

	rot.roll	= _other.rot.roll;
	rot.pitch = _other.rot.pitch;
	rot.yaw		= _other.rot.yaw;

	roll = _other.roll;
	pitch = _other.pitch;
	yaw = _other.yaw;

	next_move = _other.next_move;
	parent_num = _other.parent_num;
	depth = _other.depth;

	parent = _other.parent;				//親ノードのポインタ
	node_height = _other.node_height;	//ノード高さ
	debug = _other.debug;				//現在運動履歴として使用,前回の脚上下ノード(上下運動をした場合)2桁,前回の動作1桁,前々回の動作1桁

	last_node_num = _other.last_node_num;
	time = _other.time;

	delta_comz = _other.delta_comz;
	target_delta_comz = _other.target_delta_comz;
}

SNode& SNode::operator=(const SNode& _other)
{
	leg_state = _other.leg_state;

	leg_pos[0] = _other.leg_pos[0];
	leg_pos[1] = _other.leg_pos[1];
	leg_pos[2] = _other.leg_pos[2];
	leg_pos[3] = _other.leg_pos[3];
	leg_pos[4] = _other.leg_pos[4];
	leg_pos[5] = _other.leg_pos[5];

	Leg2[0] = _other.Leg2[0];
	Leg2[1] = _other.Leg2[1];
	Leg2[2] = _other.Leg2[2];
	Leg2[3] = _other.Leg2[3];
	Leg2[4] = _other.Leg2[4];
	Leg2[5] = _other.Leg2[5];

	global_center_of_mass = _other.global_center_of_mass;

	rot.roll = _other.rot.roll;
	rot.pitch = _other.rot.pitch;
	rot.yaw = _other.rot.yaw;

	roll = _other.roll;
	pitch = _other.pitch;
	yaw = _other.yaw;

	next_move = _other.next_move;
	parent_num = _other.parent_num;
	depth = _other.depth;

	parent = _other.parent;				//親ノードのポインタ
	node_height = _other.node_height;	//ノード高さ
	debug = _other.debug;				//現在運動履歴として使用,前回の脚上下ノード(上下運動をした場合)2桁,前回の動作1桁,前々回の動作1桁

	last_node_num = _other.last_node_num;
	time = _other.time;

	delta_comz = _other.delta_comz;
	target_delta_comz = _other.target_delta_comz;

	return *this;
}
