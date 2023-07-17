#include "Node.h"
#include <iostream>
#include "MapConst.h"
#include "MyMath.h"
#include "LegState.h"

std::ostream& operator<<(std::ostream& stream, const SNode& value)
{
	//重心パターン
	stream << "COM_type = " << LegStateEdit::getComPatternState(value.leg_state) << std::endl;
	stream << std::endl;

	//脚の遊脚・接地状態
	stream << "Legs(0,1,2,3,4,5)" << std::endl;
	stream << "Ground : ";
	for (int i = 0; i < HexapodConst::LEG_NUM; ++i) { stream << (LegStateEdit::isGrounded(value.leg_state, i) ? "ground" : "lifted") << " "; }
	stream << std::endl;

	//脚の階層
	stream << "Hierarchy : ";
	for (int i = 0; i < HexapodConst::LEG_NUM; ++i) { stream << LegStateEdit::getLegState(value.leg_state, i) << " "; }
	stream << std::endl;

	//脚位置
	stream << "Leg Position : " << std::endl;
	stream << std::fixed;
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		stream << "\t" << i << ":";
		stream << value.leg_pos[i] << "\t";
		stream << "Leg2 :";
		stream << value.Leg2[i] << std::endl;
	}
	stream << std::endl;

	//重心位置
	stream << "global_center_of_mass = " << value.global_center_of_mass << std::endl;

	//回転姿勢
	stream << "Rotate : " << value.rot << std::endl;

	//次動作
	stream << std::endl;
	stream << "(Next Move : " << std::to_string(value.next_move) << ")" << std::endl;
	stream << "(Depth : " << static_cast<int>(value.depth) << ")" << std::endl;
	stream << "(parent number : " << value.parent_num << ")" << std::endl;

	return stream;
}

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
	if (My_p->parent != NULL)
	{					//parentがNULLでない∴根ノードでなかったら
		Depth = 1 + GetRoute(My_p->parent, Root2My);//深さに1を足す
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
	using my_vec::SVector;
	using my_vec::SRotator;

	leg_state = 0;	//脚状態を初期化する

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		leg_pos[i] = SVector(0, 0, 0);
		Leg2[i] = SVector(0, 0, 0);
	}

	global_center_of_mass = SVector(0, 0, 0);
	rot = SRotator(0, 0, 0);

	next_move = EHexapodMove::COM_UP_DOWN;
	parent_num = -1;
	depth = 0;

	parent = nullptr;
	node_height = 1;
	debug = 24;
	last_node_num = 0;
	time = 0;
	roll = pitch = yaw = 0.0f;
	delta_comz = 0.0f;
	target_delta_comz = 0;
}

SNode::SNode(const SNode& _other)
{
	leg_state = _other.leg_state;

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		leg_pos[i] = _other.leg_pos[i];
		Leg2[i] = _other.Leg2[i];
	}

	global_center_of_mass = _other.global_center_of_mass;
	rot = _other.rot;

	next_move = _other.next_move;
	parent_num = _other.parent_num;
	depth = _other.depth;


	//現在未使用の変数．後で消します
	parent = _other.parent;
	node_height = _other.node_height;
	debug = _other.debug;
	last_node_num = _other.last_node_num;
	time = _other.time;
	roll = _other.roll;
	pitch = _other.pitch;
	yaw = _other.yaw;
	delta_comz = _other.delta_comz;
	target_delta_comz = _other.target_delta_comz;
}

void SNode::init(const bool _do_random)
{
	using my_vec::SVector;
	using my_vec::SRotator;

	//脚状態
	bool _leg_ground[HexapodConst::LEG_NUM] = { true, true, true, true, true, true };
	int _leg_pos[HexapodConst::LEG_NUM] = { 4, 4, 4, 4, 4, 4 };
	leg_state = LegStateEdit::makeLegState(ComType::EComPattern::center_back, _leg_ground, _leg_pos);

	//脚付け根を原点とした，脚先の位置を初期化する．
	const float _com_z = HexapodConst::VERTICAL_MIN_RANGE + MapConst::MAX_Z_BASE;	// ロボットの重心のZ座標

	leg_pos[0] = Leg2[0] = SVector(100.0f, -120.0f, -_com_z);
	leg_pos[1] = Leg2[1] = SVector(0.0f, -130.0f, -_com_z);
	leg_pos[2] = Leg2[2] = SVector(-100.0f, -120.0f, -_com_z);
	leg_pos[3] = Leg2[3] = SVector(-100.0f, 120.0f, -_com_z);
	leg_pos[4] = Leg2[4] = SVector(0.0f, 130.0f, -_com_z);
	leg_pos[5] = Leg2[5] = SVector(100.0f, 120.0f, -_com_z);

	//グローバル座標の重心位置．グローバル座標(0,0,0)を中心とした，下の変数 _x，_yを半径とする楕円形のなかに重心を移動する．
	const float _angle = _do_random ? my_math::generateRandomNumber(0.0f, 2.0f * my_math::MY_FLT_PI) : 0;
	const float _ex = _do_random ? my_math::generateRandomNumber(0.0f, 1.0f) : 0;

	const float _x = ((float)MapConst::MAP_START_ROUGH - MapConst::MAP_MIN_FORWARD) * 0.25f;
	const float _y = ((float)MapConst::MAP_MAX_HORIZONTAL - MapConst::MAP_MIN_HORIZONTAL) / 2.0f * 0.8f;

	global_center_of_mass = my_vec::SVector(_ex * _x * cos(_angle), _ex * _y * sin(_angle), _com_z);

	//ロールピッチヨーで回転を表現する．ロボットの重心を中心にして回転する． 
	rot = my_vec::SRotator(0, 0, 0);

	next_move = EHexapodMove::LEG_HIERARCHY_CHANGE;
	parent_num = -1;
	depth = 0;

	//以下，もう使う気がないパラメータ．
	parent = nullptr;		//親ノードのポインタ
	node_height = 1;		//ノード高さ
	debug = 24;			//現在運動履歴として使用,前回の脚上下ノード(上下運動をした場合)2桁,前回の動作1桁,前々回の動作1桁
	last_node_num = 0;
	time = 0;
	roll = pitch = yaw = 0.0f;
	delta_comz = 0.0f;
	target_delta_comz = 0;
}

void SNode::changeGlobalCenterOfMass(const my_vec::SVector& _new_com)
{
	using my_vec::SVector;
	using LegStateEdit::isGrounded;

	const SVector _delta_com = _new_com - global_center_of_mass;

	global_center_of_mass = _new_com;

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		if (isGrounded(leg_state, i) == true)
		{
			leg_pos[i] -= _delta_com;
			Leg2[i] -= _delta_com;
		}
	}
}
