#include "Node.h"
#include <iostream>
#include "map_const.h"
#include "my_math.h"
#include "LegState.h"
#include "HexapodStateCalculator.h"

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
		stream << value.leg_pos[i];
		stream << " base_pos :";
		stream << value.leg_base_pos[i] << std::endl;
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

SNode::SNode()
{
	using my_vec::SVector;
	using my_vec::SRotator;

	leg_state = 0;	//脚状態を初期化する

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		leg_pos[i] = SVector(0, 0, 0);
		leg_base_pos[i] = SVector(0, 0, 0);
	}

	global_center_of_mass = SVector(0, 0, 0);
	rot = SRotator(0, 0, 0);

	next_move = EHexapodMove::COM_UP_DOWN;
	parent_num = -1;
	depth = 0;
}

SNode::SNode(const SNode& _other)
{
	leg_state = _other.leg_state;

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		leg_pos[i] = _other.leg_pos[i];
		leg_base_pos[i] = _other.leg_base_pos[i];
	}

	global_center_of_mass = _other.global_center_of_mass;
	rot = _other.rot;

	next_move = _other.next_move;
	parent_num = _other.parent_num;
	depth = _other.depth;
}

void SNode::init(const bool _do_random)
{
	using my_vec::SVector;
	using my_vec::SRotator;

	//脚状態
	bool _leg_ground[HexapodConst::LEG_NUM] = { true, true, true, true, true, true };
	int _leg_pos[HexapodConst::LEG_NUM] = { 4, 4, 4, 4, 4, 4 };
	leg_state = LegStateEdit::makeLegState(ComType::EComPattern::CENTER_BACK, _leg_ground, _leg_pos);

	//脚付け根を原点とした，脚先の位置を初期化する．
	const float _com_z = HexapodConst::VERTICAL_MIN_RANGE + MapConst::MAX_Z_BASE;	// ロボットの重心のZ座標

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		leg_pos[i] = leg_base_pos[i] = HexapodStateCalclator::getLocalBaseLegPos(i, -_com_z);
	}

	//グローバル座標の重心位置．グローバル座標(0,0,0)を中心とした，下の変数 _x，_yを半径とする楕円形のなかに重心を移動する．
	const float _angle = _do_random ? my_math::generateRandomNumber(0.0f, 2.0f * my_math::MY_FLT_PI) : 0;
	const float _ex = _do_random ? my_math::generateRandomNumber(0.0f, 1.0f) : 0;

	const float _x = ((float)MapConst::MAP_START_ROUGH - MapConst::MAP_MIN_FORWARD) * 0.25f;
	const float _y = ((float)MapConst::MAP_MAX_HORIZONTAL - MapConst::MAP_MIN_HORIZONTAL) / 2.0f * 0.8f;

	global_center_of_mass = my_vec::SVector(_ex * _x * cos(_angle), _ex * _y * sin(_angle), _com_z);

	//ロールピッチヨーで回転を表現する．ロボットの重心を中心にして回転する． 
	rot = my_vec::SRotator(0, 0, 0);

	next_move = EHexapodMove::COM_UP_DOWN;
	parent_num = -1;
	depth = 0;
}

void SNode::changeGlobalCenterOfMass(const my_vec::SVector& new_com, const bool base)
{
	const my_vec::SVector delta_com = new_com - global_center_of_mass;

	global_center_of_mass = new_com;

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		if (LegStateEdit::isGrounded(leg_state, i) == true)
		{
			leg_pos[i] -= delta_com;
			leg_base_pos[i] = leg_pos[i];
		}
		else
		{
			if (base)leg_base_pos[i] = HexapodStateCalclator::getLocalBaseLegPos(i, leg_base_pos[i].z - delta_com.z);
		}
	}

}
