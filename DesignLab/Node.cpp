#include "node.h"

#include <iostream>

#include "designlab_math_util.h"
#include "hexapod_state_calculator.h"
#include "leg_state.h"
#include "map_const.h"

namespace dlm = designlab::math_util;


SNode::SNode() :
	leg_state(0)
{
	leg_state = 0;	//脚状態を初期化する

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		leg_pos[i] = designlab::Vector3{ 0, 0, 0 };
		leg_base_pos[i] = designlab::Vector3{ 0, 0, 0 };
	}

	global_center_of_mass = designlab::Vector3{ 0, 0, 0 };
	rot = designlab::EulerXYZ{ 0, 0, 0 };

	next_move = HexapodMove::kComUpDown;
	parent_num = -1;
	depth = 0;
}


void SNode::Init(const bool do_random)
{
	using designlab::Vector3;
	using designlab::EulerXYZ;

	//脚状態
	leg_state = dl_leg::MakeLegStateBit(
		EDiscreteComPos::CENTER_BACK, 
		{ true, true, true, true, true, true },
		{ DiscreteLegPos::kCenter, DiscreteLegPos::kCenter, DiscreteLegPos::kCenter,
		DiscreteLegPos::kCenter, DiscreteLegPos::kCenter, DiscreteLegPos::kCenter }
	);

	//脚付け根を原点とした，脚先の位置を初期化する．
	const float kComZ = HexapodConst::VERTICAL_MIN_RANGE + MapConst::MAX_Z_BASE;	// ロボットの重心のZ座標

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		leg_pos[i] = leg_base_pos[i] = HexapodStateCalclator_Old::getLocalBaseLegPos(i, -kComZ);
	}

	//グローバル座標の重心位置．グローバル座標(0,0,0)を中心とした，下の変数 _x，_yを半径とする楕円形のなかに重心を移動する．
	const float kAngle = do_random ? dlm::GenerateRandomNumber(0.0f, 2.0f * dlm::kFloatPi) : 0;
	const float kEx = do_random ? dlm::GenerateRandomNumber(0.0f, 1.0f) : 0;

	const float kX = ((float)MapConst::MAP_START_ROUGH - MapConst::MAP_MIN_FORWARD) * 0.25f;
	const float kY = ((float)MapConst::MAP_MAX_HORIZONTAL - MapConst::MAP_MIN_HORIZONTAL) / 2.0f * 0.8f;

	global_center_of_mass = designlab::Vector3(kEx * kX * cos(kAngle), kEx * kY * sin(kAngle), kComZ);

	//ロールピッチヨーで回転を表現する．ロボットの重心を中心にして回転する． 
	rot = designlab::EulerXYZ(0, 0, 0);

	next_move = HexapodMove::kComUpDown;
	parent_num = -1;
	depth = 0;
}


void SNode::ChangeGlobalCenterOfMass(const designlab::Vector3& new_com, const bool base)
{
	const designlab::Vector3 kComDif = new_com - global_center_of_mass;

	global_center_of_mass = new_com;

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		if (dl_leg::IsGrounded(leg_state, i))
		{
			leg_pos[i] -= kComDif;
			leg_base_pos[i] = leg_pos[i];
		}
		else
		{
			if (base)leg_base_pos[i] = HexapodStateCalclator_Old::getLocalBaseLegPos(i, leg_base_pos[i].z - kComDif.z);
		}
	}

}

std::string SNode::ToString() const
{
	// \t はタブを表す文字．また，スペースのみを追加したい場合は std::string(" ") とする．

	std::string res;	//結果として返す文字列

	//脚状態のbit列
	res += "Leg State Bit : " + leg_state.to_string() + "\n";
	res += "\n";

	//重心位置の出力
	EDiscreteComPos com = dl_leg::getComPatternState(leg_state);
	res += "Com Pattern : " + std::to_string(com) + "(" + std::to_string(static_cast<int>(com)) +")\n";

	//脚の接地状態
	res += "Ground : ";

	for (int i = 0; i < HexapodConst::LEG_NUM; ++i)
	{
		res += (dl_leg::IsGrounded(leg_state, i) ? "ground" : "lifted") + std::string(" ");
	}

	res += "\n";

	//脚の階層
	res += "Hierarchy : ";

	for (int i = 0; i < HexapodConst::LEG_NUM; ++i)
	{
		DiscreteLegPos dis_leg_pos = dl_leg::getLegState(leg_state, i);
		res += std::to_string(dis_leg_pos) + "(" + std::to_string(static_cast<int>(dis_leg_pos)) + ") ";
	}

	res += "\n\n";

	//脚位置
	res += "Leg Position : \n";

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		res += "\t" + std::to_string(i) + ":" + leg_pos[i].ToString() + "\n";
	}

	res += "Leg Base Position : \n";

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		res += "\t" + std::to_string(i) + ":" + leg_base_pos[i].ToString() + "\n";
	}

	//重心位置
	res += "\nGlobal Center of Mass : " + global_center_of_mass.ToString() + "\n";

	//回転姿勢
	res += "Rotate  : " + rot.ToString() + "\n";
	res += "(Rotate : " + rot.ToStringDeg() + ")\n";

	//次動作
	res += "\n";
	res += "(Next Move : " + std::to_string(next_move) + std::string(")\n");
	res += "(Depth : " + std::to_string(depth) + std::string(")\n");
	res += "(parent number : " + std::to_string(parent_num) + std::string(")\n");

	return res;
}


std::ostream& operator<<(std::ostream& stream, const SNode& value)
{
	const int kPrecision = 3;


	//遊脚・接地脚の出力
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		//boolalphaを指定すると，1,0 でなく，true,falseで出力される．
		stream << std::boolalpha << dl_leg::IsGrounded(value.leg_state, i) << ",";
	}

	//階層の出力
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		stream << static_cast<int>(dl_leg::getLegState(value.leg_state, i)) << ",";
	}

	//重心位置の出力
	stream << static_cast<int>(dl_leg::getComPatternState(value.leg_state)) << ",";

	//姿勢の出力
	stream << std::fixed << std::setprecision(kPrecision) << value.rot.y_angle << "," << value.rot.x_angle << "," << value.rot.z_angle << ",";

	//脚先位置
	for (int i = 0; i < HexapodConst::LEG_NUM; ++i)
	{
		stream << std::fixed << std::setprecision(kPrecision) << value.leg_pos[i].x << "," << value.leg_pos[i].y << "," << value.leg_pos[i].z << ",";
	}

	//接地予定位置
	for (int i = 0; i < HexapodConst::LEG_NUM; ++i)
	{
		stream << std::fixed << std::setprecision(kPrecision) << value.leg_base_pos[i].x << "," << value.leg_base_pos[i].y << "," << value.leg_base_pos[i].z << ",";
	}

	//重心位置
	stream << std::fixed << std::setprecision(kPrecision) << value.global_center_of_mass.x << "," << value.global_center_of_mass.y << "," << value.global_center_of_mass.z << ",";

	//次の動作
	stream << std::to_string(value.next_move);

	return stream;
}
