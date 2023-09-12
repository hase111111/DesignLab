#include "node.h"

#include <iostream>

#include "map_const.h"
#include "designlab_math.h"
#include "leg_state.h"
#include "hexapod_state_calculator.h"


SNode::SNode()
{
	leg_state = 0;	//脚状態を初期化する

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		leg_pos[i] = dl_vec::SVector{ 0, 0, 0 };
		leg_base_pos[i] = dl_vec::SVector{ 0, 0, 0 };
	}

	global_center_of_mass = dl_vec::SVector{ 0, 0, 0 };
	rot = dl_vec::SRotator{ 0, 0, 0 };

	next_move = EHexapodMove::COM_UP_DOWN;
	parent_num = -1;
	depth = 0;
}


SNode::SNode(const SNode& other)
{
	leg_state = other.leg_state;

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		leg_pos[i] = other.leg_pos[i];
		leg_base_pos[i] = other.leg_base_pos[i];
	}

	global_center_of_mass = other.global_center_of_mass;
	rot = other.rot;

	next_move = other.next_move;
	parent_num = other.parent_num;
	depth = other.depth;
}


void SNode::init(const bool do_random)
{
	using dl_vec::SVector;
	using dl_vec::SRotator;

	//脚状態
	const bool kLegGround[HexapodConst::LEG_NUM] = { true, true, true, true, true, true };
	const dl_leg::EDiscreteLegPos kLegPos[HexapodConst::LEG_NUM] = { dl_leg::EDiscreteLegPos::CENTER, dl_leg::EDiscreteLegPos::CENTER, dl_leg::EDiscreteLegPos::CENTER,
		dl_leg::EDiscreteLegPos::CENTER, dl_leg::EDiscreteLegPos::CENTER, dl_leg::EDiscreteLegPos::CENTER };

	leg_state = dl_leg::makeLegState(ComType::EComPattern::CENTER_BACK, kLegGround, kLegPos);

	//脚付け根を原点とした，脚先の位置を初期化する．
	const float kComZ = HexapodConst::VERTICAL_MIN_RANGE + MapConst::MAX_Z_BASE;	// ロボットの重心のZ座標

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		leg_pos[i] = leg_base_pos[i] = HexapodStateCalclator_Old::getLocalBaseLegPos(i, -kComZ);
	}

	//グローバル座標の重心位置．グローバル座標(0,0,0)を中心とした，下の変数 _x，_yを半径とする楕円形のなかに重心を移動する．
	const float kAngle = do_random ? dl_math::generateRandomNumber(0.0f, 2.0f * dl_math::MY_FLT_PI) : 0;
	const float kEx = do_random ? dl_math::generateRandomNumber(0.0f, 1.0f) : 0;

	const float kX = ((float)MapConst::MAP_START_ROUGH - MapConst::MAP_MIN_FORWARD) * 0.25f;
	const float kY = ((float)MapConst::MAP_MAX_HORIZONTAL - MapConst::MAP_MIN_HORIZONTAL) / 2.0f * 0.8f;

	global_center_of_mass = dl_vec::SVector(kEx * kX * cos(kAngle), kEx * kY * sin(kAngle), kComZ);

	//ロールピッチヨーで回転を表現する．ロボットの重心を中心にして回転する． 
	rot = dl_vec::SRotator(0, 0, 0);

	next_move = EHexapodMove::COM_UP_DOWN;
	parent_num = -1;
	depth = 0;
}


void SNode::changeGlobalCenterOfMass(const dl_vec::SVector& new_com, const bool base)
{
	const dl_vec::SVector kComDif = new_com - global_center_of_mass;

	global_center_of_mass = new_com;

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		if (dl_leg::isGrounded(leg_state, i))
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


std::ostream& operator<<(std::ostream& stream, const SNode& value)
{
	const int kPrecision = 3;


	//遊脚・接地脚の出力
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		//boolalphaを指定すると，1,0 でなく，true,falseで出力される．
		stream << std::boolalpha << dl_leg::isGrounded(value.leg_state, i) << ",";
	}

	//階層の出力
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		stream << static_cast<int>(dl_leg::getLegState(value.leg_state, i)) << ",";
	}

	//重心位置の出力
	stream << static_cast<int>(dl_leg::getComPatternState(value.leg_state)) << ",";

	//姿勢の出力
	stream << std::fixed << std::setprecision(kPrecision) << value.rot.pitch << "," << value.rot.roll << "," << value.rot.yaw << ",";

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

std::string std::to_string(const SNode& node)
{
	// \t はタブを表す文字．また，スペースのみを追加したい場合は std::string(" ") とする．

	std::string res;	//結果として返す文字列

	//脚状態のbit列
	res += "Leg State Bit : " + node.leg_state.to_string() + "\n";

	//重心位置の出力
	res += "\nCom Pattern : " + std::to_string(dl_leg::getComPatternState(node.leg_state)) + "\n";

	//脚の接地状態
	res += "Ground : ";

	for (int i = 0; i < HexapodConst::LEG_NUM; ++i)
	{
		res += (dl_leg::isGrounded(node.leg_state, i) ? "ground" : "lifted") + std::string(" ");
	}

	res += "\n";

	//脚の階層
	res += "Hierarchy : ";

	for (int i = 0; i < HexapodConst::LEG_NUM; ++i)
	{
		res += std::to_string(dl_leg::getLegState(node.leg_state, i)) + std::string(" ");
	}

	res += "\n\n";

	//脚位置
	res += "Leg Position : \n";

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		res += "\t" + std::to_string(i) + ":";
		res += "x : " + std::to_string(node.leg_pos[i].x) + ",\t";
		res += "y : " + std::to_string(node.leg_pos[i].y) + ",\t";
		res += "z : " + std::to_string(node.leg_pos[i].z) + "\n";
	}

	res += "Leg Base Position : \n";

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		res += "\t" + std::to_string(i) + ":";
		res += "x:" + std::to_string(node.leg_base_pos[i].x) + ",\t";
		res += "y:" + std::to_string(node.leg_base_pos[i].y) + ",\t";
		res += "z:" + std::to_string(node.leg_base_pos[i].z) + "\n";
	}

	//重心位置
	res += "\nGlobal Center of Mass : ";
	res += "x : " + std::to_string(node.global_center_of_mass.x) + std::string(",\t");
	res += "y : " + std::to_string(node.global_center_of_mass.y) + std::string(",\t");
	res += "z : " + std::to_string(node.global_center_of_mass.z) + std::string("\n");

	//回転姿勢
	res += "Rotate : ";
	res += "roll : " + std::to_string(node.rot.roll) + ",\t";
	res += "pitch : " + std::to_string(node.rot.pitch) + ",\t";
	res += "yaw : " + std::to_string(node.rot.yaw) + "\n";

	//次動作
	res += "\n";
	res += "(Next Move : " + std::to_string(node.next_move) + std::string(")\n");
	res += "(Depth : " + std::to_string(node.depth) + std::string(")\n");
	res += "(parent number : " + std::to_string(node.parent_num) + std::string(")\n");

	return res;
}
