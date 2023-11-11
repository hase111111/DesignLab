#include "robot_state_node.h"

#include <iostream>
#include <sstream>

#include "designlab_math_util.h"
#include "designlab_string_util.h"
#include "leg_state.h"
#include "map_const.h"


namespace dl = ::designlab;
namespace dllf = ::designlab::leg_func;
namespace dlm = ::designlab::math_util;
namespace dlsu = ::designlab::string_util;


void RobotStateNode::ChangeGlobalCenterOfMass(const designlab::Vector3& new_com, const bool base)
{
	const designlab::Vector3 kComDif = new_com - global_center_of_mass;

	global_center_of_mass = new_com;

	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		if (dllf::IsGrounded(leg_state, i))
		{
			leg_pos[i] -= designlab::RotateVector3(kComDif, rot * -1);
			leg_reference_pos[i] = leg_pos[i];
		}
		else
		{
			//if (base)leg_reference_pos[i] = HexapodStateCalclator_Old::getLocalBaseLegPos(i, leg_reference_pos[i].z - kComDif.z);
			if (base)leg_reference_pos[i] -= kComDif;
		}
	}

}

std::string RobotStateNode::ToString() const
{
	// \t はタブを表す文字．また，スペースのみを追加したい場合は std::string(" ") とする．

	std::string res;	//結果として返す文字列

	//脚状態のbit列
	res += "Leg State Bit : " + leg_state.to_string() + "\n";
	res += "\n";

	//重心位置の出力
	DiscreteComPos com = dllf::GetDiscreteComPos(leg_state);
	res += "Com Pattern : " + static_cast<std::string>(magic_enum::enum_name(com)) + "(" + std::to_string(static_cast<int>(com)) +")\n";

	//脚の接地状態
	res += "Ground : ";

	for (int i = 0; i < HexapodConst::kLegNum; ++i)
	{
		res += (dllf::IsGrounded(leg_state, i) ? "ground" : "lifted") + std::string(" ");
	}

	res += "\n";

	//脚の階層
	res += "Hierarchy : ";

	for (int i = 0; i < HexapodConst::kLegNum; ++i)
	{
		DiscreteLegPos dis_leg_pos = dllf::GetDiscreteLegPos(leg_state, i);
		res += static_cast<std::string>(magic_enum::enum_name(dis_leg_pos)) + "(" + std::to_string(static_cast<int>(dis_leg_pos)) + ") ";
	}

	res += "\n\n";

	//脚位置
	res += "Leg Position : \n";

	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		res += "  " + std::to_string(i) + ":" + leg_pos[i].ToString() + "\n";
	}

	res += "Leg Base Position : \n";

	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		res += "  " + std::to_string(i) + ":" + leg_reference_pos[i].ToString() + "\n";
	}

	//重心位置
	res += "\nGlobal Center of Mass : " + global_center_of_mass.ToString() + "\n";

	//回転姿勢
	res += "Rotate  : " + rot.ToString() + "\n";
	res += "(Rotate : " + rot.ToStringDeg() + ")\n";

	//次動作
	res += "\n";
	res += "(Next Move : " + static_cast<std::string>(magic_enum::enum_name(next_move)) + std::string(")\n");
	res += "(Depth : " + std::to_string(depth) + std::string(")\n");
	res += "(parent number : " + std::to_string(parent_num) + std::string(")\n");

	return res;
}

std::string RobotStateNode::ToCsvString() const
{
	std::stringstream ss;

	ss << *this;

	return ss.str();
}

RobotStateNode RobotStateNode::FromString(const std::string& str)
{
	std::vector<std::string> datas = dlsu::Split(str,",");
	
	RobotStateNode res;
	int cnt = 0;

	try
	{
		//脚状態のbit列
		res.leg_state = std::bitset<designlab::leg_func::kLegStateBitNum>(datas[cnt++]);

		//脚の位置
		for (int i = 0; i < HexapodConst::kLegNum; i++)
		{
			res.leg_pos[i] = designlab::Vector3{ std::stof(datas[cnt++]),std::stof(datas[cnt++]),std::stof(datas[cnt++]) };
		}

		//脚の基準位置
		for (int i = 0; i < HexapodConst::kLegNum; i++)
		{
			res.leg_reference_pos[i] = designlab::Vector3{ std::stof(datas[cnt++]),std::stof(datas[cnt++]),std::stof(datas[cnt++]) };
		}

		//重心位置
		res.global_center_of_mass = designlab::Vector3{ std::stof(datas[cnt++]),std::stof(datas[cnt++]),std::stof(datas[cnt++]) };

		//回転姿勢
		res.rot = designlab::EulerXYZ{ std::stof(datas[cnt++]),std::stof(datas[cnt++]),std::stof(datas[cnt++]) };

		//次動作
		res.next_move = magic_enum::enum_cast<HexapodMove>(datas[cnt++]).value();

		//親ノードの番号
		res.parent_num = std::stoi(datas[cnt++]);

		//深さ
		res.depth = std::stoi(datas[cnt++]);
	}
	catch (...)
	{
		//todo うまく読み込めなかった時の処理を書く．
	}

	return res;
}