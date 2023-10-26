#include "robot_state_node.h"

#include <iostream>

#include "designlab_math_util.h"
#include "leg_state.h"
#include "map_const.h"
#include "phantomx_const.h"


namespace dllf = designlab::leg_func;
namespace dlm = designlab::math_util;


RobotStateNode::RobotStateNode() :
	leg_state(0),
	leg_pos{},
	leg_reference_pos{},
	global_center_of_mass{},
	rot{},
	next_move(HexapodMove::kComUpDown),
	parent_num(-1),
	depth(0)
{
}


void RobotStateNode::Init(const bool do_random)
{
	using designlab::Vector3;
	using designlab::EulerXYZ;

	//脚状態
	leg_state = dllf::MakeLegStateBit(
		DiscreteComPos::kCenterBack, 
		{ true, true, true, true, true, true },
		{ DiscreteLegPos::kCenter, DiscreteLegPos::kCenter, DiscreteLegPos::kCenter,
		DiscreteLegPos::kCenter, DiscreteLegPos::kCenter, DiscreteLegPos::kCenter }
	);

	//脚付け根を原点とした，脚先の位置を初期化する．
	const float kComZ = HexapodConst::VERTICAL_MIN_RANGE + MapConst::MAX_Z_BASE;	// ロボットの重心のZ座標

	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		leg_pos[i] = leg_reference_pos[i] = {
			170 * cos(PhantomXConst::kCoxaDefaultAngle[i]),
			170 * sin(PhantomXConst::kCoxaDefaultAngle[i]),
			- kComZ
		};
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


void RobotStateNode::ChangeGlobalCenterOfMass(const designlab::Vector3& new_com, const bool base)
{
	const designlab::Vector3 kComDif = new_com - global_center_of_mass;

	global_center_of_mass = new_com;

	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		if (dllf::IsGrounded(leg_state, i))
		{
			leg_pos[i] -= designlab::rotVector(kComDif, rot * -1);
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

RobotStateNode RobotStateNode::FromString(const std::string& str)
{
	std::vector<std::string> datas;
	std::string dev = ",";

	// カンマで区切られた文字列を分割する．これあとで関数にしたほうがよい
	int first = 0;
	int last = static_cast<int>(str.find_first_of(dev));

	while (first < str.size())
	{
		std::string subStr(str, first, last - first);

		datas.push_back(subStr);

		first = last + 1;
		last = static_cast<int>(str.find_first_of(dev, first));

		if (last == std::string::npos)
		{
			last = static_cast<int>(str.size());
		}
	}
	
	RobotStateNode res;
	int cnt = 0;

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

	return res;
}