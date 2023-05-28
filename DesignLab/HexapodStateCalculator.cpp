#include "HexapodStateCalculator.h"
#include "MyMath.h"
#include <cmath>

using namespace myvector;

HexapodStateCalclator::HexapodStateCalclator()
{
	//ジョイントの位置を初期化する．
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		m_local_femurjoint_pos[i] = m_local_tibiajoint_pos[i] = SVector(0, 0, 0);
	}
}

myvector::SVector HexapodStateCalclator::getGlobalLegPos(const SNode& _node, const int _leg_num) const
{
	return rotVector(getLocalCoxaJointPos(_leg_num) + _node.Leg[_leg_num],_node.rot) + _node.global_center_of_mass;
}

myvector::SVector HexapodStateCalclator::getGlobalCoxaJointPos(const SNode& _node, const int _leg_num) const
{
	return rotVector(getLocalCoxaJointPos(_leg_num), _node.rot) + _node.global_center_of_mass;
}

void HexapodStateCalclator::calclateJointPos(const SNode& _node)
{
	// 逆運動学的にジョイントの場所を計算する．
	//ノードの脚位置は正しい場所にあるという前提のもと計算するので，めちゃくちゃな値が代入されているとうまく動作しない．
	//チェックする機能を付けると重くなるので，そもそもそんなノードを生成しないように注意する．

	using namespace my_math;

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		const float _coxa_joint_angle = atan2(_node.Leg[i].y,_node.Leg[i].x);

		m_local_femurjoint_pos[i] = SVector(HexapodConst::COXA_LENGTH * cos(_coxa_joint_angle), HexapodConst::COXA_LENGTH * sin(_coxa_joint_angle), 0);

		const float _leg_to_coxa_len = sqrt(squared(_node.Leg[i].x) + squared(_node.Leg[i].y));		//真上から見たときの，脚先から脚の付け根までの長さ．
		const float _leg_to_fumur_len = _leg_to_coxa_len - HexapodConst::COXA_LENGTH;				//真上から見たときの，脚先から第一関節までの長さ．

		const float _s1 = squared(_leg_to_fumur_len) + squared(HexapodConst::FEMUR_LENGTH) + squared(_node.Leg[i].z) - squared(HexapodConst::TIBIA_LENGTH);
		const float _s2 = 2 * HexapodConst::FEMUR_LENGTH * _leg_to_fumur_len * sqrt(squared(_leg_to_fumur_len) + squared(_node.Leg[i].z));

		const float _fumur_joint_angle = -atan(_leg_to_fumur_len / _node.Leg[i].z) + asin(_s1 / _s2);

		m_local_tibiajoint_pos[i] = m_local_femurjoint_pos[i] + 
										SVector(HexapodConst::FEMUR_LENGTH * cos(_coxa_joint_angle) * cos(_fumur_joint_angle), 
												HexapodConst::FEMUR_LENGTH * sin(_coxa_joint_angle) * cos(_fumur_joint_angle), 
												HexapodConst::FEMUR_LENGTH * sin(_fumur_joint_angle));
	}

}

myvector::SVector HexapodStateCalclator::getGlobalFemurJointPos(const SNode& _node,const int _leg_num) const
{
	return rotVector(getLocalCoxaJointPos(_leg_num) + m_local_femurjoint_pos[_leg_num], _node.rot) + _node.global_center_of_mass;
}

myvector::SVector HexapodStateCalclator::getGlobalTibiaJointPos(const SNode& _node, const int _leg_num) const
{
	return rotVector(getLocalCoxaJointPos(_leg_num) + m_local_tibiajoint_pos[_leg_num], _node.rot) + _node.global_center_of_mass;
}

myvector::SVector HexapodStateCalclator::getLocalCoxaJointPos(const int _leg_num) const
{
	//重心を原点とした座標．ロボットの正面をxの正，ロボットの上をzの正，右手座標系でy座標をとっている．
	//グローバル座標系のxyz軸とは別の軸なので，回転は考慮されていない．

	if (_leg_num == 0)		{ return SVector(HexapodConst::BODY_FRONT_LENGTH,	-HexapodConst::BODY_FRONT_WIDTH,	0.0f); }	// 脚0 右上
	else if (_leg_num == 1) { return SVector(0.0f,								-HexapodConst::BODY_CENTER_WIDTH,	0.0f); }	// 脚1 右横
	else if (_leg_num == 2)	{ return SVector(-HexapodConst::BODY_REAR_LENGTH,	-HexapodConst::BODY_REAR_WIDTH,		0.0f); }	// 脚2 右下
	else if (_leg_num == 3)	{ return SVector(-HexapodConst::BODY_REAR_LENGTH,	HexapodConst::BODY_REAR_WIDTH,		0.0f); }	// 脚3 左下
	else if (_leg_num == 4)	{ return SVector(0.0f,								HexapodConst::BODY_CENTER_WIDTH,	0.0f); }	// 脚4 左横
	else if (_leg_num == 5)	{ return SVector(HexapodConst::BODY_FRONT_LENGTH,	HexapodConst::BODY_FRONT_WIDTH,		0.0f); }	// 脚5 左上

	return myvector::SVector(0, 0, 0);
}
