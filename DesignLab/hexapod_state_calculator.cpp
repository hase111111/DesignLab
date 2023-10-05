//#include "hexapod_state_calculator.h"
//
//#include <cmath>
//
//#include "designlab_math_util.h"
//#include "designlab_line_segment2.h"
//#include "leg_state.h"
//
//
//namespace dlm = ::designlab::math_util;
//
//
//float HexapodStateCalclator_Old::m_leg_max_r[200] = {};
//float HexapodStateCalclator_Old::m_leg_min_r[200] = {};
//
//
//HexapodStateCalclator_Old::HexapodStateCalclator_Old()
//{
//	//ジョイントの位置を初期化する．
//	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
//	{
//		m_local_femurjoint_pos[i] = m_local_tibiajoint_pos[i] = designlab::Vector3(0, 0, 0);
//	}
//}
//
//
//designlab::Vector3 HexapodStateCalclator_Old::convertLocalLegPos(const RobotStateNode& node, const designlab::Vector3& global_pos, const int leg_num, const bool do_consider_rot) const
//{
//	if (do_consider_rot)
//	{
//		return rotVector(global_pos - node.global_center_of_mass, node.rot * -1) - getLocalCoxaJointPos(leg_num);
//	}
//	else
//	{
//		return global_pos - node.global_center_of_mass - getLocalCoxaJointPos(leg_num);
//	}
//}
//
//
//void HexapodStateCalclator_Old::calclateJointPos(const RobotStateNode& _node)
//{
//	// 逆運動学的にジョイントの場所を計算する．
//	//ノードの脚位置は正しい場所にあるという前提のもと計算するので，めちゃくちゃな値が代入されているとうまく動作しない．
//	//チェックする機能を付けると重くなるので，そもそもそんなノードを生成しないように注意する．
//
//	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
//	{
//		float _coxa_joint_angle = atan2(_node.leg_pos[i].y, _node.leg_pos[i].x);
//
//		if (_node.leg_pos[i].x == 0 || _node.leg_pos[i].y == 0) { _coxa_joint_angle = HexapodConst::PHANTOMX_COXA_DEFAULT_ANGLE[i]; }
//
//		m_local_femurjoint_pos[i] = designlab::Vector3(HexapodConst::PHANTOMX_COXA_LENGTH * cos(_coxa_joint_angle), HexapodConst::PHANTOMX_COXA_LENGTH * sin(_coxa_joint_angle), 0);
//
//
//		const float _L = std::sqrt(dlm::Squared(_node.leg_pos[i].x - m_local_femurjoint_pos[i].x) + dlm::Squared(_node.leg_pos[i].y - m_local_femurjoint_pos[i].y));				//脚先から第一関節までの長さ．
//		const float _leg_to_fumur_len = std::sqrt(dlm::Squared(_L) + dlm::Squared(_node.leg_pos[i].z));
//
//		const float _s1 = dlm::Squared(_leg_to_fumur_len) + dlm::Squared(HexapodConst::PHANTOMX_TIBIA_LENGTH) - dlm::Squared(HexapodConst::PHANTOMX_FEMUR_LENGTH);
//		const float _s2 = 2 * HexapodConst::PHANTOMX_TIBIA_LENGTH * _leg_to_fumur_len;
//
//		const float _fumur_joint_angle = -(std::acos(_s1 / _s2) + std::atan(-_node.leg_pos[i].z / _L));
//
//		m_local_tibiajoint_pos[i] = _node.leg_pos[i] -
//			designlab::Vector3(HexapodConst::PHANTOMX_TIBIA_LENGTH * cos(_coxa_joint_angle) * cos(_fumur_joint_angle),
//				HexapodConst::PHANTOMX_TIBIA_LENGTH * sin(_coxa_joint_angle) * cos(_fumur_joint_angle),
//				HexapodConst::PHANTOMX_TIBIA_LENGTH * sin(_fumur_joint_angle));
//
//		if (abs((m_local_femurjoint_pos[i] - m_local_tibiajoint_pos[i]).Length() - HexapodConst::PHANTOMX_FEMUR_LENGTH) > dlm::kAllowableError)
//		{
//			const float _fumur_joint_angle2 = -(-std::acos(_s1 / _s2) + std::atan(-_node.leg_pos[i].z / _L));
//
//			m_local_tibiajoint_pos[i] = _node.leg_pos[i] -
//				designlab::Vector3(HexapodConst::PHANTOMX_TIBIA_LENGTH * cos(_coxa_joint_angle) * cos(_fumur_joint_angle2),
//					HexapodConst::PHANTOMX_TIBIA_LENGTH * sin(_coxa_joint_angle) * cos(_fumur_joint_angle2),
//					HexapodConst::PHANTOMX_TIBIA_LENGTH * sin(_fumur_joint_angle2));
//		}
//	}
//
//}
//
//
//designlab::Vector3 HexapodStateCalclator_Old::getGlobalFemurJointPos(const RobotStateNode& _node, const int _leg_num) const
//{
//	return rotVector(getLocalCoxaJointPos(_leg_num) + m_local_femurjoint_pos[_leg_num], _node.rot) + _node.global_center_of_mass;
//}
//
//
//designlab::Vector3 HexapodStateCalclator_Old::getGlobalTibiaJointPos(const RobotStateNode& _node, const int _leg_num) const
//{
//	return rotVector(getLocalCoxaJointPos(_leg_num) + m_local_tibiajoint_pos[_leg_num], _node.rot) + _node.global_center_of_mass;
//}
//
//
//void HexapodStateCalclator_Old::initLegR()
//{
//	for (int _z = 0; _z < MAX_DIF_Z; _z++)
//	{
//		float _max_r = 0;
//		float _min_r = 0;
//
//		for (float _x = HexapodConst::PHANTOMX_COXA_LENGTH; _x < HexapodConst::PHANTOMX_COXA_LENGTH + HexapodConst::PHANTOMX_FEMUR_LENGTH + HexapodConst::PHANTOMX_TIBIA_LENGTH; _x++)
//		{
//			designlab::Vector3 _tmp_leg((float)_x, 0, -(float)_z);
//
//			// 以下の三変数を辺とする三角形が成立するか調べる．
//			float _a = HexapodConst::PHANTOMX_TIBIA_LENGTH;
//			float _b = HexapodConst::PHANTOMX_FEMUR_LENGTH;
//			float _c = sqrt(dlm::Squared(_tmp_leg.x - HexapodConst::PHANTOMX_COXA_LENGTH) + dlm::Squared(_tmp_leg.z));
//
//			bool _is_vaild_triangle = true;
//			if (_a + _b < _c)_is_vaild_triangle = false;
//			if (_a + _c < _b)_is_vaild_triangle = false;
//			if (_c + _b < _a)_is_vaild_triangle = false;
//
//			//成立するのならば，そこに脚を伸ばすことができるはず．
//			if (_is_vaild_triangle)
//			{
//				if (_min_r == 0)_min_r = _x;
//				_max_r = _x;
//			}
//
//		}
//
//		m_leg_max_r[_z] = _max_r;
//		m_leg_min_r[_z] = _min_r;
//	}
//}
//
//
//float HexapodStateCalclator_Old::getMaxLegR(const float _coxa_z_to_leg_z) const
//{
//	int _dif_z = (int)abs(_coxa_z_to_leg_z);		// Z座標の差異を正の整数の値に変換する．誤差はでるけど高速に処理できるようになる．
//
//	//値が範囲外であるならば，0を返す．
//	if (_dif_z < 0) { return 0.0f; }
//	if (MAX_DIF_Z < _dif_z) { return 0.0f; }
//
//	return m_leg_max_r[_dif_z];
//}
//
//
//float HexapodStateCalclator_Old::getMinLegR(const float _coxa_z_to_leg_z) const
//{
//	int _dif_z = (int)abs(_coxa_z_to_leg_z);		// Z座標の差異を正の整数の値に変換する．誤差はでるけど高速に処理できるようになる．
//
//	//値が範囲外であるならば，0を返す．
//	if (_dif_z < 0) { return 0.0f; }
//	if (MAX_DIF_Z < _dif_z) { return 0.0f; }
//
//	return m_leg_min_r[_dif_z];
//}
//
//
//bool HexapodStateCalclator_Old::IsLegInterfering(const RobotStateNode& _node) const
//{
//	//重心を原点とした，座標系において，脚の干渉を調べる．
//
//	//脚の干渉を調べる．
//	designlab::Vector2 _leg_pos[HexapodConst::LEG_NUM];
//	designlab::Vector2 _joint_pos[HexapodConst::LEG_NUM];
//
//	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
//	{
//		_joint_pos[i] = getLocalCoxaJointPos(i).ProjectedXY();
//		_leg_pos[i] = _node.leg_pos[i].ProjectedXY() + _joint_pos[i];
//	}
//
//	//隣の脚との干渉を調べる．
//	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
//	{
//		designlab::LineSegment2 _line1(_joint_pos[i], _leg_pos[i]);
//		designlab::LineSegment2 _line2(_joint_pos[(i + 1) % HexapodConst::LEG_NUM], _leg_pos[(i + 1) % HexapodConst::LEG_NUM]);
//
//		if (_line1.HasIntersection(_line2)) { return true; }
//	}
//
//	return false;
//}
//
//
//bool HexapodStateCalclator_Old::IsLegInRange(const RobotStateNode& node, const int leg_num) const
//{
//	const designlab::Vector2 leg_pos_xy = node.leg_pos[leg_num].ProjectedXY();
//	const designlab::Vector2 min_leg_pos_xy{HexapodConst::MOVABLE_LEG_RANGE_COS_MIN[leg_num], HexapodConst::MOVABLE_LEG_RANGE_SIN_MAX[leg_num]};
//	const designlab::Vector2 max_leg_pos_xy{HexapodConst::MOVABLE_LEG_RANGE_COS_MAX[leg_num], HexapodConst::MOVABLE_LEG_RANGE_SIN_MIN[leg_num]};
//
//	//脚の角度が範囲内にあるか調べる．外積計算で間にあるか調べる
//	if (min_leg_pos_xy.Cross(leg_pos_xy) > 0.0f) { return false; }
//	if (max_leg_pos_xy.Cross(leg_pos_xy) < 0.0f) { return false; }
//
//
//	////脚を伸ばすことのできない範囲に伸ばしていないか調べる．
//	if (dlm::Squared(getMinLegR(node.leg_pos[leg_num].z)) > leg_pos_xy.LengthSquare()) { return false; }
//	if (dlm::Squared(getMaxLegR(node.leg_pos[leg_num].z)) < leg_pos_xy.LengthSquare()) { return false; }
//
//	return true;
//}
//
//
//bool HexapodStateCalclator_Old::IsLegInRange(const designlab::Vector3& local_leg_pos, const int leg_num) const
//{
//	const designlab::Vector2 leg_pos_xy = local_leg_pos.ProjectedXY();
//	const designlab::Vector2 min_leg_pos_xy{HexapodConst::MOVABLE_LEG_RANGE_COS_MIN[leg_num], HexapodConst::MOVABLE_LEG_RANGE_SIN_MAX[leg_num]};
//	const designlab::Vector2 max_leg_pos_xy{HexapodConst::MOVABLE_LEG_RANGE_COS_MAX[leg_num], HexapodConst::MOVABLE_LEG_RANGE_SIN_MIN[leg_num]};
//
//	//脚の角度が範囲内にあるか調べる．外積計算で間にあるか調べる
//	if (min_leg_pos_xy.Cross(leg_pos_xy) > 0.0f) { return false; }
//	if (max_leg_pos_xy.Cross(leg_pos_xy) < 0.0f) { return false; }
//
//	////脚を伸ばすことのできない範囲に伸ばしていないか調べる．
//	if (dlm::Squared(getMinLegR(local_leg_pos.z)) > leg_pos_xy.LengthSquare()) { return false; }
//	if (dlm::Squared(getMaxLegR(local_leg_pos.z)) < leg_pos_xy.LengthSquare()) { return false; }
//
//	return true;
//}
//
//
//bool HexapodStateCalclator_Old::isAllLegInRange(const RobotStateNode& node) const
//{
//	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
//	{
//		if (dl_leg::IsGrounded(node.leg_state, i))
//		{
//			if (!IsLegInRange(node.leg_pos[i], i)) { return false; }
//		}
//	}
//
//	return true;
//}
//
//
//bool HexapodStateCalclator_Old::isAblePause(const RobotStateNode& _node) const
//{
//	//重心を原点とした座標系で，脚の位置を計算する．
//	//かつてvectorを使っていたが，処理速度の問題で，配列を使うことにした．
//
//	designlab::Vector2 leg_pos[HexapodConst::LEG_NUM];
//	int leg_pos_index = 0;
//
//	//接地脚のみ追加する
//	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
//	{
//		if (dl_leg::IsGrounded(_node.leg_state, i) == true)
//		{
//			leg_pos[leg_pos_index] = _node.leg_pos[i].ProjectedXY() + getLocalCoxaJointPos(i).ProjectedXY();
//			++leg_pos_index;
//		}
//	}
//
//	for (int i = 0; i < leg_pos_index; i++)
//	{
//		designlab::Vector2 i_to_i_plus_1 = leg_pos[(i + 1) % leg_pos_index] - leg_pos[i];
//		designlab::Vector2 i_to_com = designlab::Vector2{ 0,0 } - leg_pos[i];
//
//		if (i_to_i_plus_1.Cross(i_to_com) > 0)return false;
//	}
//
//	return true;
//}
//
//
//float HexapodStateCalclator_Old::calculateStaticMargin(const RobotStateNode& node) const
//{
//	//重心を原点とした座標系で，脚の位置を計算する．
//	// std::min をカッコで囲んでいるのは，マクロの min と被るため．(std::min) と書くと名前が衝突しない
//
//	std::vector<designlab::Vector2> leg_pos;
//
//	//接地脚のみ追加する
//	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
//	{
//		if (dl_leg::IsGrounded(node.leg_state, i) == true)
//		{
//			leg_pos.push_back(node.leg_pos[i].ProjectedXY() + getLocalCoxaJointPos(i).ProjectedXY());
//		}
//	}
//
//	float min_margin = 1000000;
//
//	for (int i = 0; i < leg_pos.size(); i++)
//	{
//		designlab::Vector2 i_to_i_plus_1 = leg_pos.at((i + 1) % leg_pos.size()) - leg_pos.at(i);
//		i_to_i_plus_1.Normalize();
//
//		designlab::Vector2 i_to_com = designlab::Vector2{ 0,0 } - leg_pos.at(i);
//
//		min_margin = (std::min)(min_margin, i_to_com.Cross(i_to_i_plus_1));
//	}
//
//	return min_margin;
//}
