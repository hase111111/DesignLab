#include "hexapod_state_calculator.h"

#include <cmath>

#include "designlab_math.h"
#include "designlab_line.h"
#include "leg_state.h"


float HexapodStateCalclator_Old::m_leg_max_r[200] = {};
float HexapodStateCalclator_Old::m_leg_min_r[200] = {};


HexapodStateCalclator_Old::HexapodStateCalclator_Old()
{
	//�W���C���g�̈ʒu������������D
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		m_local_femurjoint_pos[i] = m_local_tibiajoint_pos[i] = designlab::Vector3(0, 0, 0);
	}
}


designlab::Vector3 HexapodStateCalclator_Old::convertLocalLegPos(const SNode& node, const designlab::Vector3& global_pos, const int leg_num, const bool do_consider_rot) const
{
	if (do_consider_rot)
	{
		return rotVector(global_pos - node.global_center_of_mass, node.rot * -1) - getLocalCoxaJointPos(leg_num);
	}
	else
	{
		return global_pos - node.global_center_of_mass - getLocalCoxaJointPos(leg_num);
	}
}


void HexapodStateCalclator_Old::calclateJointPos(const SNode& _node)
{
	// �t�^���w�I�ɃW���C���g�̏ꏊ���v�Z����D
	//�m�[�h�̋r�ʒu�͐������ꏊ�ɂ���Ƃ����O��̂��ƌv�Z����̂ŁC�߂��Ⴍ����Ȓl���������Ă���Ƃ��܂����삵�Ȃ��D
	//�`�F�b�N����@�\��t����Əd���Ȃ�̂ŁC������������ȃm�[�h�𐶐����Ȃ��悤�ɒ��ӂ���D

	using namespace dl_math;

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		float _coxa_joint_angle = atan2(_node.leg_pos[i].y, _node.leg_pos[i].x);

		if (_node.leg_pos[i].x == 0 || _node.leg_pos[i].y == 0) { _coxa_joint_angle = HexapodConst::PHANTOMX_COXA_DEFAULT_ANGLE[i]; }

		m_local_femurjoint_pos[i] = designlab::Vector3(HexapodConst::PHANTOMX_COXA_LENGTH * cos(_coxa_joint_angle), HexapodConst::PHANTOMX_COXA_LENGTH * sin(_coxa_joint_angle), 0);


		const float _L = std::sqrt(squared(_node.leg_pos[i].x - m_local_femurjoint_pos[i].x) + squared(_node.leg_pos[i].y - m_local_femurjoint_pos[i].y));				//�r�悩����֐߂܂ł̒����D
		const float _leg_to_fumur_len = std::sqrt(squared(_L) + squared(_node.leg_pos[i].z));

		const float _s1 = squared(_leg_to_fumur_len) + squared(HexapodConst::PHANTOMX_TIBIA_LENGTH) - squared(HexapodConst::PHANTOMX_FEMUR_LENGTH);
		const float _s2 = 2 * HexapodConst::PHANTOMX_TIBIA_LENGTH * _leg_to_fumur_len;

		const float _fumur_joint_angle = -(std::acos(_s1 / _s2) + std::atan(-_node.leg_pos[i].z / _L));

		m_local_tibiajoint_pos[i] = _node.leg_pos[i] -
			designlab::Vector3(HexapodConst::PHANTOMX_TIBIA_LENGTH * cos(_coxa_joint_angle) * cos(_fumur_joint_angle),
				HexapodConst::PHANTOMX_TIBIA_LENGTH * sin(_coxa_joint_angle) * cos(_fumur_joint_angle),
				HexapodConst::PHANTOMX_TIBIA_LENGTH * sin(_fumur_joint_angle));

		if (abs((m_local_femurjoint_pos[i] - m_local_tibiajoint_pos[i]).length() - HexapodConst::PHANTOMX_FEMUR_LENGTH) > dl_math::ALLOWABLE_ERROR)
		{
			const float _fumur_joint_angle = -(-std::acos(_s1 / _s2) + std::atan(-_node.leg_pos[i].z / _L));

			m_local_tibiajoint_pos[i] = _node.leg_pos[i] -
				designlab::Vector3(HexapodConst::PHANTOMX_TIBIA_LENGTH * cos(_coxa_joint_angle) * cos(_fumur_joint_angle),
					HexapodConst::PHANTOMX_TIBIA_LENGTH * sin(_coxa_joint_angle) * cos(_fumur_joint_angle),
					HexapodConst::PHANTOMX_TIBIA_LENGTH * sin(_fumur_joint_angle));
		}
	}

}


designlab::Vector3 HexapodStateCalclator_Old::getGlobalFemurJointPos(const SNode& _node, const int _leg_num) const
{
	return rotVector(getLocalCoxaJointPos(_leg_num) + m_local_femurjoint_pos[_leg_num], _node.rot) + _node.global_center_of_mass;
}


designlab::Vector3 HexapodStateCalclator_Old::getGlobalTibiaJointPos(const SNode& _node, const int _leg_num) const
{
	return rotVector(getLocalCoxaJointPos(_leg_num) + m_local_tibiajoint_pos[_leg_num], _node.rot) + _node.global_center_of_mass;
}


void HexapodStateCalclator_Old::initLegR()
{
	using namespace dl_math;

	for (int _z = 0; _z < MAX_DIF_Z; _z++)
	{
		float _max_r = 0;
		float _min_r = 0;

		for (float _x = HexapodConst::PHANTOMX_COXA_LENGTH; _x < HexapodConst::PHANTOMX_COXA_LENGTH + HexapodConst::PHANTOMX_FEMUR_LENGTH + HexapodConst::PHANTOMX_TIBIA_LENGTH; _x++)
		{
			designlab::Vector3 _tmp_leg((float)_x, 0, -(float)_z);

			// �ȉ��̎O�ϐ���ӂƂ���O�p�`���������邩���ׂ�D
			float _a = HexapodConst::PHANTOMX_TIBIA_LENGTH;
			float _b = HexapodConst::PHANTOMX_FEMUR_LENGTH;
			float _c = sqrt(squared(_tmp_leg.x - HexapodConst::PHANTOMX_COXA_LENGTH) + squared(_tmp_leg.z));

			bool _is_vaild_triangle = true;
			if (_a + _b < _c)_is_vaild_triangle = false;
			if (_a + _c < _b)_is_vaild_triangle = false;
			if (_c + _b < _a)_is_vaild_triangle = false;

			//��������̂Ȃ�΁C�����ɋr��L�΂����Ƃ��ł���͂��D
			if (_is_vaild_triangle)
			{
				if (_min_r == 0)_min_r = _x;
				_max_r = _x;
			}

		}

		m_leg_max_r[_z] = _max_r;
		m_leg_min_r[_z] = _min_r;
	}
}


float HexapodStateCalclator_Old::getMaxLegR(const float _coxa_z_to_leg_z) const
{
	int _dif_z = (int)abs(_coxa_z_to_leg_z);		// Z���W�̍��ق𐳂̐����̒l�ɕϊ�����D�덷�͂ł邯�Ǎ����ɏ����ł���悤�ɂȂ�D

	//�l���͈͊O�ł���Ȃ�΁C0��Ԃ��D
	if (_dif_z < 0) { return 0.0f; }
	if (MAX_DIF_Z < _dif_z) { return 0.0f; }

	return m_leg_max_r[_dif_z];
}


float HexapodStateCalclator_Old::getMinLegR(const float _coxa_z_to_leg_z) const
{
	int _dif_z = (int)abs(_coxa_z_to_leg_z);		// Z���W�̍��ق𐳂̐����̒l�ɕϊ�����D�덷�͂ł邯�Ǎ����ɏ����ł���悤�ɂȂ�D

	//�l���͈͊O�ł���Ȃ�΁C0��Ԃ��D
	if (_dif_z < 0) { return 0.0f; }
	if (MAX_DIF_Z < _dif_z) { return 0.0f; }

	return m_leg_min_r[_dif_z];
}


bool HexapodStateCalclator_Old::isLegInterfering(const SNode& _node) const
{
	//�d�S�����_�Ƃ����C���W�n�ɂ����āC�r�̊��𒲂ׂ�D

	//�r�̊��𒲂ׂ�D
	designlab::SVector2 _leg_pos[HexapodConst::LEG_NUM];
	designlab::SVector2 _joint_pos[HexapodConst::LEG_NUM];

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		_joint_pos[i] = getLocalCoxaJointPos(i).projectedXY();
		_leg_pos[i] = _node.leg_pos[i].projectedXY() + _joint_pos[i];
	}

	//�ׂ̋r�Ƃ̊��𒲂ׂ�D
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		designlab::SLine2 _line1(_joint_pos[i], _leg_pos[i]);
		designlab::SLine2 _line2(_joint_pos[(i + 1) % HexapodConst::LEG_NUM], _leg_pos[(i + 1) % HexapodConst::LEG_NUM]);

		if (_line1.hasIntersection(_line2)) { return true; }
	}

	return false;
}


bool HexapodStateCalclator_Old::isLegInRange(const SNode& node, const int leg_num) const
{
	const designlab::SVector2 leg_pos_xy = node.leg_pos[leg_num].projectedXY();
	const designlab::SVector2 min_leg_pos_xy{HexapodConst::MOVABLE_LEG_RANGE_COS_MIN[leg_num], HexapodConst::MOVABLE_LEG_RANGE_SIN_MAX[leg_num]};
	const designlab::SVector2 max_leg_pos_xy{HexapodConst::MOVABLE_LEG_RANGE_COS_MAX[leg_num], HexapodConst::MOVABLE_LEG_RANGE_SIN_MIN[leg_num]};

	//�r�̊p�x���͈͓��ɂ��邩���ׂ�D�O�όv�Z�ŊԂɂ��邩���ׂ�
	if (min_leg_pos_xy.cross(leg_pos_xy) > 0.0f) { return false; }
	if (max_leg_pos_xy.cross(leg_pos_xy) < 0.0f) { return false; }


	////�r��L�΂����Ƃ̂ł��Ȃ��͈͂ɐL�΂��Ă��Ȃ������ׂ�D
	if (dl_math::squared(getMinLegR(node.leg_pos[leg_num].z)) > leg_pos_xy.lengthSquare()) { return false; }
	if (dl_math::squared(getMaxLegR(node.leg_pos[leg_num].z)) < leg_pos_xy.lengthSquare()) { return false; }

	return true;
}


bool HexapodStateCalclator_Old::isLegInRange(const designlab::Vector3& local_leg_pos, const int leg_num) const
{
	const designlab::SVector2 leg_pos_xy = local_leg_pos.projectedXY();
	const designlab::SVector2 min_leg_pos_xy{HexapodConst::MOVABLE_LEG_RANGE_COS_MIN[leg_num], HexapodConst::MOVABLE_LEG_RANGE_SIN_MAX[leg_num]};
	const designlab::SVector2 max_leg_pos_xy{HexapodConst::MOVABLE_LEG_RANGE_COS_MAX[leg_num], HexapodConst::MOVABLE_LEG_RANGE_SIN_MIN[leg_num]};

	//�r�̊p�x���͈͓��ɂ��邩���ׂ�D�O�όv�Z�ŊԂɂ��邩���ׂ�
	if (min_leg_pos_xy.cross(leg_pos_xy) > 0.0f) { return false; }
	if (max_leg_pos_xy.cross(leg_pos_xy) < 0.0f) { return false; }

	////�r��L�΂����Ƃ̂ł��Ȃ��͈͂ɐL�΂��Ă��Ȃ������ׂ�D
	if (dl_math::squared(getMinLegR(local_leg_pos.z)) > leg_pos_xy.lengthSquare()) { return false; }
	if (dl_math::squared(getMaxLegR(local_leg_pos.z)) < leg_pos_xy.lengthSquare()) { return false; }

	return true;
}


bool HexapodStateCalclator_Old::isAllLegInRange(const SNode& node) const
{
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		if (dl_leg::isGrounded(node.leg_state, i))
		{
			if (!isLegInRange(node.leg_pos[i], i)) { return false; }
		}
	}

	return true;
}


bool HexapodStateCalclator_Old::isAblePause(const SNode& _node) const
{
	//�d�S�����_�Ƃ������W�n�ŁC�r�̈ʒu���v�Z����D
	//����vector���g���Ă������C�������x�̖��ŁC�z����g�����Ƃɂ����D

	designlab::SVector2 leg_pos[HexapodConst::LEG_NUM];
	int leg_pos_index = 0;

	//�ڒn�r�̂ݒǉ�����
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		if (dl_leg::isGrounded(_node.leg_state, i) == true)
		{
			leg_pos[leg_pos_index] = _node.leg_pos[i].projectedXY() + getLocalCoxaJointPos(i).projectedXY();
			++leg_pos_index;
		}
	}

	for (int i = 0; i < leg_pos_index; i++)
	{
		designlab::SVector2 i_to_i_plus_1 = leg_pos[(i + 1) % leg_pos_index] - leg_pos[i];
		designlab::SVector2 i_to_com = designlab::SVector2{ 0,0 } - leg_pos[i];

		if (i_to_i_plus_1.cross(i_to_com) > 0)return false;
	}

	return true;
}


float HexapodStateCalclator_Old::calculateStaticMargin(const SNode& node) const
{
	//�d�S�����_�Ƃ������W�n�ŁC�r�̈ʒu���v�Z����D
	// std::min ���J�b�R�ň͂�ł���̂́C�}�N���� min �Ɣ�邽�߁D(std::min) �Ə����Ɩ��O���Փ˂��Ȃ�

	std::vector<designlab::SVector2> leg_pos;

	//�ڒn�r�̂ݒǉ�����
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		if (dl_leg::isGrounded(node.leg_state, i) == true)
		{
			leg_pos.push_back(node.leg_pos[i].projectedXY() + getLocalCoxaJointPos(i).projectedXY());
		}
	}

	float min_margin = 1000000;

	for (int i = 0; i < leg_pos.size(); i++)
	{
		designlab::SVector2 i_to_i_plus_1 = leg_pos.at((i + 1) % leg_pos.size()) - leg_pos.at(i);
		i_to_i_plus_1.normalized();

		designlab::SVector2 i_to_com = designlab::SVector2{ 0,0 } - leg_pos.at(i);

		min_margin = (std::min)(min_margin, i_to_com.cross(i_to_i_plus_1));
	}

	return min_margin;
}
