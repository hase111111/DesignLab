//#pragma once
//
//#include "designlab_vector3.h"
//#include "designlab_euler.h"
//#include "robot_state_node.h"
//#include "hexapod_const.h"
//
//
////! @class HexapodStateCalclator_Old
////! @brief ���{�b�g�̍��W���ԂȂǂ̒l���v�Z����N���X�D�� Hexapod�N���X���y���������́D
//class HexapodStateCalclator_Old
//{
//public:
//	HexapodStateCalclator_Old();
//
//	static constexpr designlab::Vector3 GetLocalBaseLegPos(const int _leg_num, const float _z)
//	{
//		return { 160 * HexapodConst::DEFAULT_LEG_ANGLE_COS[_leg_num],160 * HexapodConst::DEFAULT_LEG_ANGLE_SIN[_leg_num],_z };
//	}
//
//	//! @brief ��2�����̍��W�����݂̏d�S���W�Ɖ�]����C�r�̕t���������_�Ƃ������[�J�����W�ɕϊ�����D
//	//! @param [in] node ���{�b�g�̏�Ԃ�\���m�[�h
//	//! @param [in] global_pos �ϊ�����O���[�o�����W
//	//! @param [in] leg_num �r�ԍ� 0�`5
//	//! @param [in] do_consider_rot ��]���l�����邩�ǂ���
//	//! @return designlab::Vector3 ���[�J�����W�̋r���W
//	designlab::Vector3 ConvertLocalLegPos(const RobotStateNode& node, const designlab::Vector3& global_pos, const int leg_num, const bool do_consider_rot) const;
//
//	//! @brief �r���W�͋r�̕t���������_�Ƃ������W�n�Ȃ̂ŁC������O���[�o�����W�ɕϊ�����D
//	//! @param [in] node ���{�b�g�̏�Ԃ�\���m�[�h
//	//! @param [in] leg_num �r�ԍ� 0�`5
//	//! @param [in] consider_rot ��]���l�����邩�ǂ���
//	//! @return designlab::Vector3 �O���[�o�����W�̋r���W
//	inline designlab::Vector3 GetGlobalLegPos(const RobotStateNode& node, const int leg_num, const bool consider_rot) const
//	{
//		if (consider_rot) { return RotVector(getLocalCoxaJointPos(leg_num) + node.leg_pos[leg_num], node.rot) + node.global_center_of_mass; }
//		else { return node.global_center_of_mass + GetLocalCoxaJointPos(leg_num) + node.leg_pos[leg_num]; }
//	}
//
//	//! @brief �r�ʒu�͗��U������Đ��䂳��邪�C���̎���4�̈ʒu���O���[�o�����W�ŏo�͂���D
//	//! @param [in] node ���{�b�g�̏�Ԃ�\���m�[�h
//	//! @param [in] leg_num �r�ԍ� 0�`5
//	//! @param [in] consider_rot ��]���l�����邩�ǂ���
//	//! @return designlab::Vector3 �O���[�o�����W�̋r�̊�n�_�̍��W
//	inline designlab::Vector3 GetGlobalLegBasePos(const RobotStateNode& node, const int leg_num, const bool consider_rot) const
//	{
//		if (consider_rot) { return RotVector(GetLocalCoxaJointPos(leg_num) + _node.leg_reference_pos[leg_num], node.rot) + node.global_center_of_mass; }
//		else { return node.global_center_of_mass + GetLocalCoxaJointPos(leg_num) + _node.leg_reference_pos[leg_num]; }
//	}
//
//	// @brief coxa joint (�r�̕t���� : ��1�֐�) ���O���[�o�����W�ŕԂ��D
//	// @param [in] _node ���{�b�g�̏�Ԃ�\���m�[�h
//	// @param [in] leg_num �r�ԍ� 0�`5
//	// @param [in] _consider_rot ��]���l�����邩�ǂ���
//	// @return designlab::Vector3 �O���[�o�����W�̋r�̕t�����̍��W
//	inline designlab::Vector3 GetGlobalCoxaJointPos(const RobotStateNode& _node, const int _leg_num, const bool _consider_rot) const
//	{
//		if (consider_rot) { return RotateVector3(getLocalCoxaJointPos(_leg_num), _node.rot) + _node.global_center_of_mass; }
//		else { return _node.global_center_of_mass + getLocalCoxaJointPos(_leg_num); }
//	}
//
//	//! @brief �m�[�h�̏��͌��݂̋r�ʒu�Əd�S�ʒu���������Ȃ��̂ŁC�W���C���g���ǂ��ɂ��邩��������Ȃ��D����Ă��̊֐��Ōv�Z����D<br>
//	//! �O�p�֐��𑽂��g�p����̂ŁC�v�Z�ʂ������D
//	//! @param [in] _node ���{�b�g�̏�Ԃ�\���m�[�h
//	//! @details ���̊֐����g�p����ƁC�����o�ϐ����X�V�����D
//	void CalclateJointPos(const RobotStateNode& _node);
//
//	//�ycalclateJointPos�֐����g�p���Ă���g������!!�zfemur joint (��2�֐�) �̍��W��Ԃ��D��]���l�������O���[�o�����W.
//	designlab::Vector3 GetGlobalFemurJointPos(const RobotStateNode& _node, const int _leg_num) const;
//
//	//�ycalclateJointPos�֐����g�p���Ă���g������!!�ztibia joint (��3�֐�) �̍��W��Ԃ��D��]���l�������O���[�o�����W.
//	designlab::Vector3 GetGlobalTibiaJointPos(const RobotStateNode& _node, const int _leg_num) const;
//
//	//�ÓI�����o�ϐ��� m_leg_max_r , m_leg_min_r �̒l���v�Z���ď���������D���̊֐����̂��ÓI�Ȃ̂�SystemMain�ň�x�������s����΂悢�D
//	static void InitLegR();
//
//	//�yinitLegR�֐����g�p���Ă���g������!!�z�t��������r��܂ł�Z���W�̍��𗘗p���āC�t��������r��܂ł̍ő唼�a���擾����D�t�����̕�����ɂ���O��œ����܂��D
//	float GetMaxLegR(const float _coxa_z_to_leg_z) const;
//
//	//�yinitLegR�֐����g�p���Ă���g������!!�z�t��������r��܂ł�Z���W�̍��𗘗p���āC�t��������r��܂ł̍ŏ����a���擾����D�t�����̕�����ɂ���O��œ����܂��D
//	float GetMinLegR(const float _coxa_z_to_leg_z) const;
//
//	//! @brief �r�̊����`�F�b�N����D
//	//! @param _node �m�[�h���
//	//! @return �����Ă���ꍇ��true��Ԃ��D
//	bool IsLegInterfering(const RobotStateNode& _node) const;
//
//	//! @brief �r�����͈͓����`�F�b�N����D���x�d���̂��߁C��������Ƃ����v�Z���s���D
//	//! @param _node �m�[�h���
//	//! @param leg_num �r�ԍ�
//	//! @return ���͈͓��Ȃ�true��Ԃ��D
//	bool IsLegInRange(const RobotStateNode& _node, const int _leg_num) const;
//
//	//! @brief �r�����͈͓����`�F�b�N����D���x�d���̂��߁C��������Ƃ����v�Z���s���D
//	//! @param local_leg_pos �r�̕t�����̍��W�����_�Ƃ����r��̍��W
//	//! @param leg_num �r�ԍ�
//	//! @return ���͈͓��Ȃ�true��Ԃ��D
//	bool IsLegInRange(const designlab::Vector3& local_leg_pos, const int leg_num) const;
//
//	//! @brief �S�Ă̐ڒn�r�����͈͓����`�F�b�N����D���x�d���̂��߁C��������Ƃ����v�Z���s���D
//	//! @param _node �m�[�h���
//	//! @return ���͈͓��Ȃ�true��Ԃ��D
//	bool IsAllLegInRange(const RobotStateNode& _node) const;
//
//	//! @brief �]�΂Ȃ��p�����ǂ������ׂ�D
//	//! @param [in] _node �m�[�h���
//	//! @return �]�΂Ȃ��p���Ȃ�true��Ԃ��D
//	bool IsAblePause(const RobotStateNode& _node) const;
//
//	//! @brief �ÓI����]�T���v�Z����D
//	//! @param [in] _node �m�[�h���
//	//! @return float �ÓI����]�T
//	//! @note �g������̃v���O�����ɏ����Ă������܂܂��ڂ��Ă��邪�CisAblePause�֐��Ɠ����悤�Ȃ��Ƃ����Ă���̂ŁCisAblePause�֐������������ׂ����Ǝv���D
//	float CalculateStaticMargin(const RobotStateNode& node) const;
//
//private:
//
//	//! @brief coxa joint (�r�̕t����)�̍��W��Ԃ��D�d�S�����_�Ƃ��郍�[�J�����W�D
//	//! @param [in] leg_num �r�ԍ�
//	//! @return coxa joint�̃��[�J�����W
//	designlab::Vector3 GetLocalCoxaJointPos(const int leg_num) const
//	{
//		if (leg_num == 0) { return designlab::Vector3(HexapodConst::BODY_FRONT_LENGTH, -HexapodConst::BODY_FRONT_WIDTH, 0.0f); }	// �r0 �E��
//		else if (leg_num == 1) { return designlab::Vector3(0.0f, -HexapodConst::BODY_CENTER_WIDTH, 0.0f); }	// �r1 �E��
//		else if (leg_num == 2) { return designlab::Vector3(-HexapodConst::BODY_REAR_LENGTH, -HexapodConst::BODY_REAR_WIDTH, 0.0f); }	// �r2 �E��
//		else if (leg_num == 3) { return designlab::Vector3(-HexapodConst::BODY_REAR_LENGTH, HexapodConst::BODY_REAR_WIDTH, 0.0f); }	// �r3 ����
//		else if (leg_num == 4) { return designlab::Vector3(0.0f, HexapodConst::BODY_CENTER_WIDTH, 0.0f); }	// �r4 ����
//		else if (leg_num == 5) { return designlab::Vector3(HexapodConst::BODY_FRONT_LENGTH, HexapodConst::BODY_FRONT_WIDTH, 0.0f); }	// �r5 ����
//
//		return designlab::Vector3(0, 0, 0);
//	}
//
//
//	constexpr static int kMaxDifZ = 200;	//�t��������r��܂ł�Z���W�̍��̍ő�l
//
//
//	designlab::Vector3 local_femurjoint_pos_[HexapodConst::kLegNum];	//FemurJoint(��2�֐�)�̈ʒu�D�r�̕t���������_�Ƃ��郍�[�J�����W�DcalclateJointPos�֐��Œl���Z�b�g����D
//	designlab::Vector3 local_tibiajoint_pos_[HexapodConst::kLegNum];	//TibiaJoint(��3�֐�)�̈ʒu�D�r�̕t���������_�Ƃ��郍�[�J�����W�DcalclateJointPos�֐��Œl���Z�b�g����D
//
//	static float leg_max_r_[MAX_DIF_Z];	//�d�S��������r�ʒu�����������́C�r�̎�肤��ő唼�a���L�^�������́D���� Leg_ROM_R
//	static float leg_min_r_[MAX_DIF_Z];	//�d�S��������r�ʒu�����������́C�r�̎�肤��ŏ����a���L�^�������́D���� Leg_ROM_R
//};