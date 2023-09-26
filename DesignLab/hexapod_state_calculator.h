//#pragma once
//
//#include "designlab_vector3.h"
//#include "designlab_euler.h"
//#include "node.h"
//#include "hexapod_const.h"
//
//
////! @class HexapodStateCalclator_Old
////! @date 2023/08/13
////! @author ���J��
////! @brief ���{�b�g�̍��W���ԂȂǂ̒l���v�Z����N���X�D�� Hexapod�N���X���y���������́D
//class HexapodStateCalclator_Old
//{
//public:
//	HexapodStateCalclator_Old();
//
//	static constexpr designlab::Vector3 getLocalBaseLegPos(const int _leg_num, const float _z)
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
//	designlab::Vector3 convertLocalLegPos(const SNode& node, const designlab::Vector3& global_pos, const int leg_num, const bool do_consider_rot) const;
//
//	//! @brief �r���W�͋r�̕t���������_�Ƃ������W�n�Ȃ̂ŁC������O���[�o�����W�ɕϊ�����D
//	//! @param [in] _node ���{�b�g�̏�Ԃ�\���m�[�h
//	//! @param [in] leg_num �r�ԍ� 0�`5
//	//! @param [in] _consider_rot ��]���l�����邩�ǂ���
//	//! @return designlab::Vector3 �O���[�o�����W�̋r���W
//	inline designlab::Vector3 getGlobalLegPos(const SNode& _node, const int _leg_num, const bool _consider_rot) const
//	{
//		if (_consider_rot == true) { return rotVector(getLocalCoxaJointPos(_leg_num) + _node.leg_pos[_leg_num], _node.rot) + _node.global_center_of_mass; }
//		else { return _node.global_center_of_mass + getLocalCoxaJointPos(_leg_num) + _node.leg_pos[_leg_num]; }
//	}
//
//	//! @brief �r�ʒu�͗��U������Đ��䂳��邪�C���̎���4�̈ʒu���O���[�o�����W�ŏo�͂���D
//	//! @param [in] _node ���{�b�g�̏�Ԃ�\���m�[�h
//	//! @param [in] leg_num �r�ԍ� 0�`5
//	//! @param [in] _consider_rot ��]���l�����邩�ǂ���
//	//! @return designlab::Vector3 �O���[�o�����W�̋r�̊�n�_�̍��W
//	inline designlab::Vector3 getGlobalLegBasePos(const SNode& _node, const int _leg_num, const bool _consider_rot) const
//	{
//		if (_consider_rot == true) { return rotVector(getLocalCoxaJointPos(_leg_num) + _node.leg_reference_pos[_leg_num], _node.rot) + _node.global_center_of_mass; }
//		else { return _node.global_center_of_mass + getLocalCoxaJointPos(_leg_num) + _node.leg_reference_pos[_leg_num]; }
//	}
//
//	// @brief coxa joint (�r�̕t���� : ��1�֐�) ���O���[�o�����W�ŕԂ��D
//	// @param [in] _node ���{�b�g�̏�Ԃ�\���m�[�h
//	// @param [in] leg_num �r�ԍ� 0�`5
//	// @param [in] _consider_rot ��]���l�����邩�ǂ���
//	// @return designlab::Vector3 �O���[�o�����W�̋r�̕t�����̍��W
//	inline designlab::Vector3 getGlobalCoxaJointPos(const SNode& _node, const int _leg_num, const bool _consider_rot) const
//	{
//		if (_consider_rot == true) { return rotVector(getLocalCoxaJointPos(_leg_num), _node.rot) + _node.global_center_of_mass; }
//		else { return _node.global_center_of_mass + getLocalCoxaJointPos(_leg_num); }
//	}
//
//	//! @brief �m�[�h�̏��͌��݂̋r�ʒu�Əd�S�ʒu���������Ȃ��̂ŁC�W���C���g���ǂ��ɂ��邩��������Ȃ��D����Ă��̊֐��Ōv�Z����D<br>
//	//! �O�p�֐��𑽂��g�p����̂ŁC�v�Z�ʂ������D
//	//! @param [in] _node ���{�b�g�̏�Ԃ�\���m�[�h
//	//! @details ���̊֐����g�p����ƁC�����o�ϐ����X�V�����D
//	void calclateJointPos(const SNode& _node);
//
//	//�ycalclateJointPos�֐����g�p���Ă���g������!!�zfemur joint (��2�֐�) �̍��W��Ԃ��D��]���l�������O���[�o�����W.
//	designlab::Vector3 getGlobalFemurJointPos(const SNode& _node, const int _leg_num) const;
//
//	//�ycalclateJointPos�֐����g�p���Ă���g������!!�ztibia joint (��3�֐�) �̍��W��Ԃ��D��]���l�������O���[�o�����W.
//	designlab::Vector3 getGlobalTibiaJointPos(const SNode& _node, const int _leg_num) const;
//
//	//�ÓI�����o�ϐ��� m_leg_max_r , m_leg_min_r �̒l���v�Z���ď���������D���̊֐����̂��ÓI�Ȃ̂�SystemMain�ň�x�������s����΂悢�D
//	static void initLegR();
//
//	//�yinitLegR�֐����g�p���Ă���g������!!�z�t��������r��܂ł�Z���W�̍��𗘗p���āC�t��������r��܂ł̍ő唼�a���擾����D�t�����̕�����ɂ���O��œ����܂��D
//	float getMaxLegR(const float _coxa_z_to_leg_z) const;
//
//	//�yinitLegR�֐����g�p���Ă���g������!!�z�t��������r��܂ł�Z���W�̍��𗘗p���āC�t��������r��܂ł̍ŏ����a���擾����D�t�����̕�����ɂ���O��œ����܂��D
//	float getMinLegR(const float _coxa_z_to_leg_z) const;
//
//	//! @brief �r�̊����`�F�b�N����D
//	//! @param _node �m�[�h���
//	//! @return �����Ă���ꍇ��true��Ԃ��D
//	bool IsLegInterfering(const SNode& _node) const;
//
//	//! @brief �r�����͈͓����`�F�b�N����D���x�d���̂��߁C��������Ƃ����v�Z���s���D
//	//! @param _node �m�[�h���
//	//! @param leg_num �r�ԍ�
//	//! @return ���͈͓��Ȃ�true��Ԃ��D
//	bool IsLegInRange(const SNode& _node, const int _leg_num) const;
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
//	bool isAllLegInRange(const SNode& _node) const;
//
//	//! @brief �]�΂Ȃ��p�����ǂ������ׂ�D
//	//! @param [in] _node �m�[�h���
//	//! @return �]�΂Ȃ��p���Ȃ�true��Ԃ��D
//	bool isAblePause(const SNode& _node) const;
//
//	//! @brief �ÓI����]�T���v�Z����D
//	//! @param [in] _node �m�[�h���
//	//! @return float �ÓI����]�T
//	//! @note �g������̃v���O�����ɏ����Ă������܂܂��ڂ��Ă��邪�CisAblePause�֐��Ɠ����悤�Ȃ��Ƃ����Ă���̂ŁCisAblePause�֐������������ׂ����Ǝv���D
//	float calculateStaticMargin(const SNode& _node) const;
//
//private:
//
//	//! @brief coxa joint (�r�̕t����)�̍��W��Ԃ��D�d�S�����_�Ƃ��郍�[�J�����W�D
//	//! @param [in] leg_num �r�ԍ�
//	//! @return coxa joint�̃��[�J�����W
//	designlab::Vector3 getLocalCoxaJointPos(const int leg_num) const
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
//	constexpr static int MAX_DIF_Z = 200;	//�t��������r��܂ł�Z���W�̍��̍ő�l
//
//
//	designlab::Vector3 m_local_femurjoint_pos[HexapodConst::LEG_NUM];	//FemurJoint(��2�֐�)�̈ʒu�D�r�̕t���������_�Ƃ��郍�[�J�����W�DcalclateJointPos�֐��Œl���Z�b�g����D
//	designlab::Vector3 m_local_tibiajoint_pos[HexapodConst::LEG_NUM];	//TibiaJoint(��3�֐�)�̈ʒu�D�r�̕t���������_�Ƃ��郍�[�J�����W�DcalclateJointPos�֐��Œl���Z�b�g����D
//
//	static float m_leg_max_r[MAX_DIF_Z];	//�d�S��������r�ʒu�����������́C�r�̎�肤��ő唼�a���L�^�������́D���� Leg_ROM_R
//	static float m_leg_min_r[MAX_DIF_Z];	//�d�S��������r�ʒu�����������́C�r�̎�肤��ŏ����a���L�^�������́D���� Leg_ROM_R
//};
//
//
////! @file hexapod_state_calculator.h
////! @date 2023/08/13
////! @author ���J��
////! @brief HexapodStateCalclator�N���X�̃w�b�_�t�@�C���D
////! @n �s�� : @lineinfo
