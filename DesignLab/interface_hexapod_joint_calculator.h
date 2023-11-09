//! @file interface_hexapod_joint_calculator.h
//! @brief 6�r���{�b�g�̊Ԑڊp�x��W���C���g�̈ʒu���v�Z���鏈���̃C���^�[�t�F�[�X�D
 
#ifndef	DESIGNLAB_INTERFACE_HEXAPOD_JOINT_CALCULATOR_H
#define	DESIGNLAB_INTERFACE_HEXAPOD_JOINT_CALCULATOR_H


#include <array>

#include "robot_state_node.h"


//! @struct HexapodJointState
//! @brief ���{�b�g�̊֐߂̏�Ԃ�\���\���́D
//! @n �֐߂̈ʒu�Ɗp�x��\���D
struct HexapodJointState
{
	//! �֐߂̈ʒu�D�t�������珉�߂āC�r��̏��ɕ���ł���D�r�̕t�����̍��W��joint_position[0]�ł���D
	//! @n ���̍��W�͋r�̕t���������_�Ƃ��C���̓��{�b�g���W�n�Ɠ��l�ȋr���W�n�ł���D�P�ʂ� [mm]�D
	std::vector<designlab::Vector3> local_joint_position;

	//! �֐߂̈ʒu�D�t�������珉�߂āC�r��̏��ɕ���ł���D�r�̕t�����̍��W��joint_position[0]�ł���D
	//! @n ���̍��W�̓O���[�o�����W�n�ł���D�P�ʂ� [mm]�D
	std::vector<designlab::Vector3> global_joint_position;

	//! �֐߂̊p�x�D�t�������珉�߂āC�r��̏��ɕ���ł���D�r�̕t�����̊p�x��joint_angle[0]�ł���D
	//! @n ���̊p�x�̒P�ʂ� [rad] �ł���D
	std::vector<float> joint_angle;

	//! �ڕW���W�ɋr���͂��Ȃ��Ȃ��false�ɂȂ�
	bool is_in_range{false};
};


//! class IHexapodJointCalculator
//! @brief �Ԑڊp�x��p���x���v�Z���鏈���̃C���^�[�t�F�[�X�D
class IHexapodJointCalculator
{
public:

	virtual ~IHexapodJointCalculator() = default;

	//! @brief �S�Ă̊֐߂̃O���[�o�����W�ƁC�p�x���v�Z����D
	//! @n �d�����̂ŃO���t�T����C�`�揈�����Ƀ��[�v�Ŏg�p���邱�Ƃ͐������Ȃ��D
	//! @n �Ԑڂ̉��͈͊O�܂œ����Ă��܂��ꍇ�ł��C������Ԃ��D
	//! @n �ڕW���W�ɓ͂��Ȃ��ꍇ�́C�ł��߂����W�܂ŋr��L�΂��C�߂�l��is_in_range��false�ɂ���D
	//! @param [in] node �m�[�h�̏��D
	//! @return �S�Ă̊֐߂̏�ԁD
	virtual std::array<HexapodJointState, HexapodConst::kLegNum> CalculateAllJointState(const RobotStateNode& node) const = 0;

	//! @brief �w�肵���r�̊֐߂̃O���[�o�����W�ƁC�p�x���v�Z����D
	//! @n �d�����̂ŃO���t�T����C�`�揈�����Ƀ��[�v�Ŏg�p���邱�Ƃ͐������Ȃ��D
	//! @n �Ԑڂ̉��͈͊O�܂œ����Ă��܂��ꍇ�ł��C������Ԃ��D
	//! @n �ڕW���W�ɓ͂��Ȃ��ꍇ�́C�ł��߂����W�܂ŋr��L�΂��C�߂�l��is_in_range��false�ɂ���D
	//! @param [in] node �m�[�h�̏��D
	//! @param [in] leg_index �r�̃C���f�b�N�X�D
	//! @return �w�肵���r�̊֐߂̏�ԁD
	virtual HexapodJointState CalculateJointState(const RobotStateNode& node, const int leg_index) const = 0;

	//! @brief HexapodJointState���������v�Z�ł��Ă��邩�𒲂ׂ�D
	//! @n �ڕW���W�ɓ͂��Ȃ��ꍇ��C�Ԑڂ̉��͈͊O�܂œ����Ă��܂��ꍇ�C�߂�l��false�ɂȂ�D
	//! @param [in] node �m�[�h�̏��D
	//! @param [in] joint_state �֐߂̏�ԁD
	//! @return �v�Z���������ł��Ă���Ȃ��true��Ԃ�
	virtual bool IsVaildAllJointState(const RobotStateNode& node, const std::array<HexapodJointState, HexapodConst::kLegNum>& joint_state) const = 0;

	//! @brief �w�肵���r��HexapodJointState���������v�Z�ł��Ă��邩�𒲂ׂ�D
	//! @n �ڕW���W�ɓ͂��Ȃ��ꍇ��C�Ԑڂ̉��͈͊O�܂œ����Ă��܂��ꍇ�C�߂�l��false�ɂȂ�D
	//! @param [in] node �m�[�h�̏��D
	//! @param [in] joint_state �֐߂̏�ԁD
	//! @param [in] leg_index �r�̃C���f�b�N�X�D
	//! @return �v�Z���������ł��Ă���Ȃ��true��Ԃ�
	virtual bool IsVaildJointState(const RobotStateNode& node, const HexapodJointState& joint_state, const int leg_index) const = 0;
};


#endif