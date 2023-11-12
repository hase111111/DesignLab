//! @file interface_hexapod_joint_calculator.h
//! @brief 6�r���{�b�g�̊Ԑڊp�x��W���C���g�̈ʒu���v�Z���鏈���̃C���^�[�t�F�[�X�D
 
#ifndef	DESIGNLAB_INTERFACE_HEXAPOD_JOINT_CALCULATOR_H_
#define	DESIGNLAB_INTERFACE_HEXAPOD_JOINT_CALCULATOR_H_


#include <array>

#include "robot_state_node.h"


//! @struct HexapodJointState
//! @brief ���{�b�g�̊֐߂̏�Ԃ�\���\���́D
//! @n �֐߂̈ʒu�Ɗp�x��\���D
struct HexapodJointState
{
	//! �֐߂̈ʒu�D�t�������珉�߂āC�r��̏��ɕ���ł���D�r�̕t�����̍��W��joint_position[0]�ł���D
	//! @n ���̍��W�͋r�̕t���������_�Ƃ��C���̓��{�b�g���W�n�Ɠ��l�ȋr���W�n ( leg coordinate ) �ł���D�P�ʂ� [mm]�D
	std::vector<designlab::Vector3> joint_pos_leg_coordinate;

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
	[[nodiscard]] virtual std::array<HexapodJointState, HexapodConst::kLegNum> CalculateAllJointState(const RobotStateNode& node) const noexcept = 0;

	//! @brief �w�肵���r�̊֐߂̃O���[�o�����W�ƁC�p�x���v�Z����D
	//! @n �d�����̂ŃO���t�T����C�`�揈�����Ƀ��[�v�Ŏg�p���邱�Ƃ͐������Ȃ��D
	//! @n �Ԑڂ̉��͈͊O�܂œ����Ă��܂��ꍇ�ł��C������Ԃ��D
	//! @n �ڕW���W�ɓ͂��Ȃ��ꍇ�́C�ł��߂����W�܂ŋr��L�΂��C�߂�l��is_in_range��false�ɂ���D
	//! @param [in] leg_index �r�̃C���f�b�N�X�D
	//! @param [in] leg_pos �r����W�C�r���W�n�D
	//! @return �w�肵���r�̊֐߂̏�ԁD
	[[nodiscard]] virtual HexapodJointState CalculateJointState(const int leg_index, const designlab::Vector3& leg_pos) const noexcept = 0;

	//! @brief HexapodJointState���������v�Z�ł��Ă��邩�𒲂ׂ�D
	//! @n �ڕW���W�ɓ͂��Ȃ��ꍇ��C�Ԑڂ̉��͈͊O�܂œ����Ă��܂��ꍇ�C�߂�l��false�ɂȂ�D
	//! @param [in] node �m�[�h�̏��D
	//! @param [in] joint_state �֐߂̏�ԁD
	//! @return �v�Z���������ł��Ă���Ȃ��true��Ԃ�
	[[nodiscard]] virtual bool IsVaildAllJointState(const RobotStateNode& node, 
		const std::array<HexapodJointState, HexapodConst::kLegNum>& joint_state) const noexcept = 0;

	//! @brief �w�肵���r��HexapodJointState���������v�Z�ł��Ă��邩�𒲂ׂ�D
	//! @n �ڕW���W�ɓ͂��Ȃ��ꍇ��C�Ԑڂ̉��͈͊O�܂œ����Ă��܂��ꍇ�C�߂�l��false�ɂȂ�D
	//! @param [in] leg_index �r�̃C���f�b�N�X�D
	//! @param [in] leg_pos �r����W�C�r���W�n�D
	//! @param [in] joint_state �֐߂̏�ԁD
	//! @return �v�Z���������ł��Ă���Ȃ��true��Ԃ�
	[[nodiscard]] virtual bool IsVaildJointState(const int leg_index, const designlab::Vector3& leg_pos, const HexapodJointState& joint_state) const noexcept = 0;
};


#endif	// DESIGNLAB_INTERFACE_HEXAPOD_JOINT_CALCULATOR_H_