//! @file abstract_hexapod_state_calculator.h
//! @brief ���{�b�g�̏�Ԃ��v�Z����N���X�̒��ۃN���X�D
//! @details ���̃N���X���p�����āC��̓I�ȃ��{�b�g(�Ⴆ��phantomX�Ƃ�AUSRA�Ƃ�)�̏�Ԃ��v�Z����N���X���쐬����D
//! @n ���{�b�g�̎p�����v�Z����s����C���W�n����������邱�Ƃɒ��ӁD
//! @n 3�̍��W�n�����݂��Ă���C
//! @n
//! @n [1] �O���[�o�����W�n(���[���h���W�n)
//! @n		�}�b�v�̌��_�����_�Ƃ�����W�n�C���W���̓}�b�v�̍��W���Ɠ����D
//! @n
//! @n [2] ���[�J�����W�n(���{�b�g���W�n)
//! @n		���{�b�g�̏d�S�����_�Ƃ�����W�n�C���W���̓��{�b�g�̎p���ɍ��킹��D
//! @n 
//! @n [3] �r���W�n 
//! @n		�r�̕t���������_�Ƃ�����W�n�C���W���̓��{�b�g�̎p���ɍ��킹��(���[�J�����W�n�Ɠ���)�D
//! @n		�^���w�̋��ȏ��Ƃ��ǂފ����C���̍��W�n�̎����Ƃ͈Ⴄ�̂����C��s�����Ŏg��ꂽ��@�ł���C
//! @n		���p����Ȃ��̂ł��̂܂܁D
//! @n
//! @n �ϊ��́C�r���W�n���O���[�o�����W�n�Ȃ�΁C
//! @n	( �r���W�n�̍��W �{ ���{�b�g�̌��_����r�̕t�����܂ł̍��W(���[�J��) ) * �p���̋t��] �{ ���{�b�g�̏d�S�̍��W(�O���[�o��)


#ifndef DESIGNLAB_ABSTRACT_HEXAPOD_STATE_CALCULATOR_H_
#define DESIGNLAB_ABSTRACT_HEXAPOD_STATE_CALCULATOR_H_

#include <array>
#include <optional>
#include <vector>

#include "designlab_vector3.h"
#include "designlab_euler.h"
#include "leg_state.h"
#include "robot_state_node.h"
#include "hexapod_const.h"


//! @struct HexapodJointState
//! @brief ���{�b�g�̊֐߂̏�Ԃ�\���\���́D
//! @details �֐߂̈ʒu�Ɗp�x��\���D
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
	bool is_in_range = false;
};


//! @class AbstractHexapodStateCalculator
//! @brief ���{�b�g�̏�Ԃ��v�Z����N���X�̒��ۃN���X�D
//! @details ���̃N���X���p�����āC��̓I�ȃ��{�b�g(�Ⴆ��phantomX�Ƃ�AUSRA�Ƃ�)�̏�Ԃ��v�Z����N���X���쐬����D
//! @n ���̃N���X��p���ă��{�b�g�̃X�e�[�g��\�����Ă���̂ŁCHexapodPresenter�Ƃł������ق����ǂ���������Ȃ�
//! @n �X���b�h�Z�[�t�ȃN���X�ɂ��邱�ƁDhttps://yohhoy.hatenablog.jp/entry/2013/12/15/204116
class AbstractHexapodStateCalculator
{
public:
	AbstractHexapodStateCalculator() = default;
	virtual ~AbstractHexapodStateCalculator() = default;


	//! @brief �S�Ă̊֐߂̃O���[�o�����W�ƁC�p�x���v�Z����D�d�����̂ŃO���t�T����C�`�揈�����Ƀ��[�v�Ŏg�p���邱�Ƃ͐������Ȃ��D
	//! @n �ڕW���W�ɓ͂��Ȃ��ꍇ��C�Ԑڂ̉��͈͊O�܂œ����Ă��܂��ꍇ�ł��C������Ԃ��D
	//! @n �r���͂��Ȃ��ꍇ�́C�߂�l��is_in_range��false�ɂȂ�D
	//! @param [in] node �m�[�h�̏��D
	//! @param [out] joint_state �֐߂̏�ԁD
	virtual void CalculateAllJointState(const RobotStateNode& node, std::array<HexapodJointState, HexapodConst::kLegNum>* joint_state) const = 0;

	virtual bool IsVaildJointState(const RobotStateNode& node, const std::array<HexapodJointState, HexapodConst::kLegNum>& joint_state) const = 0;

	//! @brief �O���[�o�����W�n���r���W�n�ɕϊ�����D
	//! @param [in] leg_index �r�ԍ��D
	//! @param [in] global_pos �O���[�o�����W�n�̋r����W�D
	//! @param [in] global_center_of_mass ���{�b�g�̏d�S�̍��W�D�O���[�o�����W�n�D
	//! @param [in] robot_rot ���{�b�g�̎p���D�p�x��rad.
	//! @param [in] consider_rot ���{�b�g�̎p�����l�����邩�ǂ����Dfalse�Ȃ��]���l�����Ȃ��D
	//! @return designlab::Vector3 �r���W�n�̋r����W�D�r����W�n�Ƃ͋r�̕t���������_�Ƃ��C���̓��{�b�g���W�n�Ɠ��l�ȍ��W�n�D
	virtual designlab::Vector3 ConvertGlobalToLegPosition(int leg_index, const designlab::Vector3& global_pos, 
		const designlab::Vector3& global_center_of_mass, const designlab::EulerXYZ& robot_rot, bool consider_rot) const = 0;


	//! @brief �V�r����ʒu��Ԃ��C�r���W�n
	//! @param [in] leg_index �r�ԍ��D
	//! @return designlab::Vector3 �V�r����ʒu�D�r���W�n
	virtual designlab::Vector3 GetFreeLegPosition(int leg_index) const = 0;


	//! @brief �r�̕t�����̍��W( leg base position)���擾����D���[�J��(���{�b�g)���W�n
	//! @param [in] leg_index �r�ԍ��D
	//! @return designlab::Vector3 �r�̕t�����̍��W�D���[�J��(���{�b�g)���W�n
	virtual designlab::Vector3 GetLocalLegBasePosition(int leg_index) const = 0;

	//! @brief �r��̍��W���擾����D���[�J��(���{�b�g)���W�n
	//! @param [in] leg_index �r�ԍ��D
	//! @param [in] leg_pos �r���W�n�ɂ�����r��̍��W�D�r����W�n�Ƃ͋r�̕t���������_�Ƃ��C���̓��{�b�g���W�n�Ɠ��l�ȍ��W�n�D
	//! @return designlab::Vector3 �r��̍��W�D���[�J�����W�n
	virtual designlab::Vector3 GetLocalLegPosition(int leg_index, const designlab::Vector3& leg_pos) const = 0;


	//! @brief �r�̕t�����̍��W( leg base position)���擾����D�O���[�o��(���[���h)���W�n
	//! @param [in] leg_index �r�ԍ��D
	//! @param [in] global_center_of_mass ���{�b�g�̏d�S�̍��W�D�O���[�o�����W�n�D
	//! @param [in] robot_rot ���{�b�g�̎p���D�p�x��rad.
	//! @param [in] consider_rot ���{�b�g�̎p�����l�����邩�ǂ����Dfalse�Ȃ��]���l�����Ȃ��D
	//! @return designlab::Vector3 �r�̕t�����̍��W�D�O���[�o�����W�n�D
	virtual designlab::Vector3 GetGlobalLegBasePosition(int leg_index, const designlab::Vector3& global_center_of_mass, 
		const designlab::EulerXYZ& robot_rot, bool consider_rot) const = 0;

	//! @brief �r�̐�[�̍��W���擾����D�O���[�o��(���[���h)���W�n
	//! @param [in] leg_index �r�ԍ��D
	//! @param [in] leg_pos �r���W�n�ɂ�����r��̍��W�D�r����W�n�Ƃ͋r�̕t���������_�Ƃ��C���̓��{�b�g���W�n�Ɠ��l�ȍ��W�n�D
	//! @param [in] global_center_of_mass ���{�b�g�̏d�S�̍��W�D�O���[�o�����W�n�D
	//! @param [in] robot_rot ���{�b�g�̎p���D�p�x��rad.
	//! @param [in] consider_rot ���{�b�g�̎p�����l�����邩�ǂ����Dfalse�Ȃ��]���l�����Ȃ��D
	//! @return designlab::Vector3 �r��̍��W�D�O���[�o�����W�n�D
	virtual designlab::Vector3 GetGlobalLegPosition(int leg_index, const designlab::Vector3& leg_pos, 
		const designlab::Vector3& global_center_of_mass, const designlab::EulerXYZ& robot_rot, bool consider_rot) const = 0;


	//! @brief �r�����͈͓��ɂ��邩�ǂ����𔻒肷��D
	//! @param [in] leg_index �r�ԍ��D
	//! @param [in] leg_pos �r���W�n�ɂ�����r��̍��W�D�r����W�n�Ƃ͋r�̕t���������_�Ƃ��C���̓��{�b�g���W�n�Ɠ��l�ȍ��W�n�D
	//! @return bool �r�����͈͓��ɂ����true�D���͈͊O�ɂ����false�D
	virtual bool IsLegInRange(const int leg_index, const designlab::Vector3& leg_pos) const = 0;

	//! @brief �r�����̋r�Ɗ����Ă��邩�ǂ����𔻒肷��D
	//! @param [in] leg_pos �r���W�n�ɂ�����r��̍��W�̔z��D�r����W�n�Ƃ͋r�̕t���������_�Ƃ��C���̓��{�b�g���W�n�Ɠ��l�ȍ��W�n�D
	//! @return bool �r�����̋r�Ɗ����Ă����true�D�����Ă��Ȃ����false�D
	virtual bool IsLegInterfering(const std::array<designlab::Vector3, HexapodConst::kLegNum>& leg_pos) const = 0;

	//! @brief ����]�T(Stability Margin))���v�Z����D�ڂ����́u�s���n�ɂ�������s�@�B�̐ÓI���萫�]����v�Ƃ����_����ǂ�ŗ~����
	//! @n �ڒn�r���q���ō���鑽�p�`�̕ӂƏd�S�̋����̍ŏ��l���v�Z����D
	//! @param [in] leg_state �r�̏�ԁDbit�ŕ\�������C�V�r�E�ڒn�r�̏������D
	//! @param [in] leg_pos �r���W�n�ɂ�����r��̍��W�̔z��D�r����W�n�Ƃ͋r�̕t���������_�Ƃ��C���̓��{�b�g���W�n�Ɠ��l�ȍ��W�n�D
	//! @return float ����]�T�D�傫����������ƂȂ�C�܂����̒l��0�ȉ��Ȃ�]�|����D
	float CalculateStabilityMargin(const ::designlab::leg_func::LegStateBit& leg_state, const std::array<designlab::Vector3, HexapodConst::kLegNum>& leg_pos) const;
};


#endif // !DESIGNLAB_ABSTRACT_HEXAPOD_STATE_CALCULATOR_H_