#pragma once

#include <vector>

#include "designlab_vector.h"
#include "designlab_rotator.h"
#include "node.h"
#include "hexapod_const.h"



struct SHexapodJointState
{
	//! �֐߂̈ʒu�D�t�������珉�߂āC�r��̏��ɕ���ł���D�r�̕t�����̍��W��joint_position[0]�ł���D@n ���̍��W�͋r�̕t���������_�Ƃ��C���̓��{�b�g���W�n�Ɠ��l�ȋr���W�n�D
	std::vector<dl_vec::SVector> local_joint_position;

	//! �֐߂̈ʒu�D�t�������珉�߂āC�r��̏��ɕ���ł���D�r�̕t�����̍��W��joint_position[0]�ł���D@n ���̍��W�̓O���[�o�����W�n�ł���D
	std::vector<dl_vec::SVector> global_joint_position;

	//! �֐߂̊p�x�D�t�������珉�߂āC�r��̏��ɕ���ł���D�r�̕t�����̊p�x��joint_angle[0]�ł���D@n ���̊p�x�̒P�ʂ�rad�ł���D
	std::vector<float> joint_angle;
};



//! @class AbstractHexapodStateCalculator
//! @date 2023/08/30
//! @author ���J��
//! @brief ���{�b�g�̏�Ԃ��v�Z����N���X�̒��ۃN���X�D
//! @n ���̃N���X���p�����āC��̓I�ȃ��{�b�g(�Ⴆ��phantomX�Ƃ�AUSRA�Ƃ�)�̏�Ԃ��v�Z����N���X���쐬����D
//! @n �X���b�h�Z�[�t�ȃN���X�ɂ��邱�ƁDhttps://yohhoy.hatenablog.jp/entry/2013/12/15/204116
class AbstractHexapodStateCalculator
{
public:
	AbstractHexapodStateCalculator() = default;
	virtual ~AbstractHexapodStateCalculator() = default;



	//! @brief �S�Ă̊֐߂̃O���[�o�����W�ƁC�p�x���v�Z����D�d�����̂ŃO���t�T����C�`�揈�����Ƀ��[�v�Ŏg�p���邱�Ƃ͐������Ȃ��D
	//! @param [in] node �m�[�h�̏��D
	//! @param [out] joint_state �֐߂̏�ԁD
	//! @return �v�Z�ɐ���������true�D���s������false�D
	virtual bool calculateAllJointState(const SNode& node, SHexapodJointState joint_state[HexapodConst::LEG_NUM]) const = 0;



	//! @brief �y�X���b�h�Z�[�t�z�r�̕t�����̍��W( leg base position)���擾����D���[�J��(���{�b�g)���W�n
	//! @param [in] leg_index �r�ԍ��D
	//! @return dl_vec::SVector �r�̕t�����̍��W�D���[�J�����W�n
	dl_vec::SVector getLocalLegBasePosition(const int leg_index) const;

	//! @brief �y�X���b�h�Z�[�t�z�r��̍��W���擾����D���[�J��(���{�b�g)���W�n
	//! @param [in] leg_index �r�ԍ��D
	//! @param [in] leg_pos �r���W�n�ɂ�����r��̍��W�D�r����W�n�Ƃ͋r�̕t���������_�Ƃ��C���̓��{�b�g���W�n�Ɠ��l�ȍ��W�n�D
	//! @return dl_vec::SVector �r��̍��W�D���[�J�����W�n
	virtual dl_vec::SVector getLocalLegPosition(const int leg_index, const dl_vec::SVector& leg_pos) const = 0;



	//! @brief �y�X���b�h�Z�[�t�z�r�̕t�����̍��W( leg base position)���擾����D�O���[�o��(���[���h)���W�n
	//! @param [in] leg_index �r�ԍ��D
	//! @param [in] global_center_of_mass ���{�b�g�̏d�S�̍��W�D�O���[�o�����W�n�D
	//! @param [in] robot_rot ���{�b�g�̎p���D�p�x��rad.
	//! @param [in] consider_rot ���{�b�g�̎p�����l�����邩�ǂ����Dfalse�Ȃ��]���l�����Ȃ��D
	//! @return dl_vec::SVector �r�̕t�����̍��W�D�O���[�o�����W�n�D
	virtual dl_vec::SVector getGlobalLegBasePosition(const int leg_index, const dl_vec::SVector& global_center_of_mass, const dl_vec::SRotator& robot_rot, const bool consider_rot) const = 0;

	//! @brief �y�X���b�h�Z�[�t�z�r�̐�[�̍��W���擾����D�O���[�o��(���[���h)���W�n
	//! @param [in] leg_index �r�ԍ��D
	//! @param [in] leg_pos �r���W�n�ɂ�����r��̍��W�D�r����W�n�Ƃ͋r�̕t���������_�Ƃ��C���̓��{�b�g���W�n�Ɠ��l�ȍ��W�n�D
	//! @param [in] global_center_of_mass ���{�b�g�̏d�S�̍��W�D�O���[�o�����W�n�D
	//! @param [in] robot_rot ���{�b�g�̎p���D�p�x��rad.
	//! @param [in] consider_rot ���{�b�g�̎p�����l�����邩�ǂ����Dfalse�Ȃ��]���l�����Ȃ��D
	//! @return dl_vec::SVector �r��̍��W�D�O���[�o�����W�n�D
	virtual dl_vec::SVector getGlobalLegPosition(const int leg_index, const dl_vec::SVector& leg_pos, const dl_vec::SVector& global_center_of_mass, const dl_vec::SRotator& robot_rot, const bool consider_rot) const = 0;



	//! @brief �y�X���b�h�Z�[�t�z�r�����͈͓��ɂ��邩�ǂ����𔻒肷��D
	//! @param [in] leg_index �r�ԍ��D
	//! @param [in] leg_pos �r���W�n�ɂ�����r��̍��W�D�r����W�n�Ƃ͋r�̕t���������_�Ƃ��C���̓��{�b�g���W�n�Ɠ��l�ȍ��W�n�D
	//! @return bool �r�����͈͓��ɂ����true�D���͈͊O�ɂ����false�D
	virtual bool isLegInRange(const int leg_index, const dl_vec::SVector& leg_pos) const = 0;

	//! @brief �y�X���b�h�Z�[�t�z�r�����̋r�Ɗ����Ă��邩�ǂ����𔻒肷��D
	//! @param [in] leg_pos �r���W�n�ɂ�����r��̍��W�̔z��D�r����W�n�Ƃ͋r�̕t���������_�Ƃ��C���̓��{�b�g���W�n�Ɠ��l�ȍ��W�n�D
	//! @return bool �r�����̋r�Ɗ����Ă����true�D�����Ă��Ȃ����false�D
	virtual bool isLegInterfering(const dl_vec::SVector leg_pos[HexapodConst::LEG_NUM]) const = 0;

	//! @brief �y�X���b�h�Z�[�t�z����]�T(Stability Margin))���v�Z����D�ڂ����́u�s���n�ɂ�������s�@�B�̐ÓI���萫�]����v�Ƃ����_����ǂ�ŗ~����
	//! @n �ڒn�r���q���ō���鑽�p�`�̕ӂƏd�S�̋����̍ŏ��l���v�Z����D
	//! @param [in] leg_state �r�̏�ԁDbit�ŕ\�������C�V�r�E�ڒn�r�̏������D
	//! @param [in] leg_pos �r���W�n�ɂ�����r��̍��W�̔z��D�r����W�n�Ƃ͋r�̕t���������_�Ƃ��C���̓��{�b�g���W�n�Ɠ��l�ȍ��W�n�D
	//! @return float ����]�T�D�傫����������ƂȂ�C�܂����̒l��0�ȉ��Ȃ�]�|����D
	float calcStabilityMargin(const int leg_state, const dl_vec::SVector leg_pos[HexapodConst::LEG_NUM]) const;

protected:

	//! @brief �r�ԍ��̃`�F�b�N���s���Dconstexpr�Ȃ̂ŁC�R���p�C�����Ƀ`�F�b�N�����D
	//! @param [in] leg_index �r�ԍ��D0�ȏ�C6�����ł��邱�ƁD
	//! @return �r�ԍ������������true�D�������Ȃ����false�D
	constexpr bool checkLegIndex(const int leg_index) const
	{
		if (leg_index < 0 || leg_index >= HexapodConst::LEG_NUM)
		{
			return false;
		}
		return true;
	}


	static constexpr bool DO_CHECK_LEG_INDEX = false;				//!< �r�ԍ��̃`�F�b�N�����邩�ǂ����D�����̂��߂ɁC�f�o�b�O���ȊO��false�ɂ��邱�ƁD


	dl_vec::SVector m_local_leg_base_pos[HexapodConst::LEG_NUM];	//!< �r�̕t�����̍��W( leg base position)�D���{�b�g�̏d�S�����_�C�����Ă��������x���Ƃ������[�J��(���{�b�g)���W�n�ł���D
};

