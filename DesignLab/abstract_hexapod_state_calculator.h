#pragma once

#include <vector>

#include "designlab_vector.h"
#include "designlab_rotator.h"
#include "node.h"
#include "hexapod_const.h"



struct SHexapodJointState
{
	SHexapodJointState()
	{
		joint_position.resize(6);
		joint_angle.resize(6);
	}

	//! �֐߂̈ʒu�D�t�������珉�߂āC�r��̏��ɕ���ł���D2�ԋr�̕t�����̍��W��joint_position[2][0]�ł���D@n ���̍��W�̓O���[�o�����W�n�ł���D
	std::vector<std::vector<dl_vec::SVector>> joint_position;

	//! �֐߂̊p�x�D�t�������珉�߂āC�r��̏��ɕ���ł���D2�ԋr�̕t�����̊p�x��joint_angle[2][0]�ł���D@n ���̊p�x��rad�ł���D
	std::vector<std::vector<float>> joint_angle;
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
	AbstractHexapodStateCalculator() {};
	virtual ~AbstractHexapodStateCalculator() = default;


	//! @brief �S�Ă̊֐߂̃O���[�o�����W�ƁC�p�x���v�Z����D�d�����̂ŃO���t�T����C�`�揈�����Ƀ��[�v�Ŏg�p���邱�Ƃ͐������Ȃ��D
	//! @param [in] node �m�[�h�̏��D
	//! @param [out] joint_state �֐߂̏�ԁD
	//! @return �v�Z�ɐ���������true�D���s������false�D
	virtual bool calculateAllJointState(const SNode& node, SHexapodJointState* const joint_state) const = 0;


	virtual dl_vec::SVector getLocalLegBasePosition(const int leg_index) const = 0;

	virtual dl_vec::SVector getLocalLegPosition(const SNode& node, const int leg_index) = 0;


	virtual dl_vec::SVector getGlobalLegBasePosition(const int leg_index) const = 0;

	virtual dl_vec::SVector getGlobalLegPosition(const SNode& node, const int leg_index) = 0;

private:

	static constexpr bool DO_CHECK_LEG_INDEX = false;	//!< �r�ԍ��̃`�F�b�N�����邩�ǂ����D�����̂��߂ɁC�f�o�b�O���ȊO��false�ɂ��邱�ƁD


	dl_vec::SVector m_local_leg_base_pos[HexapodConst::LEG_NUM];	//!< �r�̕t�����̍��W( leg base position)�D���{�b�g�̏d�S�����_�C�����Ă��������x���Ƃ������[�J��(���{�b�g)���W�n�ł���D
};

