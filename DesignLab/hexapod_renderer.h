#pragma once

#include <memory>

#include "node.h"
#include "hexapod_const.h"
#include "abstract_hexapod_state_calculator.h"


//! @class HexapodRenderer
//! @date 2023/08/09
//! @author ���J��
//! @brief ���{�b�g�̕`����s���N���X�D
class HexapodRenderer
{
public:
	HexapodRenderer(std::shared_ptr<AbstractHexapodStateCalculator> calc);
	~HexapodRenderer() = default;

	//! @brief ���{�b�g�̏�Ԃ��X�V����D
	//! @param [in] node �`�悷�郍�{�b�g�̏��
	void setNode(const SNode& node);

	//! @brief ���{�b�g��3D��Ԃɕ`�悷��D
	//! @param [in] node �`�悷�郍�{�b�g�̏��
	void draw(const SNode& node) const;

private:

	bool isAbleCoxaLeg(const dl_vec::SVector& coxa_joint, const dl_vec::SVector& femur_joint) const;
	bool isAbleFemurLeg(const dl_vec::SVector& femur_joint, const dl_vec::SVector& tibia_joint) const;
	bool isAbleTibiaLeg(const dl_vec::SVector& tibia_joint, const dl_vec::SVector& leg_joint) const;


	const unsigned int COLOR_BODY;			// ���̂̐F
	const unsigned int COLOR_LEG;			// �r�̐F
	const unsigned int COLOR_LIFTED_LEG;	// �V�r���Ă���r�̐F
	const unsigned int COLOR_JOINT;			// �W���C���g�̐F
	const unsigned int COLOR_LIFTED_JOINT;	// �V�r���Ă���W���C���g�̐F
	const unsigned int COLOR_LEG_BASE;		// �r�̊�̐F

	const int CAPSULE_DIV_NUM;				// ���{�b�g�̃��f���̉~�����ǂꂾ���ׂ����`�悷�邩�D4 �` 20���炢�����傤�ǂ悢�Ǝv���D
	const int SPHERE_DIV_NUM;				// ���{�b�g�̃��f���̋����ǂꂾ���ׂ����`�悷�邩�D16 �` 32���炢�����傤�ǂ悢�Ǝv���D
	const float LEG_R = 10.0f;				// �r�̔��a�D���̃N���X�ł͋r���~���ɋߎ����ĕ`�悵�Ă���D�`�掞�̃f�[�^�̂��߁C�����ω������Ă��V�~�����[�V�����ɉe���͂Ȃ��D
	const float JOINT_R = 20.0f;			// �W���C���g�̔��a�D�`�掞�̃f�[�^�̂��߁C�����ω������Ă��V�~�����[�V�����ɉe���͂Ȃ��D

	const bool DO_OUTPUT_DEBUG_LOG = false;	// �r��Ԃ𕶎���ŏo�͂��邩�ǂ���


	std::shared_ptr<AbstractHexapodStateCalculator> mp_calculator;	// ���{�b�g�̏�Ԃ��v�Z����N���X

	SNode m_draw_node;						// �`�悷�郍�{�b�g�̏��

	SHexapodJointState m_draw_joint_state[HexapodConst::LEG_NUM];	// �`�悷�郍�{�b�g�̃W���C���g�̏��
};


//! @file hexapod_renderer.h
//! @date 2023/08/09
//! @author ���J��
//! @brief ���{�b�g�̕`����s��HexapodRenderer�N���X�D
//! @n �s�� : @lineinfo
