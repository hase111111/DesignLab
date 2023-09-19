#pragma once

#include "node.h"

#include "hexapod_state_calculator.h"


//! @class StabilityMarginRenderer
//! @date 2023/08/30
//! @author ���J��
//! @brief ���{�b�g�̈��萫�}�[�W����`�悷��N���X�D
//! @n �ڒn�_�𓊉e�������p�`�̓����ɁC�d�S�������Ă��邩�ǂ����ň��萫�𔻒肷��DStability Margin �ŃO�O��Əڂ����������o�Ă���D
class StabilityMarginRenderer final
{
public:

	StabilityMarginRenderer();
	~StabilityMarginRenderer() = default;

	void Draw(const SNode& node) const;


private:
	const unsigned int kMarginColor;		//!< ���萫�}�[�W���̐F�D

	const unsigned int kMarginErrorColor;	//!< ����łȂ��Ƃ��̐F

	const int kAlpha;						//!< �����x�D


	HexapodStateCalclator_Old m_hexapod_state_calclator;	//!< ���{�b�g�̏�Ԃ��v�Z����N���X�D
};



//! @file stability_margin_renderer.h
//! @date 2023/08/30
//! @author ���J��
//! @brief StabilityMarginRenderer�N���X�̎������L�q�����t�@�C���D
//! @n �s�� : @lineinfo
