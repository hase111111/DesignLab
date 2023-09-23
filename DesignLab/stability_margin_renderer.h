//! @file stability_margin_renderer.h
//! @brief ���{�b�g�̐ÓI����]�T(�x���r���p�`)��`�悷��N���X�D


#ifndef DESIGNLAB_STABILITY_MARGIN_RENDERER_H_
#define DESIGNLAB_STABILITY_MARGIN_RENDERER_H_


#include <memory>

#include "abstract_hexapod_state_calculator.h"
#include "node.h"


//! @class StabilityMarginRenderer
//! @brief ���{�b�g�̐ÓI����]�T(�x���r���p�`)��`�悷��N���X�D
//! @n �ڒn�_�𓊉e�������p�`�̓����ɁC�d�S�������Ă��邩�ǂ����ň��萫�𔻒肷��DStability Margin �ŃO�O��Əڂ����������o�Ă���D
class StabilityMarginRenderer final
{
public:

	StabilityMarginRenderer(const std::shared_ptr<const AbstractHexapodStateCalculator> calclator_ptr);
	~StabilityMarginRenderer() = default;

	//! @brief ���{�b�g�̐ÓI����]�T(�x���r���p�`)��`�悷��D
	//! @param [in] node ���{�b�g�̏�ԁD
	void Draw(const SNode& node) const;


private:
	const unsigned int kMarginColor;		//!< �x���r���p�`�̐F�D

	const unsigned int kMarginErrorColor;	//!< ����łȂ��Ƃ��̐F

	const int kAlpha;						//!< �����x�D


	const std::shared_ptr<const AbstractHexapodStateCalculator> calclator_ptr_;	//!< ���{�b�g�̏�Ԃ��v�Z����N���X�D
};


#endif // !DESIGNLAB_STABILITY_MARGIN_RENDERER_H_