//! @file stability_margin_renderer.h
//! @brief ���{�b�g�̐ÓI����]�T(�x���r���p�`)��`�悷��N���X�D


#ifndef DESIGNLAB_STABILITY_MARGIN_RENDERER_H_
#define DESIGNLAB_STABILITY_MARGIN_RENDERER_H_


#include <memory>

#include "interface_hexapod_coordinate_converter.h"
#include "robot_state_node.h"


//! @class StabilityMarginRenderer
//! @brief ���{�b�g�̐ÓI����]�T(�x���r���p�`)��`�悷��N���X�D
//! @n �ڒn�_�𓊉e�������p�`�̓����ɁC�d�S�������Ă��邩�ǂ����ň��萫�𔻒肷��DStability Margin �ŃO�O��Əڂ����������o�Ă���D
class StabilityMarginRenderer final
{
public:

	StabilityMarginRenderer(const std::shared_ptr<const IHexapodCoordinateConverter>& converter_ptr);
	~StabilityMarginRenderer() = default;

	//! @brief ���{�b�g�̐ÓI����]�T(�x���r���p�`)��`�悷��D
	//! @param [in] node ���{�b�g�̏�ԁD
	void Draw(const RobotStateNode& node) const;


private:
	const unsigned int kMarginColor;		//!< �x���r���p�`�̐F�D

	const unsigned int kMarginErrorColor;	//!< ����łȂ��Ƃ��̐F

	const int kAlpha;						//!< �����x�D


	const std::shared_ptr<const IHexapodCoordinateConverter> converter_ptr_;
};


#endif // !DESIGNLAB_STABILITY_MARGIN_RENDERER_H_