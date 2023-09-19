//! @file graphic_main_test.h
//! @brief GraphicMainTest�N���X

#ifndef DESIGNLAB_GRAPHIC_MAIN_TEST_H_
#define DESIGNLAB_GRAPHIC_MAIN_TEST_H_

#include "interface_graphic_main.h"

#include <memory>

#include "abstract_hexapod_state_calculator.h"
#include "application_setting_recorder.h"
#include "camera_gui.h"
#include "hexapod_renderer.h"
#include "map_state.h"
#include "node.h"
#include "node_display_gui.h"


//! @class GraphicMainTest
//! @brief MapState��HexapodStateClaculator�����삵�Ă��邩�e�X�g���s�����߂̃N���X�D

class GraphicMainTest final : public IGraphicMain
{
public:
	GraphicMainTest(const std::shared_ptr<const AbstractHexapodStateCalculator>& calculator_ptr,
		const std::shared_ptr<const SApplicationSettingRecorder>& setting_ptr);
	~GraphicMainTest() = default;

	bool Update() override;

	void Draw() const override;

private:

	const std::shared_ptr<const AbstractHexapodStateCalculator> calculator_ptr_;	//!< ���{�b�g�̏�Ԃ��v�Z����N���X�̃V�F�A�[�h�|�C���^�D


	CameraGui camera_gui_;				//!< �J�����̈ʒu�𐧌䂷��GUI

	NodeDisplayGui node_display_gui_;	//!< �m�[�h�̕\���𐧌䂷��GUI


	HexapodRenderer hexapod_renderer_;	//!< ���{�b�g��\������N���X�D

	MapState map_state_;				//!< �}�b�v�̏�Ԃ�ێ�����N���X�D
	DevideMapState devide_map_state_;	//!< �}�b�v�𕪊�����N���X�D

	SNode m_node;						//!< ���{�b�g�̏��

	int m_map_index = 0;				//!< �}�b�v�̃C���f�b�N�X
};


#endif // !DESIGNLAB_GRAPHIC_MAIN_TEST_H_