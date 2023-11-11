//! @file graphic_main_test.h
//! @brief GraphicMainTest�N���X

#ifndef DESIGNLAB_GRAPHIC_MAIN_TEST_H_
#define DESIGNLAB_GRAPHIC_MAIN_TEST_H_

#include "interface_graphic_main.h"

#include <memory>

#include "application_setting_recorder.h"
#include "camera_gui.h"
#include "devide_map_state.h"
#include "interface_hexapod_renderer.h"
#include "interface_hexapod_coordinate_converter.h"
#include "interface_hexapod_joint_calculator.h"
#include "interface_hexapod_vaild_checker.h"
#include "map_state.h"
#include "map_renderer.h"
#include "robot_state_node.h"
#include "node_display_gui.h"


//! @class GraphicMainTest
//! @brief MapState��HexapodStateClaculator�����삵�Ă��邩�e�X�g���s�����߂̃N���X�D
class GraphicMainTest final : public IGraphicMain
{
public:
	GraphicMainTest(
		const std::shared_ptr<const IHexapodCoordinateConverter>& converter_ptr,
		const std::shared_ptr<const IHexapodJointCalculator>& calculator_ptr,
		const std::shared_ptr<const IHexapodVaildChecker>& checker_ptr,
		const std::shared_ptr<const ApplicationSettingRecorder>& setting_ptr
	);
	~GraphicMainTest() = default;

	bool Update() override;

	void Draw() const override;

private:

	void MoveBody();

	void MoveLeg();


	CameraGui camera_gui_;				//!< �J�����̈ʒu�𐧌䂷��GUI

	NodeDisplayGui node_display_gui_;	//!< �m�[�h�̕\���𐧌䂷��GUI

	MapRenderer map_render_;

	const std::shared_ptr<const IHexapodCoordinateConverter>& converter_ptr_;
	const std::shared_ptr<const IHexapodJointCalculator>& calculator_ptr_;
	const std::unique_ptr<IHexapodRenderer> hexapod_renderer_;	

	MapState map_state_;				//!< �}�b�v�̏�Ԃ�ێ�����N���X�D

	DevideMapState devide_map_state_;	//!< �}�b�v�𕪊�����N���X�D

	RobotStateNode node_;						//!< ���{�b�g�̏��

	int map_index_ = 0;				//!< �}�b�v�̃C���f�b�N�X
};


#endif // DESIGNLAB_GRAPHIC_MAIN_TEST_H_