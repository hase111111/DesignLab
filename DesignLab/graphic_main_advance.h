#ifndef DESIGNLAB_GRAPHIC_MAIN_ADVANCE_H_
#define DESIGNLAB_GRAPHIC_MAIN_ADVANCE_H_


#include "interface_graphic_main.h"

#include <memory>
#include <vector>

#include "application_setting_recorder.h"
#include "camera_gui.h"
#include "display_node_switch_gui.h"
#include "graphic_const.h"
#include "graphic_data_broker.h"
#include "interface_hexapod_renderer.h"
#include "interface_hexapod_coordinate_converter.h"
#include "interface_hexapod_joint_calculator.h"
#include "interface_hexapod_vaild_checker.h"
#include "interpolated_node_creator.h"
#include "map_state.h"
#include "map_renderer.h"
#include "movement_locus_renderer.h"
#include "robot_state_node.h"
#include "node_display_gui.h"
#include "robot_graund_point_renderer.h"
#include "stability_margin_renderer.h"


class GraphicMainAdvance final : public IGraphicMain
{
public:
	GraphicMainAdvance(
		const std::shared_ptr<const GraphicDataBroker>& broker_ptr,
		const std::shared_ptr<const IHexapodCoordinateConverter>& converter_ptr,
		const std::shared_ptr<const IHexapodJointCalculator>& calculator_ptr,
		const std::shared_ptr<const IHexapodVaildChecker>& checker_ptr,
		const std::shared_ptr<const ApplicationSettingRecorder>& setting_ptr
	);
	~GraphicMainAdvance() = default;

	bool Update() override;

	void Draw() const override;

private:

	const int kNodeGetCount;			//!< �m�[�h���擾����Ԋu�D

	const int kInterpolatedAnimeCount;	//!< ��Ԃ��ꂽ�m�[�h�̕\����؂�ւ���Ԋu�D


	const std::shared_ptr<const GraphicDataBroker> broker_ptr_;		//!< �f�[�^���󂯎��N���X�D


	CameraGui camera_gui_;							//!< �J�����̈ʒu�𐧌䂷��GUI

	DisplayNodeSwitchGui display_node_switch_gui_;	//!< �m�[�h�̕\����؂�ւ���GUI

	NodeDisplayGui node_display_gui_;				//!< �m�[�h�̕\���𐧌䂷��GUI


	const std::unique_ptr<IHexapodRenderer> hexapod_renderer_;	//!< ���{�b�g��\������N���X�D	

	MovementLocusRenderer movement_locus_renderer_;			//!< ���{�b�g�̓����̋O�Ղ�\������N���X�D

	RobotGraundPointRenderer robot_graund_point_renderer_;	//!< ���{�b�g�̑���̈ʒu��\������N���X�D

	StabilityMarginRenderer stability_margin_renderer_;		//!< ���{�b�g�̈��萫�}�[�W����\������N���X�D

	InterpolatedNodeCreator interpolated_node_creator_;		//!< �m�[�h�Ԃ��Ԃ���N���X�D

	MapRenderer map_renderer_;								//!< �}�b�v��\������N���X�D


	MapState map_state_;		//!< �\������}�b�v�D

	std::vector<RobotStateNode> graph_;	//!< ���{�b�g�̓����̑J�ڂ��L�^����vector

	size_t display_node_index_;	//!< �`�悵�Ă���m�[�h

	int counter_;				//!< ���̃N���X�����s����Ă��牽��update�֐����Ă΂ꂽ���J�E���g����D

	std::vector<RobotStateNode> interpolated_node_;	//!< ��Ԃ��ꂽ�m�[�h���L�^����vector

	int interpolated_anime_start_count_;	//!< ��Ԃ��ꂽ�m�[�h��\�����n�߂�J�E���g


	bool is_displayed_movement_locus_;		//!< ���{�b�g�̓����̋O�Ղ�\�����邩�ǂ����D

	bool is_displayed_robot_graund_point_;	//!< ���{�b�g�̑���̈ʒu��\�����邩�ǂ����D


	int graph_update_count;	//!< �������ł̃O���t�̍X�V�񐔁D

	int map_update_count;	//!< �������ł̃}�b�v�̍X�V�񐔁D
};


#endif // !DESIGNLAB_GRAPHIC_MAIN_ADVANCE_H_