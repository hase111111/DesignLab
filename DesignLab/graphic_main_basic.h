//! @file graphic_main_basic.h
//! @brief ��{�I�ȕ`��N���X�D

#ifndef DESIGNLAB_GRAPHIC_MAIN_BASIC_H_
#define DESIGNLAB_GRAPHIC_MAIN_BASIC_H_

#include "interface_graphic_main.h"

#include <memory>
#include <vector>

#include "abstract_hexapod_state_calculator.h"
#include "application_setting_recorder.h"
#include "camera_gui.h"
#include "display_node_switch_gui.h"
#include "graphic_const.h"
#include "graphic_data_broker.h"
#include "hexapod_renderer.h"
#include "map_state.h"
#include "movement_locus_renderer.h"
#include "node.h"
#include "node_display_gui.h"
#include "robot_graund_point_renderer.h"
#include "stability_margin_renderer.h"


//! @class GraphicMainBasic
//! @brief ���̃v���W�F�N�g�ɂ�����W���I�ȃ��{�b�g�̕`��@�\�����N���X�D
//! @details �g������̃v���O�����̃��{�b�g�\���@�\���������������́D
//! ��{�I�ȏ����̓��e�͕ω����Ă��Ȃ����C���\������f�[�^�̓��e���ڂ����Ȃ��Ă���D
//! �܂��CUI�ɂ���ă����^�C���ŕ\�����@�𐧌䂷�邱�Ƃ��ł���悤�ɂȂ������߁C��胍�{�b�g�̏�Ԃ𗝉����₷���Ȃ��Ă���D�����
//! @note ������傫���������������ꍇ�͂��������V�����N���X�������悤�ɂ���Ƃ悢�Ǝv���D
//! @n GraphicSample���Q�l�ɂ��āC�쐬����悤�ɂ���Ɗy�D

class GraphicMainBasic final : public IGraphicMain
{
public:
	GraphicMainBasic(const std::shared_ptr<const GraphicDataBroker>& broker_ptr, const std::shared_ptr<const AbstractHexapodStateCalculator>& calculator_ptr,
		const std::shared_ptr<const ApplicationSettingRecorder>& setting_ptr);
	~GraphicMainBasic() = default;

	bool Update() override;

	void Draw() const override;

private:

	const int kNodeGetCount;	//!< ���̃J�E���g���ƂɃf�[�^��Broker����ǂݏo��


	const std::shared_ptr<const GraphicDataBroker> broker_ptr_;	//!< �摜�\�����s�����̃N���X�ƁC�f�[�^�������s���O���̃N���X���q������l�N���X�̃|�C���^���󂯎��D


	CameraGui camera_gui_;							//!< �J�����̈ʒu�𐧌䂷��GUI

	NodeDisplayGui node_display_gui_;				//!< �m�[�h�̕\���𐧌䂷��GUI

	DisplayNodeSwitchGui display_node_switch_gui_;	//!< �m�[�h�̕\����؂�ւ���GUI


	HexapodRenderer hexapod_renderer_;						//!< ���{�b�g��\������N���X�D	

	MovementLocusRenderer movement_locus_renderer_;			//!< ���{�b�g�̓����̋O�Ղ�\������N���X�D

	RobotGraundPointRenderer robot_graund_point_renderer_;	//!< ���{�b�g�̑���̈ʒu��\������N���X�D

	StabilityMarginRenderer stability_margin_renderer_;		//!< ���{�b�g�̐ÓI����]�T��\������N���X�D


	MapState map_state_;		//!< �\������}�b�v�D

	std::vector<SNode> graph_;	//!< ���{�b�g�̓����̑J�ڂ��L�^����vector

	size_t display_node_index_;	//!< �`�悵�Ă���m�[�h

	int counter_;				//!< ���̃N���X�����s����Ă��牽��update�֐����Ă΂ꂽ���J�E���g����D


	bool is_displayed_movement_locus_;		//!< ���{�b�g�̓����̋O�Ղ�\�����邩�ǂ����D

	bool is_displayed_robot_graund_point_;	//!< ���{�b�g�̑���̈ʒu��\�����邩�ǂ����D


	int graph_update_count;	//!< �������ł̃O���t�̍X�V�񐔁D

	int map_update_count;	//!< �������ł̃}�b�v�̍X�V�񐔁D
};


#endif // !DESIGNLAB_GRAPHIC_MAIN_BASIC_H_