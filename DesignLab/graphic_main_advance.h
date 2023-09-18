#pragma once

#include <vector>

#include "interface_graphic_main.h"
#include "map_state.h"
#include "node.h"
#include "graphic_const.h"
#include "camera_gui.h"
#include "display_node_switch_gui.h"
#include "node_display_gui.h"
#include "hexapod_renderer.h"
#include "movement_locus_renderer.h"
#include "robot_graund_point_renderer.h"
#include "stability_margin_renderer.h"
#include "interpolated_node_creator.h"




class GraphicMainAdvance final : public IGraphicMain
{
public:
	GraphicMainAdvance(const GraphicDataBroker* const  broker, std::shared_ptr<AbstractHexapodStateCalculator> calc, const SApplicationSettingRecorder* const setting);
	~GraphicMainAdvance() = default;

	bool Update() override;

	void Draw() const override;

private:

	const int kNodeGetCount;			//2�b���Ƃɓǂݏo���D

	const int kInterpolatedAnimeCount = 15;


	CameraGui m_camera_gui;							// �J�����̈ʒu�𐧌䂷��GUI

	NodeDisplayGui m_node_display_gui;				// �m�[�h�̕\���𐧌䂷��GUI

	DisplayNodeSwitchGUI m_display_node_switch_gui;	// �m�[�h�̕\����؂�ւ���GUI


	HexapodRenderer m_hexapod_renderer;						//!< ���{�b�g��\������N���X�D	

	MovementLocusRenderer m_movement_locus_renderer;		//!< ���{�b�g�̓����̋O�Ղ�\������N���X�D

	RobotGraundPointRenderer m_robot_graund_point_renderer;	//!< ���{�b�g�̑���̈ʒu��\������N���X�D

	StabilityMarginRenderer m_stability_margin_renderer;	//!< ���{�b�g�̈��萫�}�[�W����\������N���X�D


	InterpolatedNodeCreator m_interpolated_node_creator;	//!< �m�[�h�Ԃ��Ԃ���N���X�D


	std::vector<SNode> m_node;			//���{�b�g�̓����̑J�ڂ��L�^����vector

	int m_display_node_index = -1;		//�`�悵�Ă���m�[�h

	MapState m_map_state;				//�\������}�b�v�D

	int m_counter = 0;					//���̃N���X�����s����Ă��牽��update�֐����Ă΂ꂽ���J�E���g����D

	std::vector<SNode> m_interpolated_node;	//��Ԃ��ꂽ�m�[�h���L�^����vector

	int m_interpolated_anime_start_count = -100;	//��Ԃ��ꂽ�m�[�h��\�����n�߂�J�E���g


	bool m_is_display_movement_locus = true;		//���{�b�g�̓����̋O�Ղ�\�����邩�ǂ����D

	bool m_is_display_robot_graund_point = true;	//���{�b�g�̑���̈ʒu��\�����邩�ǂ����D
};

