#pragma once

#include <vector>

#include "abstract_graphic_main.h"
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



//! @class GraphicMainBasic
//! @date 2023/08/09
//! @author ���J��
//! @brief ���̃v���W�F�N�g�ɂ�����W���I�ȃ��{�b�g�̕`��@�\�����N���X�D
//! @details �g������̃v���O�����̃��{�b�g�\���@�\���������������́D
//! ��{�I�ȏ����̓��e�͕ω����Ă��Ȃ����C���\������f�[�^�̓��e���ڂ����Ȃ��Ă���D
//! �܂��CUI�ɂ���ă����^�C���ŕ\�����@�𐧌䂷�邱�Ƃ��ł���悤�ɂȂ������߁C��胍�{�b�g�̏�Ԃ𗝉����₷���Ȃ��Ă���D�����
//! @note ������傫���������������ꍇ�͂��������V�����N���X�������悤�ɂ���Ƃ悢�Ǝv���D
//! @n GraphicSample���Q�l�ɂ��āC�쐬����悤�ɂ���Ɗy�D
class GraphicMainBasic final : public AbstractGraphicMain
{
public:
	GraphicMainBasic(const GraphicDataBroker* const  broker, std::shared_ptr<AbstractHexapodStateCalculator> calc, const SApplicationSettingRecorder* const setting);
	~GraphicMainBasic() = default;

	bool update() override;

	void draw() const override;

private:

	CameraGUI m_camera_gui;							// �J�����̈ʒu�𐧌䂷��GUI

	NodeDisplayGUI m_node_display_gui;				// �m�[�h�̕\���𐧌䂷��GUI

	DisplayNodeSwitchGUI m_display_node_switch_gui;	// �m�[�h�̕\����؂�ւ���GUI


	HexapodRenderer m_hexapod_renderer;						//!< ���{�b�g��\������N���X�D	

	MovementLocusRenderer m_movement_locus_renderer;		//!< ���{�b�g�̓����̋O�Ղ�\������N���X�D

	RobotGraundPointRenderer m_robot_graund_point_renderer;	//!< ���{�b�g�̑���̈ʒu��\������N���X�D

	StabilityMarginRenderer m_stability_margin_renderer;	//!< ���{�b�g�̈��萫�}�[�W����\������N���X�D


	std::vector<SNode> m_node;			//���{�b�g�̓����̑J�ڂ��L�^����vector

	int m_display_node = 0;				//�`�悵�Ă���m�[�h

	MapState m_map_state;				//�\������}�b�v�D

	int m_counter = 0;					//���̃N���X�����s����Ă��牽��update�֐����Ă΂ꂽ���J�E���g����D

	const int kNodeGetCount;			//2�b���Ƃɓǂݏo���D
};


//! @file graphic_main_basic.h
//! @date 2023/08/09
//! @author ���J��
//! @brief ��{�I�ȕ`��N���X�D
//! @n �s�� : @lineinfo
