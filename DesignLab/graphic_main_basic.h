#pragma once

#include <vector>

#include "interface_graphic_main.h"
#include "map_state.h"
#include "node.h"
#include "graphic_const.h"
#include "CameraController.h"
#include "gui_controller.h"
#include "hexapod_renderer.h"


//! @class GraphicMainBasic
//! @date 2023/08/09
//! @author ���J��
//! @brief ���̃v���W�F�N�g�ɂ�����W���I�ȃ��{�b�g�̕`��@�\�����N���X�D
//! @details �g������̃v���O�����̃��{�b�g�\���@�\���������������́D
//! ��{�I�ȏ����̓��e�͕ω����Ă��Ȃ����C���\������f�[�^�̓��e���ڂ����Ȃ��Ă���D
//! �܂��CUI�ɂ���ă����^�C���ŕ\�����@�𐧌䂷�邱�Ƃ��ł���悤�ɂȂ������߁C��胍�{�b�g�̏�Ԃ𗝉����₷���Ȃ��Ă���D
//! @note ������傫���������������ꍇ�͂��������V�����N���X�������悤�ɂ���Ƃ悢�Ǝv���D
//! @n GraphicSample���Q�l�ɂ��āC�쐬����悤�ɂ���Ɗy�D
class GraphicMainBasic final : public IGraphicMain
{
public:
	GraphicMainBasic(const GraphicDataBroker* broker);
	~GraphicMainBasic() = default;

	bool update() override;

	void draw() const override;

private:

	CameraController m_camera_controller;	//�J��������N���X�D

	HexapodRenderer m_hexapod_renderer;

	GUIController m_gui_controller;		// GUI (���{�b�g�̏�ԂƂ��\�����鑋) �𐧌䂷��N���X�D

	std::vector<SNode> m_node;	//���{�b�g�̓����̑J�ڂ��L�^����vector

	int m_display_node = 0;		//�`�悵�Ă���m�[�h

	MapState m_map_state;				//�\������}�b�v�D

	int m_counter = 0;			//���̃N���X�����s����Ă��牽��update�֐����Ă΂ꂽ���J�E���g����D

	const int kNodeGetCount = 2 * GraphicConst::GRAPHIC_FPS;	//2�b���Ƃɓǂݏo���D
};


//! @file graphic_main_basic.h
//! @date 2023/08/09
//! @author ���J��
//! @brief ��{�I�ȕ`��N���X�D
//! @n �s�� : @lineinfo
