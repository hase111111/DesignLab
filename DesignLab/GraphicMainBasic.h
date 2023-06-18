#pragma once
#include "InterfaceGraphicMain.h"
#include <vector>
#include "MapState.h"
#include "Node.h"
#include "GraphicConst.h"
#include "CameraController.h"
#include "GUIController.h"
#include "HexapodRenderer.h"


class GraphicMainBasic final : public IGraphicMain
{
public:
	GraphicMainBasic(const GraphicDataBroker* _broker);
	~GraphicMainBasic() = default;

	bool update() override;

	void draw() const override;

private:

	CameraController m_Camera;	//�J��������N���X�D

	HexapodRenderer m_HexapodRender;

	GUIController m_GUI;		// GUI (���{�b�g�̏�ԂƂ��\�����鑋) �𐧌䂷��N���X�D

	std::vector<SNode> m_node;	//���{�b�g�̓����̑J�ڂ��L�^����vector

	int m_display_node = 0;		//�`�悵�Ă���m�[�h

	MapState m_Map;				//�\������}�b�v�D

	int m_counter = 0;			//���̃N���X�����s����Ă��牽��update�֐����Ă΂ꂽ���J�E���g����D

	const int GET_NODE_COUNT = 2 * GraphicConst::GRAPHIC_FPS;	//2�b���Ƃɓǂݏo���D
};


//! @file GraphicMainBasic.h
//! @brief ��{�I�ȕ`��N���X�̎����D
//! @author ���J��

//! @class GraphicMainBasic
//! @brief ���̃v���W�F�N�g�ɂ�����W���I�ȃ��{�b�g�̕`��@�\�����N���X�D
//! @details �g������̃v���O�����̃��{�b�g�\���@�\���������������́D<br>
//! ��{�I�ȏ����̓��e�͕ω����Ă��Ȃ����C���\������f�[�^�̓��e���ڂ����Ȃ��Ă���D<br>
//! UI�ɂ���ă����^�C���ŕ\�����@�𐧌䂷�邱�Ƃ��ł���悤�ɂȂ������߁C��胍�{�b�g�̏�Ԃ𗝉����₷���Ȃ��Ă���D<br>
//! @note ������傫���������������ꍇ�͂��������V�����N���X�������悤�ɂ���Ƃ悢�Ǝv���D<br>
//! GraphicSample���Q�l�ɂ��āC�쐬����悤�ɂ���Ɗy�D
//! @author ���J��
