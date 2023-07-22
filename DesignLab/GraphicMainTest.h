#pragma once
#include "InterfaceGraphicMain.h"
#include "CameraController.h"
#include "HexapodRenderer.h"
#include "GUIController.h"
#include "Node.h"
#include "MapState.h"

//MapState��HexapodStateClaculator�����삵�Ă��邩���o�I�ɕ�����₷�����邽�߂̃e�X�g�V�[��
class GraphicMainTest final : public IGraphicMain
{
public:
	GraphicMainTest(const GraphicDataBroker* _broker);
	~GraphicMainTest() = default;

	bool update() override;

	void draw() const override;

private:
	SNode m_node;

	CameraController m_Camera;	//�J��������N���X�D
	HexapodRenderer m_HexapodRender;
	MapState m_Map;
	GUIController m_GUI;		// GUI (���{�b�g�̏�ԂƂ��\�����鑋) �𐧌䂷��N���X�D
};
