#include "GraphicMainTest.h"
#include "Dxlib3DFunction.h"
#include "NodeEdit.h"
#include "MapRenderer.h"
#include "Keyboard.h"

GraphicMainTest::GraphicMainTest(const GraphicDataBroker* _broker) : IGraphicMain(_broker)
{
	//3D�n�̏����s���O�ɏ���������D
	myDxlib3DFunc::initDxlib3D();

	node_edit::initNode(m_node, false);

	m_Camera.setTargetPos(myDxlib3DFunc::convertToDxVec(m_node.global_center_of_mass));

	m_Map.init(EMapCreateMode::Flat, MapCreator::OPTION_NONE, false);
}

bool GraphicMainTest::update()
{
	const float _s = 1;
	if (Keyboard::getIns()->getPressingCount(KEY_INPUT_1) > 0) 
	{
		if (Keyboard::getIns()->getPressingCount(KEY_INPUT_Q) > 0) { m_node.Leg[0].z += _s; }
		else if (Keyboard::getIns()->getPressingCount(KEY_INPUT_E) > 0) { m_node.Leg[0].z -= _s; }
		else if (Keyboard::getIns()->getPressingCount(KEY_INPUT_A) > 0) { m_node.Leg[0].y += _s; }
		else if (Keyboard::getIns()->getPressingCount(KEY_INPUT_D) > 0) { m_node.Leg[0].y -= _s; }
		else if (Keyboard::getIns()->getPressingCount(KEY_INPUT_W) > 0) { m_node.Leg[0].x += _s; }
		else if (Keyboard::getIns()->getPressingCount(KEY_INPUT_S) > 0) { m_node.Leg[0].x -= _s; }
	}
	else 
	{
		if (Keyboard::getIns()->getPressingCount(KEY_INPUT_Q) > 0)
		{
			m_node.global_center_of_mass.z += _s;
			for (int i = 0; i < HexapodConst::LEG_NUM; i++) { m_node.Leg[i].z -= _s; }
		}
		else if (Keyboard::getIns()->getPressingCount(KEY_INPUT_E) > 0)
		{
			m_node.global_center_of_mass.z -= _s;
			for (int i = 0; i < HexapodConst::LEG_NUM; i++) { m_node.Leg[i].z += _s; }
		}
	}

	m_HexapodRender.update(m_node);
//	m_GUI.update(m_Camera); //GUI���X�V����D
	m_Camera.update();

	return false;
}

void GraphicMainTest::draw() const
{
	MapRenderer _map_render;
	_map_render.setNode(m_node);
	_map_render.draw(m_Map);

	m_HexapodRender.draw(m_node);

	m_GUI.draw(m_node);
}