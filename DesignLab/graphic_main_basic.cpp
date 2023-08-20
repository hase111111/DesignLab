#include "graphic_main_basic.h"

#include "DxLib.h"

#include "designlab_dxlib.h"
#include "map_renderer.h"


GraphicMainBasic::GraphicMainBasic(const GraphicDataBroker* broker) : IGraphicMain(broker), m_map_state(mp_broker->getMapState())
{
	m_node.clear();
}


bool GraphicMainBasic::update()
{
	if (m_counter % kNodeGetCount == 0)
	{
		//�m�[�h��ǂݏo�����ԂɂȂ�����C�ǂݏo���D
		mp_broker->copyOnlyNewNode(&m_node);
	}

	m_gui_controller.update((int)m_node.size(), m_display_node, m_counter); //GUI���X�V����D

	if (!m_node.empty())
	{
		m_camera_gui.setHexapodPos(m_node.at(m_display_node).global_center_of_mass);

		m_hexapod_renderer.update(m_node.at(m_display_node));      //���{�b�g�̏�Ԃ��X�V����D
	}

	m_counter++;            //�J�E���^��i�߂�D

	m_camera_gui.update();      //�J������GUI���X�V����D

	return true;
}


void GraphicMainBasic::draw() const
{
	if (!m_node.empty())
	{
		dl_dxlib::setZBufferEnable();

		//�}�b�v��`�悷��D
		MapRenderer map_render;
		map_render.setNode(m_node.at(m_display_node));
		map_render.draw(m_map_state);

		//�m�[�h�����݂��Ă���Ȃ�΁C���{�b�g��`�悷��D
		m_hexapod_renderer.draw(m_node.at(m_display_node));

		//UI��\������D
		m_gui_controller.draw(m_node.at(m_display_node));
	}

	m_camera_gui.draw();        //�J������GUI��`�悷��D
}
