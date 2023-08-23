#include "graphic_main_basic.h"

#include "DxLib.h"

#include "designlab_dxlib.h"
#include "world_grid_renderer.h"
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

		m_movement_locus_renderer.setMovementLocus(m_node);   //�ړ��O�Ղ��X�V����D
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
	dl_dxlib::setZBufferEnable();   //Z�o�b�t�@��L���ɂ���D


	WorldGridRenderer grid_renderer;	//�C���X�^���X�𐶐�����D

	grid_renderer.draw();    //�O���b�h��`�悷��D


	m_movement_locus_renderer.draw();   //�ړ��O�Ղ�`�悷��D


	if (!m_node.empty())
	{
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
