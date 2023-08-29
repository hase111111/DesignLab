#include "graphic_main_basic.h"

#include "DxLib.h"

#include "designlab_dxlib.h"
#include "world_grid_renderer.h"
#include "map_renderer.h"


GraphicMainBasic::GraphicMainBasic(const GraphicDataBroker* const  broker, const SApplicationSettingRecorder* const setting)
	: AbstractGraphicMain(broker, setting), m_map_state(mp_broker->getMapState()), kNodeGetCount(setting->window_fps * 2), m_gui_controller(mp_setting),
	m_node_display_gui(mp_setting->window_size_x - NodeDisplayGUI::BOX_SIZE_X - 10, 10)
{
	m_node.clear();
}


bool GraphicMainBasic::update()
{

	//�m�[�h��ǂݏo�����ԂɂȂ�����C����l����f�[�^��ǂݏo���D
	if (m_counter % kNodeGetCount == 0)
	{
		mp_broker->copyOnlyNewNode(&m_node);

		m_movement_locus_renderer.setMovementLocus(m_node);   //�ړ��O�Ղ��X�V����D

		m_robot_graund_point_renderer.setNode(m_node);        //���{�b�g�̐ڒn�_���X�V����D


		std::vector<size_t> simu_end_index;

		mp_broker->copySimuEndIndex(&simu_end_index);

		m_movement_locus_renderer.setSimuEndIndex(simu_end_index);
	}


	m_gui_controller.update((int)m_node.size(), m_display_node, m_counter); //GUI���X�V����D


	//�m�[�h�����݂��Ă���̂Ȃ�΁C�e�N���X�ɏ���`�B����
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
	// 3D�̃I�u�W�F�N�g�̕`��

	dl_dxlib::setZBufferEnable();		//Z�o�b�t�@��L���ɂ���D


	WorldGridRenderer grid_renderer;	//�C���X�^���X�𐶐�����D

	grid_renderer.draw();				//�O���b�h��`�悷��D


	MapRenderer map_render;				//�}�b�v��`�悷��D

	map_render.draw(m_map_state);


	m_movement_locus_renderer.draw();   //�ړ��O�Ղ�`�悷��D

	m_robot_graund_point_renderer.draw(-1, true);

	if (!m_node.empty())
	{
		//�m�[�h�����݂��Ă���Ȃ�΁C���{�b�g��`�悷��D
		m_hexapod_renderer.draw(m_node.at(m_display_node));
	}


	// 2D��GUI�̕`��

	m_camera_gui.draw();        //�J������GUI��`�悷��D

	m_node_display_gui.draw();	 //�m�[�h�̏���\������GUI��`�悷��D
}
