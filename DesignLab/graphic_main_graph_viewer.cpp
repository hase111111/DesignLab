#include "graphic_main_graph_viewer.h"

#include <memory>

#include "designlab_dxlib.h"
#include "map_renderer.h"
#include "graph_tree_creator_hato.h"
#include "Keyboard.h"


GraphicMainGraphViewer::GraphicMainGraphViewer(const GraphicDataBroker* const  broker, const SApplicationSettingRecorder* const setting)
	: AbstractGraphicMain(broker, setting), m_map_state(broker->getMapState())
{
	//�K���ȃm�[�h�𐶐����āC�`��N���X������������
	SNode init_node;
	init_node.init(false);

	m_hexapod_renderer.update(init_node);
	m_camera_manager.setTargetPos(dl_dxlib::convertToDxVec(init_node.global_center_of_mass));

	// GUI �ɃO���t�̃|�C���^��n��.
	mp_gui_controller = std::make_unique<GraphViewerGUIController>(&m_graph, &m_display_node_index, mp_setting);
}


bool GraphicMainGraphViewer::update()
{
	updateCameraState();

	mp_gui_controller->update();

	//����l�̎��O���t�f�[�^�Ǝ��g�̎����Ă���O���t�f�[�^����v���Ă��Ȃ��Ȃ��
	if (mp_broker->getNodeNum() != m_graph.size())
	{
		mp_broker->copyAllNode(&m_graph);	//�f�[�^���X�V����

		//�O���t�̒��g����łȂ��Ȃ�΁C�\������m�[�h������������
		if (m_graph.size() > 0) { m_display_node_index = 0; }

		mp_gui_controller->updateGraphNodeDepthData();

	}

	//HexapodReander�̍X�V
	if (m_display_node_index < m_graph.size() && m_graph.size() > 0)
	{
		m_hexapod_renderer.update(m_graph.at(m_display_node_index));
	}

	return true;
}


void GraphicMainGraphViewer::draw() const
{
	MapRenderer map_renderer;
	map_renderer.draw(m_map_state);

	if (m_display_node_index < m_graph.size())
	{
		m_hexapod_renderer.draw(m_graph.at(m_display_node_index));
	}


	mp_gui_controller->draw();
}


void GraphicMainGraphViewer::updateCameraState()
{
	m_camera_manager.update();

	if (Keyboard::getIns()->getPressingCount(KEY_INPUT_Z) == 1)
	{
		m_camera_mode++;
		m_camera_mode %= 5;
		m_camera_manager.setCameraViewMode(static_cast<ECameraMode>(m_camera_mode));
	}

	if (m_display_node_index < m_graph.size())
	{
		m_camera_manager.setTargetPos(dl_dxlib::convertToDxVec(m_graph.at(m_display_node_index).global_center_of_mass));
	}
}
