#include "graphic_main_graph_viewer.h"

#include <memory>

#include "designlab_dxlib.h"
#include "map_renderer.h"
#include "graph_tree_creator_hato.h"
#include "Keyboard.h"


GraphicMainGraphViewer::GraphicMainGraphViewer(const GraphicDataBroker* const  broker, std::shared_ptr<AbstractHexapodStateCalculator> calc, const SApplicationSettingRecorder* const setting) :
	AbstractGraphicMain(broker, calc, setting),
	m_map_state(broker->map_state()),
	m_hexapod_renderer(calc),
	m_camera_gui(10, (*setting).window_size_y - CameraGUI::GUI_SIZE_Y - 10),
	m_node_display_gui((*setting).window_size_x - NodeDisplayGui::kWidth - 10, 10, calc)
{
	//�K���ȃm�[�h�𐶐����āC�`��N���X������������
	SNode init_node;
	init_node.init(false);

	m_hexapod_renderer.setNode(init_node);

	// GUI �ɃO���t�̃|�C���^��n��.
	mp_gui_controller = std::make_unique<GraphViewerGUIController>(&m_graph, &m_display_node_index, mp_setting);
}


bool GraphicMainGraphViewer::Update()
{
	mp_gui_controller->Update();

	//����l�̎��O���t�f�[�^�Ǝ��g�̎����Ă���O���t�f�[�^����v���Ă��Ȃ��Ȃ��
	if (mp_broker->GetNodeNum() != m_graph.size())
	{
		mp_broker->CopyAllNode(&m_graph);	//�f�[�^���X�V����

		//�O���t�̒��g����łȂ��Ȃ�΁C�\������m�[�h������������
		if (m_graph.size() > 0) { m_display_node_index = 0; }

		mp_gui_controller->updateGraphNodeDepthData();

	}

	//HexapodReander�̍X�V
	if (m_display_node_index < m_graph.size() && m_graph.size() > 0)
	{
		m_hexapod_renderer.setNode(m_graph.at(m_display_node_index));

		m_camera_gui.setHexapodPos(m_graph.at(m_display_node_index).global_center_of_mass);

		m_node_display_gui.SetDisplayNode(m_graph.at(m_display_node_index));
	}

	m_camera_gui.Update();

	m_node_display_gui.Update();

	return true;
}


void GraphicMainGraphViewer::Draw() const
{
	dl_dxlib::setZBufferEnable();

	MapRenderer map_renderer;

	map_renderer.Draw(m_map_state);

	if (m_display_node_index < m_graph.size())
	{
		m_hexapod_renderer.Draw();
	}


	mp_gui_controller->Draw();

	m_camera_gui.Draw();

	m_node_display_gui.Draw();
}
