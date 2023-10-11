//! @file graph_viewer_gui_controller.h
//! @brief �O���t�̃m�[�h�̃f�[�^��\������GUI�̃R���g���[���[�N���X

#ifndef DESIGNLAB_GRAPH_VIEWER_GUI_CONTROLLER_H_
#define DESIGNLAB_GRAPH_VIEWER_GUI_CONTROLLER_H_

#include <memory>
#include <vector>

#include "application_setting_recorder.h"
#include "robot_state_node.h"


//! @class GraphViewerGUIController
//! @brief �O���t�̃m�[�h�̃f�[�^��\������GUI�̃R���g���[���[�N���X
class GraphViewerGUIController final
{
public:
	GraphViewerGUIController(const std::vector<RobotStateNode>* const graph_ptr, size_t* const display_node_index_ptr,
		const std::shared_ptr<const ApplicationSettingRecorder>& setting_ptr);
	~GraphViewerGUIController() = default;

	void Update();

	void Draw() const;

	void updateGraphNodeDepthData();

private:

	void drawGraphData() const;
	void drawNodeControllPanel() const;
	void drawNodeData(const RobotStateNode& node) const;

	void inputNumber();
	void changeDisplayNodeIndex();
	void updateChildrenList();

	const std::vector<RobotStateNode>* const mp_graph;

	const std::shared_ptr<const ApplicationSettingRecorder> setting_ptr_;


	size_t* const mp_display_node_index;
	std::pair<int, std::vector<int>> m_childen_list = std::make_pair<int, std::vector<int>>(-1, {});	//�q�m�[�h�̃��X�g
	int m_display_children_list_index = 0;	//�\������q�m�[�h�̃��X�g�̃C���f�b�N�X

	std::vector<int> m_graph_node_depth_data;	//�e�[�����Ƃ̃m�[�h���̃f�[�^
	int m_input_number = -1;	//���͂��ꂽ���l
};


#endif // !DESIGNLAB_GRAPH_VIEWER_GUI_CONTROLLER_H_