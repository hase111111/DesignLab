//! @file graph_viewer_gui_controller.h
//! @brief �O���t�̃m�[�h�̃f�[�^��\������GUI�̃R���g���[���[�N���X

#ifndef DESIGNLAB_GRAPH_VIEWER_GUI_CONTROLLER_H_
#define DESIGNLAB_GRAPH_VIEWER_GUI_CONTROLLER_H_

#include <memory>
#include <vector>

#include "application_setting_recorder.h"
#include "robot_state_node.h"

//! @todo ���u���̂͂����C�{�����ɂȂ��Ă���̂ŁC��Œ���

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

	void UpdateGraphNodeDepthData();

private:

	void DrawGraphData() const;
	void DrawNodeControllPanel() const;
	void DrawNodeData(const RobotStateNode& node) const;

	void InputNumber();
	void ChangeDisplayNodeIndex();
	void UpdateChildrenList();

	const std::vector<RobotStateNode>* const graph_ptr_;

	const std::shared_ptr<const ApplicationSettingRecorder> setting_ptr_;


	size_t* const display_node_index_ptr_;
	std::pair<int, std::vector<int>> childen_list_ = std::make_pair<int, std::vector<int>>(-1, {});	//�q�m�[�h�̃��X�g
	int display_children_list_index_ = 0;	//�\������q�m�[�h�̃��X�g�̃C���f�b�N�X

	std::vector<int> graph_node_depth_data_;	//�e�[�����Ƃ̃m�[�h���̃f�[�^
	int input_number_ = -1;	//���͂��ꂽ���l
};


#endif // !DESIGNLAB_GRAPH_VIEWER_GUI_CONTROLLER_H_