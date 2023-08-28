#pragma once

#include <vector>

#include "node.h"
#include "application_setting_recorder.h"


//! @class GraphViewerGUIController
//! @date 2023/08/14
//! @author ���J��
//! @brief �O���t�̃m�[�h�̃f�[�^��\������GUI�̃R���g���[���[�N���X
class GraphViewerGUIController final
{
public:
	GraphViewerGUIController(const std::vector<SNode>* const p_graph, size_t* const p_display_node_index, const SApplicationSettingRecorder* const setting);
	~GraphViewerGUIController() = default;

	void update();

	void draw() const;

	void updateGraphNodeDepthData();

private:

	void drawGraphData() const;
	void drawNodeControllPanel() const;
	void drawNodeData(const SNode& node) const;

	void inputNumber();
	void changeDisplayNodeIndex();
	void updateChildrenList();

	const std::vector<SNode>* const mp_graph;

	const SApplicationSettingRecorder* const mp_setting;


	size_t* const mp_display_node_index;
	std::pair<int, std::vector<int>> m_childen_list = std::make_pair<int, std::vector<int>>(-1, {});	//�q�m�[�h�̃��X�g
	int m_display_children_list_index = 0;	//�\������q�m�[�h�̃��X�g�̃C���f�b�N�X

	std::vector<int> m_graph_node_depth_data;	//�e�[�����Ƃ̃m�[�h���̃f�[�^
	int m_input_number = -1;	//���͂��ꂽ���l
};


//! @file graph_viewer_gui_controller.h
//! @date 2023/08/14
//! @author ���J��
//! @brief �O���t�̃m�[�h�̃f�[�^��\������GUI�̃R���g���[���[�N���X
//! @n �s�� ; @lineinfo
