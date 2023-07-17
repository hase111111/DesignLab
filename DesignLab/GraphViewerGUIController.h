#pragma once
#include <vector>
#include "Node.h"

class GraphViewerGUIController final
{
public:
	GraphViewerGUIController(const std::vector<SNode>* const _p_graph, size_t* const _p_display_node_index);
	~GraphViewerGUIController() = default;

	void update();

	void draw() const;

	void updateGraphNodeDepthData();

private:
	const std::vector<SNode>* const mp_graph;
	size_t* const mp_display_node_index;
	std::pair<int, std::vector<int>> m_childen_list = std::make_pair<int, std::vector<int>>(-1, {});	//�q�m�[�h�̃��X�g
	int m_display_children_list_index = 0;	//�\������q�m�[�h�̃��X�g�̃C���f�b�N�X

	std::vector<int> m_graph_node_depth_data;	//�e�[�����Ƃ̃m�[�h���̃f�[�^
	int m_input_number = -1;	//���͂��ꂽ���l

	void drawGraphData() const;
	void drawNodeControllPanel() const;

	void inputNumber();
	void changeDisplayNodeIndex();
	void updateChildrenList();
};
