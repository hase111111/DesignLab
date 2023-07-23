#pragma once
#include "MapState.h"
#include "GraphicSystem.h"
#include "GraphicDataBroker.h"
#include "InterfaceGraphTreeCreator.h"

class GraphViewerSystemMain final
{
public:

	GraphViewerSystemMain();

	void main();

private:
	MapState m_MapState;
	GraphicDataBroker m_GraphicDataBroker;
	GraphicSystem m_GraphicSystem;
	std::unique_ptr<IGraphTreeCreator> mp_GraphTreeCreator;

	// �O���t���쐬����
	void createGraph(const SNode _parent, std::vector<SNode>& _graph);

	//�O���t�𒇉�l�ɃZ�b�g����
	void setGraphToBroker(const std::vector<SNode>& _graph);

	// y / n �̎��������
	bool askYesNo(const std::string& question) const;

	// �O���t�̃X�e�[�^�X��\������
	void showGraphStatus(const std::vector<SNode>& _graph) const;
};
