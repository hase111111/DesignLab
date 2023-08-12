#pragma once

#include "map_state.h"
#include "graphic_system.h"
#include "graphic_data_broker.h"
#include "InterfacePassFinder.h"


//! @class GraphViewerSystemMain
//! @date 2023/08/13
//! @author ���J��
//! @brief �O���t��\������V�X�e���̃��C���N���X
class GraphViewerSystemMain final
{
public:

	GraphViewerSystemMain();

	//! @brief ���C���֐�
	void main();

private:

	// �O���t���쐬����
	void createGraph(const SNode parent, std::vector<SNode>& graph);

	//�O���t�𒇉�l�ɃZ�b�g����
	void setGraphToBroker(const std::vector<SNode>& graph);

	// y / n �̎��������
	bool askYesNo(const std::string& question) const;

	// �O���t�̃X�e�[�^�X��\������
	void showGraphStatus(const std::vector<SNode>& graph) const;


	MapState m_map_state;

	GraphicDataBroker m_graphic_data_broker;

	GraphicSystem m_graphic_system;

	std::unique_ptr<IPassFinder> mp_pass_finder;

};


//! @file graph_viewer_system_main.h
//! @date 2023/08/13
//! @author ���J��
//! @brief �O���t��\������V�X�e���̃��C���N���X
//! @n �s�� : @lineinfo
