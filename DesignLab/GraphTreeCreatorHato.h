#pragma once
#include "InterfaceGraphTreeCreator.h"
#include "MapState.h"
#include "ComMoveNodeCreator.h"
#include "ComUpDownNodeCreator.h"
#include "LegHierarchyNodeCreator.h"
#include "LegUpDownNodeCreator.h"
#include "InterfaceNodeCreator.h"
#include <memory>

class GraphTreeCreatorHato final : public IGraphTreeCreator
{
public:
	GraphTreeCreatorHato() = default;
	~GraphTreeCreatorHato() = default;

	EGraphSearchResult createGraphTree(const SNode& _current_node, const MapState* const _p_map, std::vector<SNode>& _output_graph, int& _make_node_num) override;

private:

	//_out_put_graph�̒l�����Z�b�g���Ă���C_current_node�̎q�m�[�h�𐶐����āC_output_graph�ɑ������D
	void makeNewNodesByCurrentNode(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph);

	const MapState* mp_Map;

	LegHierarchyNodeCreator m_LegHierarchy;
	LegUpDownNodeCreator m_LegUpDown;
	std::unique_ptr<INodeCreator> m_LegUp;
	std::unique_ptr<INodeCreator> m_LegDown;
	ComUpDownNodeCreator m_ComUpDown;
	ComMoveNodeCreator m_ComMove;
};


//! @file GraphTreeCreatorHato.h 
//! @brief �g������̃O���t���쐬����v���O�������ڐA�����N���X�̎���
//! @date 2023/7/23
//! @auther ���J��

//! @class GraphTreeCreatorHato
//! @brief �g������̃O���t���쐬����v���O�������ڐA��������
//! @details ���Ƃ��Ƃ̃v���O�����ōs��ꂽ�����̒��ŃR�����g�A�E�g����Ă������̂͂��ׂč폜�����̂ŁC�m�肽����Ήߋ��̃v���O�������Q�Ƃ��邱�ƁD
//! @date 2023/7/23
//! @auther ���J��