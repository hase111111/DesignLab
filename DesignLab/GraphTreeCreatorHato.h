#pragma once
#include "IGraphTreeCreator.h"
#include "MapState.h"
#include "ComUpDownNodeCreator.h"
#include "LegHierarchyNodeCreator.h"
#include "LegUpDownNodeCreator.h"

class GraphTreeCreatorHato final : public IGraphTreeCreator
{
public:
	GraphTreeCreatorHato() = default;
	~GraphTreeCreatorHato() = default;

	bool createGraphTree(const SNode& _current_node, const MapState* const _p_map, std::vector<SNode>& _output_graph) override;

private:

	//_out_put_graph�̒l�����Z�b�g���Ă���C_current_node�̎q�m�[�h�𐶐����āC_output_graph�ɑ������D
	void makeNewNodesByCurrentNode(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph);

	const MapState * mp_Map;

	LegHierarchyNodeCreator m_LegHierarchy;
	LegUpDownNodeCreator m_LegUpDown;
	ComUpDownNodeCreator m_ComUpDown;
};


//! @file GraphTreeCreatorHato.h 
//! @brief �g������̃O���t���쐬����v���O�������ڐA�����N���X�̎���

//! @class GraphTreeCreatorHato
//! @brief �g������̃O���t���쐬����v���O�������ڐA��������
//! @details ���Ƃ��Ƃ̃v���O�����ōs��ꂽ�����̒��ŃR�����g�A�E�g����Ă������̂͂��ׂč폜�����̂ŁC�m�肽����Ήߋ��̃v���O�������Q�Ƃ��邱�ƁD