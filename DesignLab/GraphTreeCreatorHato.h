#pragma once
#include "IGraphTreeCreator.h"
#include "MapState.h"
#include "ComMoveNodeCreator.h"
#include "ComUpDownNodeCreator.h"
#include "LegHierarchyNodeCreator.h"
#include "LegUpDownNodeCreator.h"

class GraphTreeCreatorHato final : public IGraphTreeCreator
{
public:
	GraphTreeCreatorHato() = default;
	~GraphTreeCreatorHato() = default;

	//! @brief �O���t���쐬����
	//! @param _current_node ���݂̃m�[�h
	//! @param _p_map �}�b�v�̏��
	//! @param _output_graph �쐬�����O���t���i�[����
	//! @return �O���t�̍쐬�ɐ����������ǂ���
	//! @details ���̊֐��́C_current_node�̎q�m�[�h�𐶐����āC_output_graph�ɑ������D<br>
	//! _output_graph�̒l�����Z�b�g���Ă���������D<br>
	//! ���̊֐��́C_current_node�̎q�m�[�h�𐶐����邽�߂ɁCmakeNewNodesByCurrentNode�֐����Ăяo���D<br>
	bool createGraphTree(const SNode& _current_node, const MapState* const _p_map, std::vector<SNode>& _output_graph) override;

private:

	//_out_put_graph�̒l�����Z�b�g���Ă���C_current_node�̎q�m�[�h�𐶐����āC_output_graph�ɑ������D
	void makeNewNodesByCurrentNode(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph);

	const MapState * mp_Map;

	LegHierarchyNodeCreator m_LegHierarchy;
	LegUpDownNodeCreator m_LegUpDown;
	ComUpDownNodeCreator m_ComUpDown;
	ComMoveNodeCreator m_ComMove;
};


//! @file GraphTreeCreatorHato.h 
//! @brief �g������̃O���t���쐬����v���O�������ڐA�����N���X�̎���

//! @class GraphTreeCreatorHato
//! @brief �g������̃O���t���쐬����v���O�������ڐA��������
//! @details ���Ƃ��Ƃ̃v���O�����ōs��ꂽ�����̒��ŃR�����g�A�E�g����Ă������̂͂��ׂč폜�����̂ŁC�m�肽����Ήߋ��̃v���O�������Q�Ƃ��邱�ƁD