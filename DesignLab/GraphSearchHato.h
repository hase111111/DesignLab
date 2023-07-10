#pragma once
#include "MapState.h"
#include "Node.h"
#include "IGraphTreeCreator.h"
#include "IGraphSearcher.h"
#include "InterfaceGraphSearch.h"
#include <memory>

class GraphSearchHato final :public IGraphSearch
{
public:
	GraphSearchHato() = default;
	~GraphSearchHato() = default;

	//! @brief �O���t�T�����s���C���̓���Ƃ��čœK�ȃm�[�h��Ԃ��D
	//! @param [in] _current_node ���݂̏�Ԃ�\���m�[�h
	//! @param [in] _p_map �}�b�v�̏�Ԃ�\���N���X�ւ̃|�C���^
	//! @param [in] _target ����̖ڕW
	//! @param [out] _output_node ���ʂ̃m�[�h
	//! @return bool �O���t�T���Ɏ��s�����ꍇfalse��Ԃ�
	bool getNextNodebyGraphSearch(const SNode& _current_node, const MapState* const _p_map, const STarget& _target, SNode& _output_node) override;

private:
	std::unique_ptr<IGraphTreeCreator> mp_GraphTreeCreator;
	std::unique_ptr<IGraphSearcher> mp_GraphSearcher;
};


//! @file GraphSearch.h 
//! @brief �O���t�T�����s���N���X�̎����D
//! @author ���J��

//! @class GraphSearch
//! @brief �O���t�T�����s���N���X�D���O�ʂ�
//! @author ���J��