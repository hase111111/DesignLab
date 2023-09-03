#pragma once

#include <vector>

#include "map_state.h"
#include "node.h"
#include "graph_search_result.h"
#include "interface_node_creator.h"
#include "graph_search_const.h"


//! @class IGraphTreeCreator
//! @brief �O���t�؂��쐬����N���X�̃C���^�[�t�F�[�X�D���͍̂쐬�ł��Ȃ��̂ł�����p�����Ă��N���X���g�����ƁD
//! @details �p��������N���X�̃f�X�g���N�^��virtual�ɂ��Ă����D<br> 
//! �Q�l https://www.yunabe.jp/docs/cpp_virtual_destructor.html
//! @date 2023/07/23
//! @author ���J��
class IGraphTreeCreator
{
public:

	IGraphTreeCreator(std::map<EHexapodMove, std::unique_ptr<INodeCreator>>& map);
	virtual ~IGraphTreeCreator() = default;

	//! @brief �؃O���t���쐬����N���X�D���������O���t�͎Q�Ɠn������D
	//! @param current_node [in] ���݂̏�Ԃ�\���m�[�h
	//! @param p_map [in] ���݂̃}�b�v�̏��
	//! @param output_graph [out] �o�͂����؃O���t
	//! @return EGraphSearchResult �����ɐ��������Ȃ��true
	virtual EGraphSearchResult createGraphTree(const SNode& current_node, const MapState* const p_map, std::vector<SNode>* output_graph) = 0;

	void setMaxDepth(const int max_depth) { m_max_depth = max_depth; };

protected:

	int getMaxDepth() const { return m_max_depth; };

	std::map<EHexapodMove, std::unique_ptr<INodeCreator>> m_node_creator_map;

	int m_max_depth = GraphSearchConst::MAX_DEPTH;
};


//! @file interface_graph_tree_creator.h
//! @date 2023/09/03
//! @author ���J��
//! @brief �O���t�؂̒T�����s���N���X�̃C���^�[�t�F�C�X�D
//! @n �s�� : @lineinfo
