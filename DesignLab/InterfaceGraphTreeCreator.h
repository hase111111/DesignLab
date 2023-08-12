#pragma once
#include "map_state.h"
#include "Node.h"
#include <vector>
#include "GraphSearchResult.h"
#include "interface_node_creator.h"
#include "GraphSearchConst.h"

class IGraphTreeCreator
{
public:

	IGraphTreeCreator(std::map<EHexapodMove, std::unique_ptr<INodeCreator>>& map);
	virtual ~IGraphTreeCreator() = default;

	//! @brief �؃O���t���쐬����N���X�D���������O���t�͎Q�Ɠn������D
	//! @param current_node [in] ���݂̏�Ԃ�\���m�[�h
	//! @param p_map [in] ���݂̃}�b�v�̏��
	//! @param output_graph [out] �o�͂����؃O���t
	//! @param make_node_num [out] �쐬�����m�[�h�̐�
	//! @return EGraphSearchResult �����ɐ��������Ȃ��true
	virtual EGraphSearchResult createGraphTree(const SNode& current_node, const MapState* const p_map, std::vector<SNode>* output_graph, int* make_node_num) = 0;

	void setMaxDepth(const int max_depth) { m_max_depth = max_depth; };

protected:

	int getMaxDepth() const { return m_max_depth; };

	std::map<EHexapodMove, std::unique_ptr<INodeCreator>> m_node_creator_map;

	int m_max_depth = GraphSearchConst::MAX_DEPTH;
};


//! @file InterfaceGraphTreeCreator.h
//! @brief �O���t�؂̒T�����s���N���X�̃C���^�[�t�F�C�X�D
//! @author ���J��

//! @class IGraphTreeCreator
//! @brief �O���t�؂��쐬����N���X�̃C���^�[�t�F�[�X�D���͍̂쐬�ł��Ȃ��̂ł�����p�����Ă��N���X���g�����ƁD
//! @details �p��������N���X�̃f�X�g���N�^��virtual�ɂ��Ă����D<br> 
//! �Q�l https://www.yunabe.jp/docs/cpp_virtual_destructor.html
//! @date 2023/07/23
//! @author ���J��
