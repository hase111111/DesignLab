#pragma once
#include "MapState.h"
#include "Node.h"
#include <vector>
#include "GraphSearchResult.h"
#include "InterfaceNodeCreator.h"

class IGraphTreeCreator
{
public:

	IGraphTreeCreator(std::map<EHexapodMove, std::unique_ptr<INodeCreator>>& _map);
	virtual ~IGraphTreeCreator() = default;

	//! @brief �؃O���t���쐬����N���X�D���������O���t�͎Q�Ɠn������D
	//! @param _current_node [in] ���݂̏�Ԃ�\���m�[�h
	//! @param _p_map [in] ���݂̃}�b�v�̏��
	//! @param _output_graph [out] �o�͂����؃O���t
	//! @param _make_node_num [out] �쐬�����m�[�h�̐�
	//! @return EGraphSearchResult �����ɐ��������Ȃ��true
	virtual EGraphSearchResult createGraphTree(const SNode& _current_node, const MapState* const _p_map, std::vector<SNode>& _output_graph, int& _make_node_num) = 0;

protected:

	std::map<EHexapodMove, std::unique_ptr<INodeCreator>> m_node_creator_map;
};


//! @file InterfaceGraphTreeCreator.h
//! @brief �O���t�؂̒T�����s���N���X�̃C���^�[�t�F�C�X�D
//! @author ���J��

//! @class IGraphTreeCreator
//! @brief �O���t�؂��쐬����N���X�̃C���^�[�t�F�[�X�D���͍̂쐬�ł��Ȃ��̂ł�����p�����Ă��N���X���g�����ƁD
//! @details �p��������N���X�̃f�X�g���N�^��virtual�ɂ��Ă����D<br> 
//! �Q�l https://www.yunabe.jp/docs/cpp_virtual_destructor.html
//! @date 2023/07/23
//! @auther ���J��
