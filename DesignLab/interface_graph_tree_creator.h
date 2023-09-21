//! @file interface_graph_tree_creator.h
//! @brief �O���t�؂̒T�����s���N���X�̃C���^�[�t�F�C�X�D

#ifndef DESIGNLAB_INTERFACE_GRAPH_TREE_CREATOR_H_
#define DESIGNLAB_INTERFACE_GRAPH_TREE_CREATOR_H_


#include <vector>

#include "graph_search_result.h"
#include "node.h"
#include "map_state.h"


//! @class IGraphTreeCreator
//! @brief �O���t�؂��쐬����N���X�̃C���^�[�t�F�[�X�D���͍̂쐬�ł��Ȃ��̂ł�����p�����Ă��N���X���g�����ƁD
//! @details �p��������N���X�̃f�X�g���N�^��virtual�ɂ��Ă����D
//! @n �Q�l https://www.yunabe.jp/docs/cpp_virtual_destructor.html
class IGraphTreeCreator
{
public:

	IGraphTreeCreator() = default;
	virtual ~IGraphTreeCreator() = default;

	//! @brief �O���t�؂��쐬����N���X�̏��������s���D
	//! @param map_state [in] �������ꂽ�}�b�v
	virtual void Init(const DevideMapState& map_state) = 0;

	//! @brief �O���t�؂��쐬����N���X�D���������O���t�͎Q�Ɠn������D
	//! @param current_node [in] ���݂̏�Ԃ�\���m�[�h�D�[����0�̃m�[�h��n�����ƁD
	//! @param max_depth [in] �쐬����O���t�؂̍ő�[���D�傫������l��n���Əd���Ȃ� or �����Ȃ��Ȃ�̂Œ��ӁD
	//! @param output_graph [out] �o�͂����O���t��
	//! @return EGraphSearchResult �����̌��ʁC�����ɐ����������ǂ����C���s�����ꍇ�͎��s�̗��R��Ԃ�
	virtual EGraphSearchResult CreateGraphTree(const SNode& current_node, int max_depth, std::vector<SNode>* output_graph) = 0;
};


#endif