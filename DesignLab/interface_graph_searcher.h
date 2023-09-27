//! @file interface_graph_searcher.h
//! @brief �O���t�T�����s���C���^�[�t�F�[�X

#ifndef DESIGNLAB_INTERFACE_GRAPH_SEARCHER_H_
#define DESIGNLAB_INTERFACE_GRAPH_SEARCHER_H_


#include <memory>
#include <vector>

#include "abstract_hexapod_state_calculator.h"
#include "graph_search_result.h"
#include "node.h"
#include "target.h"


//! @class IGraphSearcher
//! @brief �O���t�T�����s���C���^�[�t�F�[�X�D���͍̂쐬�ł��Ȃ��̂ł�����p�����Ă��N���X���g�����ƁD
class IGraphSearcher
{
public:

	IGraphSearcher() = default;
	virtual ~IGraphSearcher() = default;		//!< �p��������N���X�̃f�X�g���N�^��virtual�ɂ��Ă����D�Q�l https://www.yunabe.jp/docs/cpp_virtual_destructor.html


	//! @brief �O���t���󂯎��C���̒�����œK�Ȏ��̓�����o�͂���D
	//! @param graph [in] �O���t��
	//! @param target [in] �ڕW�n�_
	//! @param output_result [out] �o�͂����m�[�h
	//! @return GraphSearchResult �T���̌���
	virtual GraphSearchResult SearchGraphTree(const std::vector<SNode>& graph, const STarget& target, SNode* output_result) = 0;
};


#endif	