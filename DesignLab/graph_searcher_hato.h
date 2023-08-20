#pragma once

#include "InterfaceGraphSearcher.h"


//! @class GraphSearcherHato
//! @date 2023/08/14
//! @author ���J��
//! @brief �g����y�̎�@�ŁC�O���t�T�����s���N���X�D
class GraphSearcherHato final : public IGraphSearcher
{
public:

	GraphSearcherHato();
	~GraphSearcherHato();

	EGraphSearchResult searchGraphTree(const std::vector<SNode>& graph, const STarget& target, SNode* output_result) override;

private:

	//! @brief ������Ȃ���-1��������
	size_t getParentNodeIndex(const std::vector<SNode>& graph) const;

	//! @brief ������Ȃ���false��������DMAX_DEPTH�����̂ڂ��Ă�������Ȃ��ꍇ��false��������
	//! @param [in] graph �O���t
	//! @param [in] max_depth_node_index �ő�[���̃m�[�h�̃C���f�b�N�X
	//! @param [out] put_node �e�m�[�h�̏����i�[����
	//! @return bool �����������ǂ���
	bool getDepth1NodeFromMaxDepthNode(const std::vector<SNode>& graph, size_t max_depth_node_index, SNode* output_node) const;

	void initEvaluationValue(const SNode& parent_node, const STarget& target);

	//! @brief �O�i���邽�߂̕]���l���v�Z����
	float calcMoveFrowardEvaluationValue(const SNode& current_node, const STarget& target) const;

	//! @brief �r�̕��ω�]�ʂ̕]���l���v�Z����
	float calcLegRotEvaluationValue(const SNode& current_node, const STarget& target) const;


	const float MARGIN_OF_MOVE = 10;


	SNode m_parent_node;
};


//! @file graph_searcher_hato.h
//! @date 2023/08/14
//! @author ���J��
//! @brief �g������̎�@�ŃO���t�T�����s���N���X�̎����D
//! @n �s�� : @lineinfo