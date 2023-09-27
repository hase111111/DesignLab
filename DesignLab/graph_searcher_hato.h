//! @file graph_searcher_hato.h
//! @brief �g������̎�@�ŃO���t�T�����s���N���X�̎����D

#ifndef DESIGNLAB_GRAPH_SEARCHER_HATO_H_
#define DESIGNLAB_GRAPH_SEARCHER_HATO_H_


#include "interface_graph_searcher.h"


//! @class GraphSearcherHato
//! @brief �g����y�̎�@�ŁC�O���t�T�����s���N���X�D
class GraphSearcherHato final : public IGraphSearcher
{
public:

	GraphSearcherHato(const std::shared_ptr<const AbstractHexapodStateCalculator>& calc);
	~GraphSearcherHato();

	GraphSearchResult SearchGraphTree(const std::vector<SNode>& graph, const STarget& target, SNode* output_result) override;

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

	const std::shared_ptr<const AbstractHexapodStateCalculator> mp_calculator;	//!< �w�L�T�|�b�h�̏�Ԃ��v�Z����N���X
};


#endif	//DESIGNLAB_GRAPH_SEARCHER_HATO_H_