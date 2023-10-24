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

	GraphSearchResult SearchGraphTree(const std::vector<RobotStateNode>& graph, const TargetRobotState& target, RobotStateNode* output_result) override;

private:

	//! @brief ������Ȃ���-1��������
	size_t GetParentNodeIndex(const std::vector<RobotStateNode>& graph) const;

	//! @brief ������Ȃ���false��������DMAX_DEPTH�����̂ڂ��Ă�������Ȃ��ꍇ��false��������
	//! @param [in] graph �O���t
	//! @param [in] max_depth_node_index �ő�[���̃m�[�h�̃C���f�b�N�X
	//! @param [out] put_node �e�m�[�h�̏����i�[����
	//! @return bool �����������ǂ���
	bool GetDepth1NodeFromMaxDepthNode(const std::vector<RobotStateNode>& graph, size_t max_depth_node_index, RobotStateNode* output_node) const;

	void InitEvaluationValue(const RobotStateNode& parent_node, const TargetRobotState& target);

	//! @brief �O�i���邽�߂̕]���l���v�Z����
	float CalcMoveFrowardEvaluationValue(const RobotStateNode& current_node, const TargetRobotState& target) const;

	//! @brief �r�̕��ω�]�ʂ̕]���l���v�Z����
	float CalcLegRotEvaluationValue(const RobotStateNode& current_node, const TargetRobotState& target) const;


	const float kMarginOfMove = 10;


	RobotStateNode parent_node_;

	const std::shared_ptr<const AbstractHexapodStateCalculator> calculator_ptr_;	//!< �w�L�T�|�b�h�̏�Ԃ��v�Z����N���X
};


#endif	//DESIGNLAB_GRAPH_SEARCHER_HATO_H_