#pragma once
#include "InterfaceGraphSearcher.h"


class GraphSearcherHato final : public IGraphSearcher
{
public:

	GraphSearcherHato();
	~GraphSearcherHato();

	EGraphSearchResult searchGraphTree(const std::vector<SNode>& graph, const STarget& target, SNode* output_result) override;

private:

	const float MARGIN_OF_MOVE = 10;
};

//! @file GraphSearcherHato
//! @brief �g������̎�@�ŃO���t�T�����s���N���X�̎����D
//! @date 2023/07/23
//! @auther ���J��

//! @class GraphSearcherHato
//! @brief �g����y�̎�@�ŁC�O���t�T�����s���N���X�D
//! @date 2023/07/23
//! @auther ���J��