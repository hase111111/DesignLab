#pragma once
#include "MapState.h"
#include "Node.h"
#include "InterfacePassFinder.h"
#include <memory>

class PassFinderNone final :public IPassFinder
{
public:
	PassFinderNone(std::unique_ptr<AbstractPassFinderFactory>&& _factory) : IPassFinder(std::move(_factory)) {};
	~PassFinderNone() = default;

	EGraphSearchResult getNextNodebyGraphSearch(const SNode& _current_node, const MapState* const _p_map, const STarget& _target, SNode& _output_node) override;

private:

};


//! @file PassFinderNone.h 
//! @brief �O���t�T�����s��Ȃ��N���X�̎����D
//! @author ���J��
//! @date 2023/07/24

//! @class PassFinderNone
//! @brief �O���t�T�����s��Ȃ��N���X�DGraphic�N���X�̃f�o�b�O���s���ۂɗp����D
//! @author ���J��
//! @date 2023/07/24
