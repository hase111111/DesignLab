#pragma once
#include "InterfaceGraphTreeCreator.h"
#include "InterfaceGraphSearcher.h"
#include <memory>
#include "MapState.h"

class AbstractPassFinderFactory
{
public:
	AbstractPassFinderFactory() = default;
	virtual ~AbstractPassFinderFactory() = default;

	//! @brief �p�X�T���ɕK�v�ȃN���X�𐶐�����D
	//! @param [out] _tree ���e�p�^�[�������̒T���ɕK�v�Ȗ؍\���𐶐�����N���X�D
	//! @param [out] _searcher ���e�p�^�[�������̒T�����s���N���X�D
	virtual void createPassFinder(std::unique_ptr<IGraphTreeCreator>& _tree, std::unique_ptr<IGraphSearcher>& _searcher, const MapState* const _map) = 0;

};
