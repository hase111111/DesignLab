#pragma once
#include "InterfaceGraphTreeCreator.h"
#include "InterfaceGraphSearcher.h"
#include <memory>
#include "map_state.h"

class AbstractPassFinderFactory
{
public:
	AbstractPassFinderFactory() = default;
	virtual ~AbstractPassFinderFactory() = default;

	//! @brief �p�X�T���ɕK�v�ȃN���X�𐶐�����D
	//! @param [in] map �}�b�v���D
	//! @param [out] tree ���e�p�^�[�������̒T���ɕK�v�Ȗ؍\���𐶐�����N���X�D
	virtual void createGraphTreeCreator(const MapState* const map, std::unique_ptr<IGraphTreeCreator>& tree) = 0;

	//! @param [out] searcher ���e�p�^�[�������̒T�����s���N���X�D
	virtual void createGraphSearcher(std::unique_ptr<IGraphSearcher>& searcher) = 0;
};
