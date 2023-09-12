#pragma once

#include <memory>

#include "interface_graph_tree_creator.h"
#include "abstract_hexapod_state_calculator.h"
#include "abstract_graph_searcher.h"
#include "map_state.h"


//! @class IPassFinderFactory
//! @date 2023/08/21
//! @author ���J��
//! @brief �p�X�T���ɕK�v�ȃN���X�𐶐�����t�@�N�g���[�N���X�D
class IPassFinderFactory
{
public:
	IPassFinderFactory() = default;
	virtual ~IPassFinderFactory() = default;

	//! @brief �O���t�؂̐����ɕK�v�ȃN���X�𐶐�����D
	//! @param [in] map �}�b�v���D
	//! @param [out] tree ���e�p�^�[�������̒T���ɕK�v�Ȗ؍\���𐶐�����N���X�D
	virtual void createGraphTreeCreator(const MapState* const map, std::shared_ptr<AbstractHexapodStateCalculator> calc, std::unique_ptr<IGraphTreeCreator>& tree) = 0;

	//! @brief �O���t�T�����s���N���X�𐶐�����D
	//! @param [out] searcher ���e�p�^�[�������̒T�����s���N���X�D
	virtual void createGraphSearcher(std::unique_ptr<AbstractGraphSearcher>& searcher, std::shared_ptr<AbstractHexapodStateCalculator> calc) = 0;
};


//! @file interface_pass_finder_factory.h
//! @date 2023/08/21
//! @author ���J��
//! @brief �p�X�T���ɕK�v�ȃN���X�𐶐�����t�@�N�g���[�N���X�D
//! @n �s�� : @lineinfo
