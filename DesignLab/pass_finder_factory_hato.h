#pragma once

#include "interface_pass_finder_factory.h"


//! @class PassFinderFactoryHato
//! @date 2023/08/14
//! @author ���J��
//! @brief �p�X�T���N���X�̃t�@�N�g���[�N���X
class PassFinderFactoryHato final : public IPassFinderFactory
{
public:
	PassFinderFactoryHato() = default;
	~PassFinderFactoryHato() = default;

	void createGraphTreeCreator(const MapState* const map, std::shared_ptr<AbstractHexapodStateCalculator> calc, std::unique_ptr<IGraphTreeCreator>& tree) override;

	void createGraphSearcher(std::unique_ptr<AbstractGraphSearcher>& searcher, std::shared_ptr<AbstractHexapodStateCalculator> calc) override;
};


//! @file pass_finder_factory_hato.h
//! @date 2023/08/14
//! @author ���J��
//! @brief �p�X�T���N���X�̃t�@�N�g���[�N���X
//! @n �s�� : @lineinfo
