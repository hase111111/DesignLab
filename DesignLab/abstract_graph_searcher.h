#pragma once

#include <vector>
#include <memory>

#include "node.h"
#include "Target.h"
#include "graph_search_result.h"
#include "abstract_hexapod_state_calculator.h"


//! @class AbstractGraphSearcher
//! @date 2023/08/14
//! @author ���J��
//! @brief �O���t�؂��쐬���钊�ۃN���X�D���͍̂쐬�ł��Ȃ��̂ł�����p�����Ă��N���X���g���Ă��������D
//! @n �p���̎d����g������������Ȃ��ꍇ�́CGraphSearcherSample�����Ă݂Ă��������D
class AbstractGraphSearcher
{
public:

	AbstractGraphSearcher(const std::shared_ptr<const AbstractHexapodStateCalculator>& calc) : mp_calculator(calc) {};

	virtual ~AbstractGraphSearcher() = default;		//!< �p��������N���X�̃f�X�g���N�^��virtual�ɂ��Ă����D�Q�l https://www.yunabe.jp/docs/cpp_virtual_destructor.html


	//! @brief �O���t���󂯎��C���̒�����œK�Ȏ��̓�����o�͂���D
	//! @param graph [in] �O���t��
	//! @param target [in] �ڕW�n�_
	//! @param output_result [out] �o�͂����m�[�h
	//! @return EGraphSearchResult �T���̌���
	virtual EGraphSearchResult searchGraphTree(const std::vector<SNode>& graph, const STarget& target, SNode* output_result) = 0;

protected:

	const std::shared_ptr<const AbstractHexapodStateCalculator> mp_calculator;	//!< �w�L�T�|�b�h�̏�Ԃ��v�Z����N���X
};


//! @file abstract_graph_searcher.h
//! @date 2023/08/14
//! @author ���J��
//! @brief �O���t�؂��쐬���钊�ۃN���X�D
//! @n �s�� : @lineinfo
