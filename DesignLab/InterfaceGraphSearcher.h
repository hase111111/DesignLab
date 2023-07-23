#pragma once
#include <vector>
#include "Node.h"
#include "Target.h"
#include "GraphSearchResult.h"


class IGraphSearcher
{
public:
	IGraphSearcher() = default;
	virtual ~IGraphSearcher() = default;		//!< �p��������N���X�̃f�X�g���N�^��virtual�ɂ��Ă����D�Q�l https://www.yunabe.jp/docs/cpp_virtual_destructor.html

	//! @brief �O���t���󂯎��C���̒�����œK�Ȏ��̓�����o�͂���D
	//! @param _graph [in] �O���t��
	//! @param _target [in] �ڕW�n�_
	//! @param _output_result [out] �o�͂����m�[�h
	//! @return EGraphSearchResult �T���̌���
	virtual EGraphSearchResult searchGraphTree(const std::vector<SNode>& _graph, const STarget& _target, SNode& _output_result) = 0;
};

//! @file InterfaceGraphSearcher.h
//! @brief �O���t�؂��쐬����N���X�̃C���^�[�t�F�[�X�D
//! @date 2023/07/23
//! @auther ���J��

//! @class IGraphSearcher
//! @brief �O���t�؂��쐬����N���X�̃C���^�[�t�F�[�X�D���͍̂쐬�ł��Ȃ��̂ł�����p�����Ă��N���X���g���Ă��������D<br>
//! �p���̎d����g������������Ȃ��ꍇ�́CGraphSearcherSample�����Ă݂Ă��������D
//! @date 2023/07/23
//! @author ���J��