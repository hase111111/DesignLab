#pragma once
#include "MapState.h"
#include "Node.h"
#include "Target.h"
#include "GraphSearchResult.h"
#include "InterfaceGraphTreeCreator.h"
#include "InterfaceGraphSearcher.h"
#include "AbstractPassFinderFactory.h"

class IGraphSearch
{
public:

	IGraphSearch(std::unique_ptr<AbstractPassFinderFactory>&& _factory) : mp_PassFinderFactory(std::move(_factory)) {};
	virtual ~IGraphSearch() = default;

	//! @brief �O���t�T�����s���C���̓���Ƃ��čœK�ȃm�[�h��Ԃ��D
	//! @param [in] _current_node ���݂̏�Ԃ�\���m�[�h
	//! @param [in] _p_map ���݂̃}�b�v�̏��
	//!	@param [in] _target �ڕW�̏��
	//! @param [out] _output_node ���ʂ̃m�[�h
	//! @return EGraphSearchResult �O���t�T���̌��ʂ�Ԃ��D
	virtual EGraphSearchResult getNextNodebyGraphSearch(const SNode& _current_node, const MapState* const _p_map, const STarget& _target, SNode& _output_node) = 0;

	//! @brief �쐬�����O���t�̐���Ԃ�
	//! @return int �쐬�����O���t�̐�
	int getMadeNodeNum() const { return m_made_node_num; }

protected:

	int m_made_node_num = 0;	//!< �쐬�����O���t�̐�

	std::unique_ptr<AbstractPassFinderFactory> mp_PassFinderFactory;	//!< �p�X�T���N���X�̃t�@�N�g���[
	std::unique_ptr<IGraphTreeCreator> mp_GraphTreeCreator;	//!< �O���t�؂̍쐬�N���X
	std::unique_ptr<IGraphSearcher> mp_GraphSearcher;	//!< �O���t�T���N���X
};

//! @file IGraphSearch
//! @brief �O���t�T�����s���N���X�̃C���^�[�t�F�C�X�̎���
//! @author ���J��
//! @date 2023/07/24

//! @class IGraphSearch
//! @brief �O���t�T�����s���N���X�̃C���^�[�t�F�C�X�D���͍̂쐬�ł��Ȃ��̂ł�����p�����Ă��N���X���g�����ƁD
//! @date 2023/07/24
//! @auther ���J��
//! @details �p��������N���X�̃f�X�g���N�^��virtual�ɂ��Ă����D<br>
//! �Q�l https://www.yunabe.jp/docs/cpp_virtual_destructor.html