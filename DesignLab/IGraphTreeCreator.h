#pragma once
#include "MapState.h"
#include "Node.h"
#include <vector>

class IGraphTreeCreator
{
public:

	IGraphTreeCreator() = default;
	virtual ~IGraphTreeCreator() = default;	

	//! @brief �؃O���t���쐬����N���X�D���������O���t�͎Q�Ɠn������D
	//! @param _current_node [in] ���݂̏�Ԃ�\���m�[�h
	//! @param _p_map [in] ���݂̃}�b�v�̏��
	//! @param _output_graph [out] �o�͂����؃O���t
	//! @return bool �����ɐ��������Ȃ��true
	virtual bool createGraphTree(const SNode& _current_node, const MapState* const _p_map, std::vector<SNode>& _output_graph) = 0;

};


//! @file IGraphTreeCreator.h
//! @brief �O���t�؂̒T�����s���N���X�̃C���^�[�t�F�C�X�D
//! @author ���J��
 
//! @class IGraphTreeCreator
//! @brief �O���t�؂��쐬����N���X�̃C���^�[�t�F�[�X�D���͍̂쐬�ł��Ȃ��̂ł�����p�����Ă��N���X���g�����ƁD
//! @details �p��������N���X�̃f�X�g���N�^��virtual�ɂ��Ă����D<br> �Q�l https://www.yunabe.jp/docs/cpp_virtual_destructor.html
