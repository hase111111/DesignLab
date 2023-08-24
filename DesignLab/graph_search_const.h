#pragma once


//! @clas GraphSearchConst
//! @date 2023/08/14
//! @author ���J��
//! @brief �O���t�T���̒萔���܂Ƃ߂��N���X
class GraphSearchConst final
{
public:

	static constexpr int MAX_DEPTH = 5;	//!< �O���t�T���̍ő�[��

	static constexpr int MAX_NODE_NUM = 100000000;	//!< �O���t�T���̍ő�m�[�h��

	static constexpr bool DO_DEBUG_PRINT = false;	//!< �f�o�b�O�p�̏o�͂����邩�ǂ���

private:

	//���̂������ł��Ȃ��悤�ɃR���X�g���N�^��private�ɂ���
	GraphSearchConst() = default;
	GraphSearchConst(const GraphSearchConst&) = default;
};


//! @file graph_search_const.h
//! @date 2023/08/14
//! @author ���J��
//! @brief �O���t�T���̒萔���܂Ƃ߂��N���X
