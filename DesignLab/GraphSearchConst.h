#pragma once

class GraphSearchConst final
{
public:

	static constexpr int MAX_DEPTH = 4;	//!< �O���t�T���̍ő�[��

	static constexpr int MAX_NODE_NUM = 100000000;	//!< �O���t�T���̍ő�m�[�h��

	static constexpr bool DO_DEBUG_PRINT = true;	//!< �f�o�b�O�p�̏o�͂����邩�ǂ���

private:

	//���̂������ł��Ȃ��悤�ɃR���X�g���N�^��private�ɂ���
	GraphSearchConst() = default;
	GraphSearchConst(const GraphSearchConst&) = default;
};
