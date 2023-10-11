//! @file graph_search_const.h
//! @brief �O���t�T���̒萔���܂Ƃ߂��N���X

#ifndef DESIGNLAB_GRAPH_SEARCH_CONST_H_
#define DESIGNLAB_GRAPH_SEARCH_CONST_H_


//! @clas GraphSearchConst
//! @brief �O���t�T���̒萔���܂Ƃ߂��N���X
class GraphSearchConst final
{
public:

	//���̂������ł��Ȃ��悤�ɃR���X�g���N�^��private�ɂ���
	GraphSearchConst() = default;
	GraphSearchConst(const GraphSearchConst&) = default;
	GraphSearchConst(GraphSearchConst&&) = default;
	GraphSearchConst& operator=(const GraphSearchConst&) = default;


	static const int kMaxDepth;	//!< �O���t�T���̍ő�[��

	static constexpr int kMaxNodeNum = 100000000;	//!< �O���t�T���̍ő�m�[�h��

};


#endif