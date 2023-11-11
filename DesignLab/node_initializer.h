//! @file node_initializer.h
//! @brief �m�[�h�̏��������s���N���X�D

#ifndef DESIGNLAB_NODE_INITIALIZER_H_
#define DESIGNLAB_NODE_INITIALIZER_H_


#include "robot_state_node.h"


//! @class NodeInitializer
//! @brief �m�[�h�̏��������s���N���X�D
//! @n �V�~�����[�V�������Ƀm�[�h�̏����l��ݒ肷�邽�߂Ɏg�p����D
class NodeInitializer final
{
public:

	RobotStateNode InitNode() const;
};


#endif	// DESIGNLAB_NODE_INITIALIZER_H_