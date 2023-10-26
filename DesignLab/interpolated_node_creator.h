//! @file interpolated_node_creator.h
//! @brief �m�[�h�Ԃ��Ԃ���N���X


#ifndef INTERPOLATED_NODE_CREATOR_H_
#define INTERPOLATED_NODE_CREATOR_H_

#include <vector>

#include "robot_state_node.h"


//! @class InterpolatedNodeCreator
//! @brief �m�[�h�Ԃ��Ԃ���N���X�D��`�O���𐶐�����
class InterpolatedNodeCreator
{
public:
	InterpolatedNodeCreator() = default;


	//! @brief �m�[�h�Ԃ��Ԃ���
	//! @param[in]	node ���݂̃m�[�h
	//! @param[in]	next_node ���̃m�[�h
	//! @param[out]	interpolated_node ��Ԃ��ꂽ�m�[�h
	void CreateInterpolatedNode(const RobotStateNode& node, const RobotStateNode& next_node, std::vector<RobotStateNode>* interpolated_node) const;

private:

	const int INTERPOLATED_NODE_NUM1 = 5;		//��Ԃ���m�[�h�̐�
	const int INTERPOLATED_NODE_NUM2 = 5;		//��Ԃ���m�[�h�̐�
	const int INTERPOLATED_NODE_NUM = INTERPOLATED_NODE_NUM1 + INTERPOLATED_NODE_NUM2;		//��Ԃ���m�[�h�̐�
};


#endif