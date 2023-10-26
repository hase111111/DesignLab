//! @file interface_node_creator.h
//! @brief �m�[�h�����C���^�[�t�F�[�X

#ifndef DESIGNLAB_INTERFACE_NODE_CREATOR_H_
#define DESIGNLAB_INTERFACE_NODE_CREATOR_H_


#include <vector>

#include "robot_state_node.h"


//! @class INodeCreator
//! @brief �m�[�h�����C���^�[�t�F�[�X
class INodeCreator
{
public:

	//! @brief �R���X�g���N�^�ł͎������ݒ肷��D�܂��}�b�v�̃|�C���^���󂯎��
	INodeCreator() = default;
	virtual ~INodeCreator() = default;


	//! @brief ���݂̃m�[�h���玟�̃m�[�h�Q�𐶐�����
	//! @param[in] current_node ���݂̃m�[�h
	//! @param[in] current_node_index ���݂̃m�[�h�̃C���f�b�N�X
	//! @param[out] output_graph ���������m�[�h�Q��Ԃ�
	virtual void Create(const RobotStateNode& current_node, int current_node_index, std::vector<RobotStateNode>* output_graph) const = 0;
};


#endif // DESIGNLAB_INTERFACE_NODE_CREATOR_H_