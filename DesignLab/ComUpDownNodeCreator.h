#pragma once
#include "InterfaceNodeCreator.h"


class ComUpDownNodeCreator final : public INodeCreator
{
public:
	ComUpDownNodeCreator(const MapState* const p_map, const EHexapodMove next_move);
	~ComUpDownNodeCreator();

	void create(const SNode& current_node, const int current_num, std::vector<SNode>* output_graph) override;

private:

	const MapState* const mp_map;

	// �O���[�o�����W�̏d�S�̍Œ�ʒu�ƍō��ʒu����C�d�S���㉺�ɕω��������m�[�h��ǉ�����D
	void pushNodeByMaxAndMinPosZ(const SNode& current_node, const int current_num, const float high, const float low, std::vector<SNode>* output_graph);

	static constexpr int DISCRETIZATION = 5;	//���U�����D�ő�ʒu���ŏ��ʒu������������̂��D

	static constexpr float MARGIN = 10.0f;		//�r��L�΂��؂�Ȃ��悤�ɂ��邽�߂̃}�[�W��[mm]�D���l�͐�y�̃v���O��������Ƃ��Ă����̂łȂ����̐��l���ǂ��̂��͂킩��Ȃ��D
};

//! @file ComUpDownNodeCreator.h
//! @brief �d�S�̏グ����������G�b�W(�ӁC�m�[�h�ƃm�[�h���q����)�̏���������N���X�D
//! @date 2023/7/24
//! @auther ���J��

//! @class ComUpDownNodeCreator
//! @brief �d�S�̏グ����������G�b�W(�ӁC�m�[�h�ƃm�[�h���q����)�̏���������N���X�D
//! @date 2023/7/24
//! @auther ���J��