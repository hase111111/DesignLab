#pragma once
#include "MapState.h"

//�d�S�̏グ����������G�b�W(�ӁC�m�[�h�ƃm�[�h���q����)�̏���������N���X�D���̂̉�]����؍l�����Ă��Ȃ��̂ŁC������l���������ꍇ��蒼�����V�������̂�����Ă��������D
class ComUpDownNodeCreator final
{
public:
	void init();

	void create(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph);

private:

	const MapState* mp_Map;

	const int DISCRETIZATION = 5;	//���U�����D�ő�ʒu���ŏ��ʒu������������̂��D
	const float MARGIN = 10.0f;		//�r��L�΂��؂�Ȃ��悤�ɂ��邽�߂̃}�[�W��[mm]�D���l�͐�y�̃v���O��������Ƃ��Ă����̂łȂ����̐��l���ǂ��̂��͂킩��Ȃ��D
};
