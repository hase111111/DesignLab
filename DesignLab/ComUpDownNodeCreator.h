#pragma once
#include "MapState.h"
#include "HexapodStateCalculator.h"

//�d�S�̏グ����������G�b�W(�ӁC�m�[�h�ƃm�[�h���q����)�̏���������N���X�D���̂̉�]����؍l�����Ă��Ȃ��̂ŁC������l���������ꍇ��蒼�����V�������̂�����Ă��������D
class ComUpDownNodeCreator final
{
public:
	void init(const MapState* const _p_Map);

	void create(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph);

private:

	const MapState* mp_Map;
	HexapodStateCalclator m_HexaCalc;

	// �O���[�o�����W�̏d�S�̍Œ�ʒu�ƍō��ʒu����C�d�S���㉺�ɕω��������m�[�h��ǉ�����D
	void pushNodeByMaxAndMinPosZ(const SNode& _current_node, const int _current_num, const float _high, const float _low, std::vector<SNode>& _output_graph);


	const int DISCRETIZATION = 5;	//���U�����D�ő�ʒu���ŏ��ʒu������������̂��D
	const float MARGIN = 10.0f;		//�r��L�΂��؂�Ȃ��悤�ɂ��邽�߂̃}�[�W��[mm]�D���l�͐�y�̃v���O��������Ƃ��Ă����̂łȂ����̐��l���ǂ��̂��͂킩��Ȃ��D
	const EHexapodMove m_next_move = EHexapodMove::COM_MOVE;	//���̓���
};
