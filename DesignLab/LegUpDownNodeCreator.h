#pragma once
#include "MapState.h"
#include "HexapodStateCalculator.h"
#include "MapState.h"

//�r�̏グ����������G�b�W(�ӁC�m�[�h�ƃm�[�h���q����)�̏���������N���X�D���̂̉�]����؍l�����Ă��Ȃ��̂ŁC������l���������ꍇ��蒼�����V�������̂�����Ă��������D
class LegUpDownNodeCreator final
{
public:

	// _output_graph�̌��ɋr���㉺�������V�����m�[�h��push����D
	void create(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph);

private:

	const MapState* mp_Map;

	EHexapodMove getNextMove(const EHexapodMove& _last_move) const;

	//�r���ڒn�\�����ׂ�D�n�ʂɊ����邩�ǂ����𒲂ׂĂ��Ȃ��̂Œ��ӁD���ۂɐڒn����Ƃ�����ǂ��ɂȂ邩��output_ground_pos�ŏo�͂���D
	bool isGroundableLeg(const int _leg_num, const SNode& _current_node, SNode& _output_ground_pos);

	const float HORIZONTAL_MARGIN = 50.0f;	//���ꂾ���������Ό��݂̋r�ʒu�ł��͂��̂Ȃ�΁C�r�ʒu4����ƂȂ�D
};
