#pragma once
#include "MapState.h"
#include "HexapodStateCalculator.h"

//�r�̏グ����������G�b�W(�ӁC�m�[�h�ƃm�[�h���q����)�̏���������N���X�D���̂̉�]����؍l�����Ă��Ȃ��̂ŁC������l���������ꍇ��蒼�����V�������̂�����Ă��������D
class LegUpDownNodeCreator final
{
public:

	// _output_graph�̌��ɋr���㉺�������V�����m�[�h��push����D
	void create(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph);

private:

};
