#pragma once
#include "IGraphTreeCreator.h"

// IGraphTreeCreator ���p���������ꍇ�D�ȉ��̂悤��
// 
//	class �D���Ȗ��O final : public IGraphTreeCreator
//	{
//	}
// 
// �Ɛ錾���܂��Dfinal�͂�������ȏ�p�����Ȃ���Ƃ����Ӗ��Dpublic IGraphTreeCreator �͂��̃N���X���p��������Ƃ����Ӗ��ł��D
// IGraphTreeCreator���p�������N���X�ɉۂ����鐧��͂�����CIGraphTreeCreator �̏������z�֐��CcreateGraphTree���������邱�Ƃł��D
// �ȉ��̂悤��createGraphTree��public�ȂƂ���ɐ錾���āC����override�Ƃ��Ă��������D


//�O���t���쐬����N���X�̃T���v���ł��D�e�L�g�[�ɃO���t���쐬���܂��D
class GraphTreeCreatorSample final : public IGraphTreeCreator
{
public:
	GraphTreeCreatorSample() = default;
	~GraphTreeCreatorSample() = default;

	bool createGraphTree(const SNode& _current_node, const MapState* const _p_map, std::vector<SNode>& _output_graph) override;

private:

};
