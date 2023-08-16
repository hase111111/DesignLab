#pragma once

#include "InterfaceGraphSearcher.h"

// IGraphSearcher ���p���������ꍇ�D�ȉ��̂悤��
// 
//	class �D���Ȗ��O final : public IGraphSearcher
//	{
//	}
// 
// �Ɛ錾���܂��Dfinal�͂�������ȏ�p�����Ȃ���Ƃ����Ӗ��Dpublic IGraphSearcher�͂��̃N���X���p��������Ƃ����Ӗ��ł��D
// IGraphSearcher���p�������N���X�ɉۂ����鐧��͂�����CIGraphSearcher�̏������z�֐��CsearchGraphTree���������邱�Ƃł��D
// �ȉ��̂悤��searchGraphTree��public�ȂƂ���ɐ錾���āC����override�Ƃ��Ă��������D



// GraphSearcherSample�N���X�̓e�L�g�[�Ɏ��̓����I��ŕԂ��܂��D�����܂ł��̃N���X��IGraphSearcher�̐����p�ł��D
class GraphSearcherSample final : public IGraphSearcher
{
public:
	GraphSearcherSample() = default;
	~GraphSearcherSample() = default;

	EGraphSearchResult searchGraphTree(const std::vector<SNode>& graph, const STarget& target, SNode* output_result) override;

private:

};
