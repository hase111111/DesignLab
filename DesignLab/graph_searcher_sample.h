//#pragma once
//
//#include "abstract_graph_searcher.h"
//
//
//
////! @class GraphSearcherSample
////! @date 2023/09/04
////! @author ���J��
////! @brief GraphSearcherSample�N���X�̓e�L�g�[�Ɏ��̓����I��ŕԂ��D�����܂ł��̃N���X��IGraphSearcher�̐����p�D
////! @details AbstractGraphSearcher ���p���������ꍇ�D�ȉ��̂悤�� @n
////! @n
////!	class �D���Ȗ��O final : public AbstractGraphSearcher @n
////!	{														@n
////!	}														@n
////! @n
////! �Ɛ錾���邱�ƁDfinal�͂�������ȏ�p�����Ȃ���Ƃ����Ӗ��Dpublic AbstractGraphSearcher�͂��̃N���X���p��������Ƃ����Ӗ��D@n
////! AbstractGraphSearcher���p�������N���X�ɉۂ����鐧��͂�����CAbstractGraphSearcher�̏������z�֐��CsearchGraphTree���������邱�ƁD@n
////! �ȉ��̂悤��searchGraphTree��public�ȂƂ���ɐ錾���āC����override�Ƃ���΂悢�D@n
//class GraphSearcherSample final : public AbstractGraphSearcher
//{
//public:
//	GraphSearcherSample(const std::shared_ptr<const AbstractHexapodStateCalculator>& calc) : AbstractGraphSearcher(calc) {}
//	~GraphSearcherSample() = default;
//
//	EGraphSearchResult searchGraphTree(const std::vector<SNode>& graph, const STarget& target, SNode* output_result) override;
//
//private:
//
//};
//
//
////! @file graph_searcher_sample.h
////! @date 2023/09/04
////! @author ���J��
////! @brief GraphSearcherSample�N���X�̓e�L�g�[�Ɏ��̓����I��ŕԂ��D�����܂ł��̃N���X��IGraphSearcher�̐����p�D
////! @n �s�� : @lineinfo
