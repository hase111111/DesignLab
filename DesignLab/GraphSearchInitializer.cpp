#include "GraphSearchInitializer.h"
#include "GraphTreeCreatorSample.h"
#include "GraphSearcherSample.h"


bool GraphSearchInitializer::init(std::unique_ptr<IGraphTreeCreator>& _p_tree_creator, std::unique_ptr<IGraphSearcher>& _p_graph_searcher)
{
	_p_tree_creator = std::make_unique<GraphTreeCreatorSample>();
	_p_graph_searcher = std::make_unique<GraphSearcherSample>();

	return true;
}


//���̃N���X�̎g�����ɂ��āC
//�Q�̈����ɑ΂���std::make_uniqur<�N���X��>() ��n�����ƂŁC�N���X���쐬���Ă����邱�Ƃ��ł��܂��D
//�Ⴆ�΁CGraphTreeCreatorSample�N���X�ƁCGraphSearcherSample�N���X���쐬�������ꍇ�́C
//
//	bool GraphSearchInitializer::init(std::unique_ptr<IGraphTreeCreator>& _p_tree_creator, std::unique_ptr<IGraphSearcher>& _p_graph_searcher)
//	{
//		_p_tree_creator = std::make_unique<GraphTreeCreatorSample>();
//		_p_graph_searcher = std::make_unique<GraphSearcherSample>();
//
//		return true;
//	}
//
// ����Ȋ����ŏ����������Ă݂Ă��������D
// 
// �Ⴆ�Βn�`�̏�Ԃ𔻒f���āC����������e��ύX������C�O���t�̕]����@��ύX�������ꍇ�́C
// �܂��CsetMap() �֐������̃N���X�Ɏ������āCGraphSearch�N���X����n�`�f�[�^���擾���Ă���D
// �擾�����f�[�^�𔻒f���āC����ɂ���� make_unique����N���X��؂�ւ��Ă�����΂悢�D
//
