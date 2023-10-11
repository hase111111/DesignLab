#include "node_validity_checker.h"


void NodeValidityChecker::SetNode(const RobotStateNode& node)
{
	//�m�[�h��O����ǉ�����
	node_.push_front(node);

	//�ő�ێ����𒴂��Ă���Ȃ�΂��̕������C��납��폜����D
	while (node_.size() > kMaxDataNum) { node_.pop_back(); }
}


bool NodeValidityChecker::IsLoopMove() const
{
	//�S�Ă̗v�f�ƁC��ԐV�����v�f���r���āC���l�̓��삪����΃��[�v���Ă���Ƃ݂Ȃ��D

	if (node_.size() < 1) { return false; }	//��r���邽�߂̃m�[�h���Ȃ��Ȃ�Α��I���D

	auto itr = node_.begin();		//�m�[�h�̍ŏ����w���C�e���[�^�[���擾���āC
	itr++;							//��i�߂�D

	for (size_t i = 0; i < 10; i++)
	{
		if (itr != node_.end())itr++;
	}

	//�C�e���[�^�[���Ō�ɂȂ�܂Ń��[�v����D
	for (itr; itr != node_.end(); itr++)
	{
		//�����m�[�h������΁C���삪���[�v���Ă���Ƃ݂Ȃ��Ctrue��Ԃ��D
		if (node_.front() == (*itr)) { return true; }
	}

	return false;
}
