#include "node_validity_checker.h"


void NodeValidityChecker::setNode(const RobotStateNode& node)
{
	//�m�[�h��O����ǉ�����
	m_node.push_front(node);

	//�ő�ێ����𒴂��Ă���Ȃ�΂��̕������C��납��폜����D
	while (m_node.size() > kMaxDataNum) { m_node.pop_back(); }
}


bool NodeValidityChecker::isLoopMove() const
{
	//�S�Ă̗v�f�ƁC��ԐV�����v�f���r���āC���l�̓��삪����΃��[�v���Ă���Ƃ݂Ȃ��D

	if (m_node.size() < 1) { return false; }	//��r���邽�߂̃m�[�h���Ȃ��Ȃ�Α��I���D

	auto itr = m_node.begin();		//�m�[�h�̍ŏ����w���C�e���[�^�[���擾���āC
	itr++;							//��i�߂�D

	for (size_t i = 0; i < 10; i++)
	{
		if (itr != m_node.end())itr++;
	}

	//�C�e���[�^�[���Ō�ɂȂ�܂Ń��[�v����D
	for (itr; itr != m_node.end(); itr++)
	{
		//�����m�[�h������΁C���삪���[�v���Ă���Ƃ݂Ȃ��Ctrue��Ԃ��D
		if (m_node.front() == (*itr)) { return true; }
	}

	return false;
}
