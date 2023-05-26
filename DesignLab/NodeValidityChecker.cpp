#include "NodeValidityChecker.h"

void NodeValidityChecker::setNode(const SNode& _node)
{
	//�m�[�h��O����ǉ�����
	m_node.push_front(_node);

	//�ő�ێ����𒴂��Ă���Ȃ�΂��̕������C��납��폜����D
	while (m_node.size() > MAX_DATA_NUM) { m_node.pop_back(); }
}

bool NodeValidityChecker::isLoopMove() const
{
	//�S�Ă̗v�f�ƁC��ԐV�����v�f���r���āC���l�̓��삪����΃��[�v���Ă���Ƃ݂Ȃ��D

	if (m_node.size() < 1) { return false; }	//��r���邽�߂̃m�[�h���Ȃ��Ȃ�Α��I���D

	auto _itr = m_node.begin();		//�m�[�h�̍ŏ����w���C�e���[�^�[���擾���āC
	_itr++;							//��i�߂�D

	//�C�e���[�^�[���Ō�ɂȂ�܂Ń��[�v����D
	for (_itr; _itr != m_node.end(); _itr++)
	{
		//�����m�[�h������΁C���삪���[�v���Ă���Ƃ݂Ȃ��Ctrue��Ԃ��D
		if (isNodeEqual(m_node.front(), (*_itr)) == true) { return true; }
	}

	return false;
}
