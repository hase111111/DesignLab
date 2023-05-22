#pragma once
#include "boost/thread.hpp"
#include "MapState.h"

//�摜�\�����ƁC�f�[�^�����������т��钇��l(Broker:�u���[�J�[)�����̃N���X�ł��D
//�f�[�^���������X�V���ꂽ�f�[�^�����̃N���X�ɓn���C
//�摜�\���������̃N���X����X�V���ꂽ�f�[�^�������Ă����܂��D
//�������Ă��邩������Ȃ��ꍇ�C�uPub / Sub�p�^�[���v�Œ��ׂĂ݂Ă��������D

class GraphicDataBroker final
{
public:
	GraphicDataBroker() = default;
	~GraphicDataBroker() = default;

	void setMapState(const MapState& _map);	//�}�b�v�̏�Ԃ��Z�b�g����
	MapState getMapState() const;			//�}�b�v�̏�Ԃ�Ԃ�

	void pushNode(const SNode _node);							//�V������Ԃ������m�[�h��n���D
	void copyNode(std::vector<SNode>& _node_vec) const;			//�m�[�h�̏W����vector��p���ĎQ�Ɠn������D�S�f�[�^���R�s�[����D
	void copyOnlyNewNode(std::vector<SNode>& _node_vec) const;	//�m�[�h�̏W����vector��p���ĎQ�Ɠn������D�n����vector���C�����Ă���f�[�^�������ꍇ�̂ݍ������R�s�[����D

private:
	
	//�񓯊�����(�����ɏ������s������)���s���ۂɁC��Ƀf�[�^�ɓ����^�C�~���O�ŃA�N�Z�X����Ɗ댯�ł�(����`�����ɂȂ�C���������s�����s��ɂȂ�)�D������g���Ƃ����h���܂��D
	//�ڂ����� https://www.mathkuro.com/c-cpp/boost/how-to-use-boost-thread/#toc10 ��5�͂��݂Ă��������D
	// mutable �� const�Ȋ֐�(�����o�̒l��ύX�ł��Ȃ��֐�)�ɂ����Ă��ύX�ł���悤�ɂȂ郁���o�ϐ���\���܂��D�ʏ��Ύg���ׂ��ł͂Ȃ��ł����C����̂悤�ȏꍇ�͗L���I�ł��D
	mutable boost::shared_mutex m_mtx;

	MapState m_Map;

	std::vector<SNode> m_node;
};