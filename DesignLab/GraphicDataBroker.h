#pragma once
#include <boost/thread.hpp>
#include "MapState.h"
#include "Node.h"


class GraphicDataBroker final
{
public:

	GraphicDataBroker() = default;
	~GraphicDataBroker() = default;

	//! �}�b�v�̏�Ԃ𒇉�l�ɓn���D
	//! @param [in] _map �}�b�v���Q�Ɠn������
	void setMapState(const MapState& _map);	

	//! �}�b�v�̏�Ԃ�Ԃ�
	//! @return MapState �}�b�v�̏�Ԃ�l�n������D<br>��{�I�ɂ͑傫�ȃN���X�͒l�n������ׂ��łȂ����C�}�b�v�͉��x���X�V����Ȃ��f�[�^�����l�n������D
	MapState getMapState() const;


	//! ���{�b�g�̐V������Ԃ������m�[�h�𒇉�l�ɓn���D�m�[�h�͓��I�z��ŊǗ�����Ă���C�V�����̂����ɒǉ�(push)����D
	//! @param [in] _node ���{�b�g�̐V�������
	void pushNode(const SNode &_node);				

	//! �m�[�h�̏W����vector��p���ĎQ�Ɠn������D�S�f�[�^���R�s�[����D
	//! @param [out] _node_vec ���̊֐��̒��ň�x���g����ɂ��Ă���C�f�[�^��������D
	void copyAllNode(std::vector<SNode>& _node_vec) const;

	//! �m�[�h�̏W����vector��p���ĎQ�Ɠn������D�n����vector���C�����Ă���f�[�^�������ꍇ�̂ݍ������R�s�[����D<br> copyAllNode���y�������ɂȂ�܂��D
	//! @param [out] _node_vec �����ȊO�ɕύX�͂���Ȃ��D�����̂�push�����
	void copyOnlyNewNode(std::vector<SNode>& _node_vec) const;

private:
	
	mutable boost::shared_mutex m_mtx;

	MapState m_Map;

	std::vector<SNode> m_node;
};


//! @file GraphicDataBroker.h
//! @brief GraphicDataBroker�N���X�̎������s���Ă���D
//! @author ���J��

//! @class GraphicDataBroker
//! @brief �摜�\�����ƁC�f�[�^�����������т��钇��l�N���X
//! @details Broker:�u���[�J�[�C����l�̂��ƁD<br> �f�[�^������(�O���t�T��)���X�V�����f�[�^�����̃N���X�ɓn���C�摜�\���������̃N���X����X�V���ꂽ�f�[�^�������Ă����C�`�悵�܂��D<br>
//! �������Ă��邩������Ȃ��ꍇ�C�uPub / Sub�p�^�[���v�Œ��ׂĂ݂Ă��������D<br> 
//! <br> 
//! [�񓯊������ɂ���] <br>
//! �񓯊�����(�����ɏ������s������)���s���ۂɁC��̂Ƀf�[�^�ɓ����^�C�~���O�ŃA�N�Z�X����Ɗ댯�ł�(����`�����ɂȂ�C���������s�����s��ɂȂ�)�D<br>
//! ���̃N���X�͂����h�����߂�boost::shared_mutex���g�p���Ă��܂��D<br>
//! �ڂ����� https://www.mathkuro.com/c-cpp/boost/how-to-use-boost-thread/#toc10 ��5�͂��݂Ă��������D<br>
//! ���̃N���X���ł�read lock, write lock���g���Ă��܂��D<br> 
//! �Q�l https://iorate.hatenablog.com/entry/20130222/1361538198 <br> 
//! <br>
//! �����o��m_mtx�ɂ��Ă���mutable �� const�ȃ����o�֐�(�����o�̒l��ύX�ł��Ȃ������o�֐�)�ɂ����Ă��ύX�ł���悤�ɂȂ郁���o�ϐ���\���܂��D<br>
//! �ʏ��Ύg���ׂ��ł͂Ȃ��ł����C����̂悤�ȏꍇ(boost::shared_mutex���g���ꍇ)�͗L���I�ł��D
//! @author ���J��

