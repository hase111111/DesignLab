#pragma once

#include <boost/thread.hpp>

#include "map_state.h"
#include "Node.h"


//! @class GraphicDataBroker
//! @date 2023/08/08
//! @author ���J��
//! @brief �摜�\�����ƁC�f�[�^�����������т��钇��l�N���X
//! @details Broker:�u���[�J�[�C����l�̂��ƁD@n �f�[�^������(�O���t�T��)���X�V�����f�[�^�����̃N���X�ɓn���C�摜�\���������̃N���X����X�V���ꂽ�f�[�^�������Ă����C�`�悷��D@n
//! @n �������Ă��邩������Ȃ��ꍇ�C�uPub / Sub�p�^�[���v�Œ��ׂĂق����D@n 
//! @n [�񓯊������ɂ���] 
//! @n �񓯊����� (����E�����ɏ������s������) ���s���ۂɁC��̂Ƀf�[�^�ɓ����^�C�~���O�ŃA�N�Z�X����Ɗ댯(����`�����ɂȂ�C���������s�����s��ɂȂ�)�D
//! @n ���̃N���X�͂����h�����߂�boost::shared_mutex���g�p���Ă���D
//! @n �ڂ����� https://www.mathkuro.com/c-cpp/boost/how-to-use-boost-thread/#toc10 ��5�͂��Q�Ƃ��Ăق����D
//! @n ���̃N���X���ł�read lock, write lock���g���Ă���D 
//! @n �Q�l https://iorate.hatenablog.com/entry/20130222/1361538198 @n 
//! @n �����o��m_mtx�ɂ��Ă���mutable �� const�ȃ����o�֐�(�����o�̒l��ύX�ł��Ȃ������o�֐�)�ɂ����Ă��ύX�ł���悤�ɂȂ郁���o�ϐ���\���܂��D
//! @n �ʏ��Ύg���ׂ��ł͂Ȃ��ł����C����̂悤�ȏꍇ(boost::shared_mutex���g���ꍇ)�͗L���I�ł��D
class GraphicDataBroker final
{
public:

	GraphicDataBroker() = default;
	~GraphicDataBroker() = default;

	//! �}�b�v�̏�Ԃ𒇉�l�ɓn���D
	//! @param [in] map �}�b�v���Q�Ɠn������
	void setMapState(const MapState& map);

	//! �}�b�v�̏�Ԃ�Ԃ�
	//! @return MapState �}�b�v�̏�Ԃ�l�n������D@n ��{�I�ɂ͑傫�ȃN���X�͒l�n������ׂ��łȂ����C�}�b�v�͉��x���X�V����Ȃ��f�[�^�����l�n������D
	MapState getMapState() const;


	//! @brief ���{�b�g�̐V������Ԃ������m�[�h�𒇉�l�ɓn���D�m�[�h�͓��I�z��ŊǗ�����Ă���C�V�����̂����ɒǉ�(push)����D
	//! @param [in] node ���{�b�g�̐V�������
	void pushNode(const SNode& node);

	//! @brief �m�[�h�̏W����S�č폜����D
	void deleteAllNode();

	//! @brief �m�[�h�̏W����vector��p���ĎQ�Ɠn������D�S�f�[�^���R�s�[����D
	//! @param [out] node_vec ���̊֐��̒��ň�x���g����ɂ��Ă���C�f�[�^��������D
	void copyAllNode(std::vector<SNode>* node_vec) const;

	//! @brief �m�[�h�̏W����vector��p���ĎQ�Ɠn������D�n����vector���C�����Ă���f�[�^�������ꍇ�̂ݍ������R�s�[����D@n copyAllNode���y�������ɂȂ�D
	//! @param [out] node_vec �����ȊO�ɕύX�͂���Ȃ��D�����̂�push�����
	void copyOnlyNewNode(std::vector<SNode>* node_vec) const;

	//! @brief �Z�b�g����Ă���m�[�h�̐���Ԃ��D
	//! @return size_t �m�[�h�̐�
	size_t getNodeNum() const;

private:

	mutable boost::shared_mutex m_mtx;

	MapState m_Map;

	std::vector<SNode> m_node;
};


//! @file graphic_data_broker.h
//! @date 2023/08/08
//! @author ���J��
//! @brief �O���t�T���̌��ʂ�ʃX���b�h�̃O���t�B�b�N�N���X�ɘA������GraphicDataBroker�N���X�D
//! @n �s�� : @lineinfo
