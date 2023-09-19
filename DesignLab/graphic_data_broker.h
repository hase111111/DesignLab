//! @file graphic_data_broker.h
//! @brief �O���t�T���̌��ʂ�ʃX���b�h�̃O���t�B�b�N�N���X�ɘA������N���X�D

#ifndef DESIGNLAB_GRAPHIC_DATA_BROKER_H_
#define DESIGNLAB_GRAPHIC_DATA_BROKER_H_


#include <boost/thread.hpp>

#include "map_state.h"
#include "Node.h"


//! @class GraphicDataBroker
//! @brief �摜�\�����ƁC�f�[�^�����������т��钇��l�N���X
//! @details Broker:�u���[�J�[�C����l�̂��ƁD
//! @n �f�[�^������(�O���t�T��)���X�V�����f�[�^�����̃N���X�ɓn���C�摜�\���������̃N���X����X�V���ꂽ�f�[�^�������Ă����C�`�悷��D
//! @n
//! @n �O���t�T���@���@GraphicDataBroker�@���@�摜�\����
//! @n 
//! @n [�񓯊������ɂ���] 
//! @n �񓯊����� (����E�����ɏ������s������) ���s���ۂɁC��̂Ƀf�[�^�ɓ����^�C�~���O�ő��삷��Ɗ댯(����`�����ɂȂ�C���������s�����s��ɂȂ�)�D
//! @n ���̃N���X�͂����h�����߂�boost::shared_mutex���g�p���Ă���D
//! @n �ڂ����� https://www.mathkuro.com/c-cpp/boost/how-to-use-boost-thread/#toc10 ��5�͂��Q�Ƃ��Ăق����D
//! @n ���̃N���X���ł�read lock, write lock���g���Ă���D 
//! @n �Q�l https://iorate.hatenablog.com/entry/20130222/1361538198 
//! @n 
//! @n �����o��m_mtx�ɂ��Ă���mutable �� const�ȃ����o�֐�(�����o�̒l��ύX�ł��Ȃ������o�֐�)�ɂ����Ă��ύX�ł���悤�ɂȂ郁���o�ϐ���\���D
//! @n �ʏ��Ύg���ׂ��ł͂Ȃ����C����̂悤�ȏꍇ(boost::shared_mutex���g���ꍇ)�͗L���I�D

class GraphicDataBroker final
{
public:

	GraphicDataBroker() : update_count_(0) {};

	//! @brief �f�[�^�̍X�V�񐔂�Ԃ��D
	//! @return int �f�[�^�̍X�V��
	int update_count() const;

	//! @brief �}�b�v�̏�Ԃ𒇉�l�ɓn���D
	//! @param [in] map �}�b�v���Q�Ɠn������
	void set_map_state(const MapState_Old& map);

	//! @brief �}�b�v�̏�Ԃ�Ԃ�
	//! @return MapState_Old �}�b�v�̏�Ԃ�l�n������D
	//! @n ��{�I�ɂ͑傫�ȃN���X�͒l�n������ׂ��łȂ����C�}�b�v�͉��x���X�V����Ȃ��f�[�^�����l�n������D
	const MapState_Old& map_state() const;

	//! @brief ���{�b�g�̐V������Ԃ������m�[�h�𒇉�l�ɓn���D�m�[�h�͓��I�z��ŊǗ�����Ă���C�V�����̂����ɒǉ�(push)����D
	//! @param [in] node ���{�b�g�̐V�������
	void PushNode(const SNode& node);

	//! @brief �m�[�h�̏W����S�č폜����D
	void DeleteAllNode();

	//! @brief �m�[�h�̏W����vector��p���ĎQ�Ɠn������D�S�f�[�^���R�s�[����D
	//! @param [out] node_vec ���̊֐��̒��ň�x���g����ɂ��Ă���C�f�[�^��������D
	void CopyAllNode(std::vector<SNode>* node_vec) const;

	//! @brief �m�[�h�̏W����vector��p���ĎQ�Ɠn������D�n����vector���C�����Ă���f�[�^�������ꍇ�̂ݍ������R�s�[����D@n copyAllNode���y�������ɂȂ�D
	//! @param [out] node_vec �����ȊO�ɕύX�͂���Ȃ��D�����̂�push�����
	void CopyOnlyNewNode(std::vector<SNode>* node_vec) const;

	//! @brief �Z�b�g����Ă���m�[�h�̐���Ԃ��D
	//! @return size_t �m�[�h�̐�
	size_t GetNodeNum() const;

	//! @brief �V�~�����[�V�������I���������Ƃ𒇉�l�ɓ`����D
	void SetSimuEnd();

	//! @brief �V�~�����[�V�������I������m�[�h�̃C���f�b�N�X���擾����
	//! @param [in] simu_num �V�~�����[�V�����ԍ�( 0 , 1, 2, ...)
	//! @return size_t �V�~�����[�V�������I������m�[�h�̃C���f�b�N�X�C�V�~�����[�V�������I�����Ă��Ȃ��ꍇ�͍Ō�̃m�[�h�̃C���f�b�N�X��Ԃ��D
	//! @n �V�~�����[�V�������J�n����Ă��Ȃ��ꍇ��-1��Ԃ��D
	size_t GetSimuEndIndex(const int simu_num);

	//! @brief �V�~�����[�V�����̏I���������m�[�h�̃C���f�b�N�X��vector��p���ĎQ�Ɠn������D
	//! @param [out] simu_end_index ���̊֐��̒��ň�x���g����ɂ��Ă���C�f�[�^��������D
	void CopySimuEndIndex(std::vector<size_t>* simu_end_index) const;

private:

	mutable boost::shared_mutex mtx_;		//!< ���b�N�p��mutex

	int update_count_;						//!< �f�[�^�̍X�V��

	MapState_Old map_state_;				//!< �}�b�v�̏��

	std::vector<SNode> graph_;				//!< ���{�b�g�̏�Ԃ������m�[�h�̏W��

	std::vector<size_t> simu_end_index_;	//!< �V�~�����[�V�������I������m�[�h�̃C���f�b�N�X���i�[����D
};


#endif // !DESIGNLAB_GRAPHIC_DATA_BROKER_H_