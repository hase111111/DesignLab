//! @file interface_map_creator.h
//! @brief �}�b�v�����N���X�̃C���^�[�t�F�[�X


#ifndef DESIGNLAB_INTERFACE_MAP_CREATOR_H_
#define DESIGNLAB_INTERFACE_MAP_CREATOR_H_

#include "map_state.h"


//! @class IMapCreator
//! @brief �}�b�v�����N���X�̃C���^�[�t�F�[�X
class IMapCreator 
{	
public:

	virtual ~IMapCreator() = default;

	//! @brief �}�b�v�̏��������s���D
	//! @details �V�~�����[�V�����ɂ����Ă͂��̋@�\�݂̂���΂悢���C
	//! @n ���@�𓮍삳����ꍇ�C�J���������m�ɔF���ł��鋗���̊֌W�ŁC�}�b�v��ǂݒ����K�v������D
	//! @n ���̂��߁C���@�������͂�����̊֐��ŏ�����������CUpdateMap()�Ń}�b�v���X�V����K�v������D
	//! @return �����������}�b�v
	virtual MapState InitMap() = 0;

	//! @brief �}�b�v�̍X�V���s���D
	//! @details ���@�𓮍삳����ꍇ�ɁC�}�b�v��ǂݒ����K�v������D
	//! @n �V�~�����[�V�����ł͂��̋@�\�͕s�v�D
	//! @param [in,out] current_map ���݂̃}�b�v���󂯎��C�X�V�����}�b�v��Ԃ��D
	virtual void UpdateMap(MapState* current_map) = 0;

};


#endif	// DESIGNLAB_INTERFACE_MAP_CREATOR_H_