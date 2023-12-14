//! @file model_loader.h
//! @brief Dxlib��3D���f����ǂݍ��ރN���X�D

#ifndef DESIGNLAB_MODEL_LOADER_H_
#define DESIGNLAB_MODEL_LOADER_H_

#include <string>
#include <map>

#include "singleton.h"


//! @class ModelLoader
//! @brief Dxlib��3D���f����ǂݍ��ރN���X�D
//! @details �V���O���g���ł���CModelLoader::GetIns()�ŃC���X�^���X���擾����D
//! @n �Ăяo���Ƃ���ModelLoader::GetIns()->GetModelHandle(file_path)�Ƃ���D
//! @code
//! // ��
//! 
//! // ���f���̃t�@�C���p�X
//! std::string file_path = "data/model/robot/robot.x";
//! 
//! // ���f���̃n���h���ԍ�
//! int model_handle = ModelLoader::GetIns()->GetModelHandle(file_path);
//! @endcode
//! @see Singleton
//! @attention ���̃N���X���g�p����Ƃ��́C�K��DxLib_Init()�̌�Ɏg�p���邱�ƁD
//! �܂��CDxLib_End()�̑O�Ɏg�p���I�����邱�ƁD
//! ���������Ȃ��ƁC�A�v�����N���b�V������D
class ModelLoader final : public Singleton<ModelLoader>
{
public:
	
	//! @brief Dxlib��3D���f����`�悷��ۂɁC���f���̃n���h�����w�肷��D
	//! @n ���f�����܂��ǂݍ��܂�Ă��Ȃ��ꍇ�́C���f����ǂݍ���ł���C�n���h���ԍ���Ԃ��D
	//! @n ���łɓǂݍ��݂��݂̃��f����ǂݍ��񂾏ꍇ�́C�n���h���ԍ���Ԃ��D
	//! @param [in] file_path ���f���̃t�@�C���p�X.
	//! @return int ���f���̃n���h���ԍ��D���s������-1��Ԃ��D�����l�͐��̒l�D
	[[nodiscard]] int GetModelHandle(const std::string& file_path);

private:

	friend Singleton<ModelLoader>;
	ModelLoader() = default;
	~ModelLoader() = default;
	ModelLoader(const ModelLoader& r) = default;
	ModelLoader& operator=(const ModelLoader& r) = default;

	std::map<std::string, int> model_handle_map_;	//!< �ǂݍ��ݍς݂̃��f���̃t�@�C���p�X�ƃn���h���ԍ��̑Ή��\
};


#endif	// DESIGNLAB_MODEL_LOADER_H_