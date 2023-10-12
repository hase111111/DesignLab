//! @file model_loader.h
//! @brief Dxlib��3D���f����ǂݍ��ރN���X

#ifndef DESIGNLAB_MODEL_LOADER_H_
#define DESIGNLAB_MODEL_LOADER_H_

#include <string>
#include <map>

#include "singleton.h"


//! @class ModelLoader
//! @brief Dxlib��3D���f����ǂݍ��ރN���X�D�Ăяo���Ƃ���ModelLoader::GetIns()->LoadModel(file_path)�Ƃ���D
class ModelLoader final : public Singleton<ModelLoader>
{
public:
	
	//! @brief ���f����ǂݍ��ށD���łɓǂݍ��݂��݂̃��f����ǂݍ��񂾏ꍇ�́C�n���h���ԍ���Ԃ��D
	//! @param [in] file_path ���f���̃t�@�C���p�X
	//! @return int ���f���̃n���h���ԍ��D���s������-1��Ԃ��D�����l�͐��̒l�D
	int LoadModel(const std::string file_path);

private:

	friend Singleton<ModelLoader>;
	ModelLoader() = default;
	~ModelLoader() = default;
	ModelLoader(const ModelLoader& r) = default;
	ModelLoader& operator=(const ModelLoader& r) = default;

	std::map<std::string , int> model_handle_map_;	//!< �ǂݍ��ݍς݂̃��f���̃t�@�C���p�X�ƃn���h���ԍ��̑Ή��\
};


#endif