#include "model_loader.h"

#include <DxLib.h>


int ModelLoader::GetModelHandle(const std::string& file_path)
{
	//���łɓǂݍ��ݍς݂̃��f���̏ꍇ�́C�n���h���ԍ���Ԃ��D
	if (model_handle_map_.count(file_path) != 0)
	{
		return model_handle_map_[file_path];
	}

	//���f����ǂݍ��ށD
	int handle = MV1LoadModel(file_path.c_str());

	//�ǂݍ��݂Ɏ��s�����ꍇ��-1��Ԃ��D
	if (handle == -1)
	{
		return -1;
	}

	//�ǂݍ��݂ɐ��������ꍇ�́C�Ή��\�ɒǉ�����D
	model_handle_map_[file_path] = handle;

	return handle;
}
