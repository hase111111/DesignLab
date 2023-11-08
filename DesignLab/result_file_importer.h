//! @file result_file_importer.h
//! @brief ���ʂ��t�@�C������ǂݍ��ރN���X�D


#ifndef DESIGNLAB_RESULT_FILE_IMPORTER_H_
#define DESIGNLAB_RESULT_FILE_IMPORTER_H_


#include "result_file_exporter.h"


class ResultFileImporter final
{
public:

	//! @brief �m�[�h���X�g�ƃ}�b�v�̏�Ԃ��t�@�C������ǂݍ��ށD
	//! @param [in] file_path �m�[�h���X�g�̃p�X
	//! @param [out] node_list �m�[�h���X�g
	//! @param [out] map_state �}�b�v�̏��
	//! @return bool �ǂݍ��݂������������ǂ���
	bool ImportNodeListAndMapState(
		const std::string& file_path,
		std::vector<RobotStateNode>* node_list,
		MapState* map_state
	) const;


private:

	bool ImportNodeList(const std::string& file_path, std::vector<RobotStateNode>* node_list) const;

	bool ImportMapState(const std::string& file_path, MapState* map_state) const;
};


#endif