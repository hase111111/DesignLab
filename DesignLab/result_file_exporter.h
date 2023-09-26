#pragma once

#include <string>
#include <vector>
#include <fstream>

#include "node.h"
#include "simulation_result_recorder.h"
#include "graph_search_result.h"


//! @class ResultFileExporter
//! @date 2023/08/24
//! @brief ���ʂ��t�@�C���ɏo�͂���N���X�D
class ResultFileExporter final
{
public:

	ResultFileExporter() = default;


	//! @brief result�t�H���_���Ȃ���΍쐬����D�܂��C�t�H���_�����w�肷��D
	void init();


	//! @brief �V�~�����[�V�������ʂ��t�@�C���ɏo�͂���D
	//! @n ���������ł��Ă��Ȃ��ꍇ�́C�Ȃɂ��o�͂��Ȃ��D�܂��C�o�̓t���O��false�̏ꍇ���Ȃɂ��o�͂��Ȃ��D
	//! @n �o�͂����t�@�C���� sim_result_��.csv �Ƃ������O�ɂȂ�D
	//! @param [in] recoder �V�~�����[�V�����̌���
	void exportResult(const SimulationResultRecorder& recoder);


	//! @brief �o�͂��s�����ǂ����̃t���O��ݒ肷��D
	//! @param [in] do_export �o�͂��s�����ǂ����̃t���O
	void setDoExport(const bool do_export) { m_do_export = do_export; }


private:

	void outputResultDetail(const SimulationResultRecorder& recoder, std::ofstream& stream);


	const std::string RESULT_FOLDER_NAME = "result";	//result�t�H���_��

	const std::string FILE_NAME = "sim_result_";		//�t�@�C����


	std::string m_folder_name;		//�t�H���_��

	bool m_init_success = false;	//�����������t���O

	bool m_do_export = true;		//�o�͂��s�����ǂ����̃t���O

	int m_export_count = 0;			//�o�͉�
};


//! @file result_file_exporter.h
//! @date 2023/08/24
//! @author ���J��
//! @brief ���ʂ��t�@�C���ɏo�͂���N���X�D
//! @n �s�� : @lineinfo
