//! @file result_file_exporter.h
//! @brief ���ʂ��t�@�C���ɏo�͂���N���X�D


#ifndef DESIGNLAB_RESULT_FILE_EXPORTER_H_
#define DESIGNLAB_RESULT_FILE_EXPORTER_H_


#include <string>
#include <vector>
#include <fstream>

#include "robot_state_node.h"
#include "simulation_result_recorder.h"


class ResultFileConst final
{
public:
	const static std::string kDirectoryName;	//!< �o�͐�f�B���N�g��(�t�H���_)��

	const static std::string kFileName;			//!< �t�@�C���� ( �l�Ԃ�����p )

	const static std::string kNodeListName;	//!< �m�[�h���X�g�̃t�@�C����( �v���O�����̓ǂݍ��ݗp )
};


//! @class ResultFileExporter
//! @brief ���ʂ��t�@�C���ɏo�͂���N���X�D
class ResultFileExporter final
{
public:

	ResultFileExporter();


	//! @brief result�t�H���_���Ȃ���΍쐬����D�܂��C�t�H���_�����w�肷��D
	void Init();

	//! @brief �V�~�����[�V�������ʂ�ݒ肷��D�����āC�m�[�h���X�g���t�@�C���ɏo�͂���D
	//! @n ���������ł��Ă��Ȃ��ꍇ�́C�Ȃɂ����Ȃ��D�܂��C�o�̓t���O��false�̏ꍇ���Ȃɂ����Ȃ��D
	//! @param [in] simu_result �V�~�����[�V�����̌���
	void SetSimulationResultAndExportNodeList(const SimulationResultRecorder& simu_result);

	//! @brief �V�~�����[�V�������ʂ��t�@�C���ɏo�͂���D
	//! @n ���������ł��Ă��Ȃ��ꍇ�́C�Ȃɂ��o�͂��Ȃ��D�܂��C�o�̓t���O��false�̏ꍇ���Ȃɂ��o�͂��Ȃ��D
	//! @n �o�͂����t�@�C���� sim_result_��.csv �Ƃ������O�ɂȂ�D
	//! @param [in] recoder �V�~�����[�V�����̌���
	void ExportResult(const SimulationResultRecorder& recoder);

	//! @brief �o�͂��s�����ǂ����̃t���O��ݒ肷��D
	//! @param [in] do_export �o�͂��s�����ǂ����̃t���O
	inline void SetDoExport(const bool do_export) { do_export_ = do_export; }


private:

	void OutputResultDetail(const SimulationResultRecorder& recoder, std::ofstream& stream);


	std::string folder_name_;	//!< �o�͐�t�H���_��

	bool init_success_;	//!< �����������t���O

	bool do_export_;	//!< �o�͂��s�����ǂ����̃t���O

	int export_count_;	//!< �o�͉�


	std::vector<SimulationResultRecorder> result_list_;	//!< �V�~�����[�V�������ʂ̃��X�g
};


#endif	//DESIGNLAB_RESULT_FILE_EXPORTER_H_