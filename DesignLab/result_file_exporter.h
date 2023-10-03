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
	const static std::string kDirectoryPath;//!< �o�͐�f�B���N�g��(�t�H���_)��

	const static std::string kFileName;		//!< �t�@�C���� ( �l�Ԃ�����p )

	const static std::string kNodeListName;	//!< �m�[�h���X�g�̃t�@�C����( �v���O�����̓ǂݍ��ݗp )

	const static std::string kMapStateName;	//!< �}�b�v��Ԃ̃t�@�C����( �v���O�����̓ǂݍ��ݗp )
};


//! @class ResultFileExporter
//! @brief ���ʂ��t�@�C���ɏo�͂���N���X�D
class ResultFileExporter final
{
public:

	ResultFileExporter();

	//! @brief result�t�H���_���Ȃ���΍쐬����D�܂��C�t�H���_�����w�肷��D
	void Init();

	//! @brief �V�~�����[�V�������ʂ�ǉ�����D
	//! @param [in] simu_result �V�~�����[�V�����̌���
	void PushSimulationResult(const SimulationResultRecorder& simu_result);

	//! @brief �o�͂��s�����ǂ����̃t���O��ݒ肷��D
	//! @param [in] do_export �o�͂��s�����ǂ����̃t���O
	inline void SetDoExport(const bool do_export) { do_export_ = do_export; }


	//! @brief �ŐV�̃m�[�h���X�g���t�@�C���ɏo�͂���D
	//! @n ���������ł��Ă��Ȃ��ꍇ�́C�Ȃɂ��o�͂��Ȃ��D�܂��C�o�̓t���O��false�̏ꍇ���Ȃɂ��o�͂��Ȃ��D
	//! @n Init()���Ă΂�Ă��Ȃ��ꍇ�́C�Ȃɂ��o�͂��Ȃ��̂ŁC�K��Init()���Ăяo���Ă���Ăяo�����ƁD
	void ExportLatestNodeList() const;

	//! @brief �ŐV�̃}�b�v��Ԃ��t�@�C���ɏo�͂���D
	//! @n ���������ł��Ă��Ȃ��ꍇ�́C�Ȃɂ��o�͂��Ȃ��D�܂��C�o�̓t���O��false�̏ꍇ���Ȃɂ��o�͂��Ȃ��D
	//! @n Init()���Ă΂�Ă��Ȃ��ꍇ�́C�Ȃɂ��o�͂��Ȃ��̂ŁC�K��Init()���Ăяo���Ă���Ăяo�����ƁD
	void ExportLatestMapState() const;

	//! @brief �V�~�����[�V�������ʂ�S�ăt�@�C���ɏo�͂���D
	//! @n ���������ł��Ă��Ȃ��ꍇ�́C�Ȃɂ��o�͂��Ȃ��D�܂��C�o�̓t���O��false�̏ꍇ���Ȃɂ��o�͂��Ȃ��D
	//! @n Init()���Ă΂�Ă��Ȃ��ꍇ�́C�Ȃɂ��o�͂��Ȃ��̂ŁC�K��Init()���Ăяo���Ă���Ăяo�����ƁD
	void ExportResult() const;

private:

	//! @brief �V�~�����[�V�������ʂ��t�@�C���ɏo�͂���D
	//! @param [in] recoder �V�~�����[�V��������
	//! @param [in] simu_index �V�~�����[�V�����ԍ�
	//! @return �o�͂ɐ���������
	bool OutputResultDetail(const SimulationResultRecorder& recoder, int simu_index) const;


	std::string folder_name_;	//!< �o�͐�t�H���_��

	bool init_success_;	//!< �����������t���O

	bool do_export_;	//!< �o�͂��s�����ǂ����̃t���O


	std::vector<SimulationResultRecorder> result_list_;	//!< �V�~�����[�V�������ʂ̃��X�g
};


#endif	//DESIGNLAB_RESULT_FILE_EXPORTER_H_