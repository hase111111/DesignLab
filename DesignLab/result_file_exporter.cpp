#include "result_file_exporter.h"

#include <filesystem>
#include <fstream>

#include "designlab_timer.h"


namespace sf = std::filesystem;	//��������̂ŁCfilesystem�̖��O��Ԃ�Z�k����D


void ResultFileExporter::init()
{
	//result�t�H���_���Ȃ���΍쐬����D
	if (!sf::exists(RESULT_FOLDER_NAME))
	{
		sf::create_directory(RESULT_FOLDER_NAME);
	}

	//�t�H���_�����w�肷��D���ݎ������擾���C������t�H���_���ɂ���D
	DesignlabTimer timer;
	m_folder_name = timer.getNowTime();


	//�o�͐�t�H���_���쐬����D
	std::string output_folder_name = RESULT_FOLDER_NAME + "/" + m_folder_name;

	if (sf::exists(output_folder_name))
	{
		//���łɓ����̃t�H���_�����݂���ꍇ�́C���������s�t���O�𗧂Ă�D
		m_init_success = false;
		return;
	}

	sf::create_directory(output_folder_name);	//�t�H���_���쐬����D

	if (!sf::exists(output_folder_name))
	{
		//���x�͋t�ɁC�t�H���_���쐬�ł��Ȃ������ꍇ�́C���������s�t���O�𗧂Ă�D
		m_init_success = false;
		return;
	}

	m_init_success = true;
}


void ResultFileExporter::exportResult(const SSimulationRecord& recoder)
{
	//���������ł��Ă��Ȃ��ꍇ�́C�Ȃɂ��o�͂��Ȃ��D�܂��C�o�̓t���O��false�̏ꍇ���Ȃɂ��o�͂��Ȃ��D
	if (!m_init_success || !m_do_export) { return; }


	//�o�͐�t�@�C�����쐬����D
	std::string output_file_name = RESULT_FOLDER_NAME + "/" + m_folder_name + "/" + FILE_NAME + std::to_string(m_export_count + 1) + ".csv";

	std::ofstream ofs(output_file_name);

	//�t�@�C�����쐬�ł��Ȃ������ꍇ�́C�Ȃɂ��o�͂��Ȃ��D
	if (!ofs) { return; }


	//�m�[�h�̏����o�͂���D


	++m_export_count;	//�o�͂����񐔂��J�E���g����D
}