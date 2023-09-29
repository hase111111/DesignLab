#include "result_file_exporter.h"

#include <filesystem>

#include "stopwatch.h"


namespace sf = std::filesystem;	//��������̂ŁCfilesystem�̖��O��Ԃ�Z�k����D


void ResultFileExporter::init()
{
	//result�t�H���_���Ȃ���΍쐬����D
	if (!sf::exists(RESULT_FOLDER_NAME))
	{
		sf::create_directory(RESULT_FOLDER_NAME);
	}

	//�t�H���_�����w�肷��D���ݎ������擾���C������t�H���_���ɂ���D
	Stopwatch timer;
	m_folder_name = timer.GetNowTimeString();


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


void ResultFileExporter::exportResult(const SimulationResultRecorder& recoder)
{
	//���������ł��Ă��Ȃ��ꍇ�́C�Ȃɂ��o�͂��Ȃ��D�܂��C�o�̓t���O��false�̏ꍇ���Ȃɂ��o�͂��Ȃ��D
	if (!m_init_success || !m_do_export) { return; }


	//�o�͐�t�@�C�����쐬����D
	std::string output_file_name = RESULT_FOLDER_NAME + "/" + m_folder_name + "/" + FILE_NAME + std::to_string(m_export_count + 1) + ".csv";

	std::ofstream ofs(output_file_name);

	//�t�@�C�����쐬�ł��Ȃ������ꍇ�́C�Ȃɂ��o�͂��Ȃ��D
	if (!ofs) { return; }


	//���ʂ��o�͂���D
	ofs << "@SimuRes" << std::endl;

	ofs << recoder;


	//���ʂ̏ڍׂ��o�͂���D
	ofs << std::endl << "@SimuResDetail" << std::endl;

	outputResultDetail(recoder, ofs);


	++m_export_count;	//�o�͂����񐔂��J�E���g�A�b�v����D


	//�t�@�C�������D
	ofs.close();
}

void ResultFileExporter::outputResultDetail(const SimulationResultRecorder& recoder, std::ofstream& stream)
{
	if (!stream) { return; }

	//���Ԃ̓��v���o�͂���D
	double max_time = recoder.graph_search_result_recoder[0].computation_time;
	double min_time = max_time;
	double sum_time = 0.0;

	for (const auto& i : recoder.graph_search_result_recoder)
	{
		if (i.computation_time > max_time) { max_time = i.computation_time; }
		if (i.computation_time < min_time) { min_time = i.computation_time; }

		sum_time += i.computation_time;
	}

	double average_time = sum_time / static_cast<double>(recoder.graph_search_result_recoder.size());

	stream << "�ő�T������," << max_time << ",[msec]" << std::endl;
	stream << "�ŏ��T������," << min_time << ",[msec]" << std::endl;
	stream << "�����T������," << sum_time << ",[msec]" << std::endl;
	stream << "���ϒT������," << average_time << ",[msec]" << std::endl;


	//�ړ������̓��v���o�͂���
	if (recoder.graph_search_result_recoder.size() > 1)
	{
		float x_move_sum = 0.0f;
		float y_move_sum = 0.0f;
		float z_move_sum = 0.0f;

		for (size_t i = 0; i != recoder.graph_search_result_recoder.size() - 1; ++i)
		{
			RobotStateNode current_node = recoder.graph_search_result_recoder[i].result_node;
			RobotStateNode next_node = recoder.graph_search_result_recoder[i + 1].result_node;
			designlab::Vector3 com_dif = next_node.global_center_of_mass - current_node.global_center_of_mass;

			x_move_sum += com_dif.x;
			y_move_sum += com_dif.y;
			z_move_sum += com_dif.z;
		}

		double x_move_average = x_move_sum / static_cast<double>(recoder.graph_search_result_recoder.size() - 1);
		double y_move_average = y_move_sum / static_cast<double>(recoder.graph_search_result_recoder.size() - 1);
		double z_move_average = z_move_sum / static_cast<double>(recoder.graph_search_result_recoder.size() - 1);

		stream << "X�������ړ�����," << x_move_sum << ",[mm]" << std::endl;
		stream << "Y�������ړ�����," << y_move_sum << ",[mm]" << std::endl;
		stream << "Z�������ړ�����," << z_move_sum << ",[mm]" << std::endl;
		stream << "X�������ψړ�����," << x_move_average << ",[mm/����]" << std::endl;
		stream << "Y�������ψړ�����," << y_move_average << ",[mm/����]" << std::endl;
		stream << "Z�������ψړ�����," << z_move_average << ",[mm/����]" << std::endl;
	}

}
