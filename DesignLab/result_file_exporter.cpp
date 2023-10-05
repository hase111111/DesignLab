#include "result_file_exporter.h"

#include <filesystem>

#include <magic_enum.hpp>

#include "cmdio_util.h"
#include "designlab_math_util.h"
#include "stopwatch.h"


namespace dlio = designlab::cmdio;
namespace dlm = designlab::math_util;
namespace sf = std::filesystem;	//��������̂ŁCfilesystem�̖��O��Ԃ�Z�k����D


const std::string ResultFileConst::kDirectoryPath = sf::current_path().string() + "/result";

const std::string ResultFileConst::kFileName = "sim_result";

const std::string ResultFileConst::kNodeListName = "node_list";

const std::string ResultFileConst::kMapStateName = "map_state";


ResultFileExporter::ResultFileExporter() :
	init_success_(false),
	do_export_(true)
{
}

void ResultFileExporter::Init()
{
	//result�t�H���_���Ȃ���΍쐬����D
	if (not sf::exists(ResultFileConst::kDirectoryPath))
	{
		dlio::Output("���ʏo�͐�t�H���_ " + ResultFileConst::kDirectoryPath + "�����݂��Ȃ��̂ō쐬���܂��D", OutputDetail::kInfo);
		sf::create_directory(ResultFileConst::kDirectoryPath);
	}

	//�t�H���_�����w�肷��D���ݎ������擾���C������t�H���_���ɂ���D
	Stopwatch timer;
	folder_name_ = timer.GetNowTimeString();


	//�o�͐�t�H���_���쐬����D
	std::string output_folder_name = ResultFileConst::kDirectoryPath + "/" + folder_name_;

	if (sf::exists(output_folder_name))
	{
		//���łɓ����̃t�H���_�����݂���ꍇ�́C���������s�t���O�𗧂Ă�D
		init_success_ = false;

		dlio::Output("���ʏo�͐�̃t�H���_ " + output_folder_name + "�͂��łɑ��݂��܂��D", OutputDetail::kError);

		return;
	}

	sf::create_directory(output_folder_name);	//�t�H���_���쐬����D

	if (not sf::exists(output_folder_name))
	{
		//���x�͋t�ɁC�t�H���_���쐬�ł��Ȃ������ꍇ�́C���������s�t���O�𗧂Ă�D
		init_success_ = false;

		dlio::Output("���ʏo�͐�̃t�H���_ " + output_folder_name + "���쐬�ł��܂���ł����D", OutputDetail::kError);

		return;
	}

	init_success_ = true;
}

void ResultFileExporter::PushSimulationResult(const SimulationResultRecorder& simu_result)
{
	//���ʂ��Z�b�g����
	result_list_.push_back(simu_result);
}


void ResultFileExporter::ExportLatestNodeList() const 
{
	//���������ł��Ă��Ȃ��ꍇ�́C�Ȃɂ��o�͂��Ȃ��D�܂��C�o�̓t���O��false�̏ꍇ���Ȃɂ��o�͂��Ȃ��D
	if (not init_success_)
	{
		dlio::Output("���ʏo�͐�̃t�H���_�̏������Ɏ��s���Ă��邽�߁CNodeList���o�͂ł��܂���", OutputDetail::kError);
		return;
	}

	if (not do_export_)
	{
		dlio::Output("���ʏo�̓t���O��false�̂��߁CNodeList���o�͂��܂���", OutputDetail::kInfo);
		return;
	}

	dlio::Output("NodeList���o�͂��܂��D", OutputDetail::kInfo);

	//�o�͐�t�@�C�����쐬����D
	std::string output_file_name = ResultFileConst::kDirectoryPath + "/" + folder_name_ + "/" + ResultFileConst::kNodeListName + std::to_string(result_list_.size()) + ".csv";

	std::ofstream ofs(output_file_name);

	//�t�@�C�����쐬�ł��Ȃ������ꍇ�́C�Ȃɂ��o�͂��Ȃ��D
	if (not ofs)
	{
		dlio::Output("�t�@�C�� " + output_file_name + "���쐬�ł��܂���ł����D", OutputDetail::kError);
		return;
	}

	for (const auto& i : result_list_.back().graph_search_result_recoder)
	{
		ofs << i.result_node << "\n";	//�m�[�h���o�͂���D
	}

	ofs.close();	//�t�@�C�������D

	dlio::Output("�o�͊��� : " + output_file_name, OutputDetail::kInfo);
}


void ResultFileExporter::ExportLatestMapState() const 
{
	//���������ł��Ă��Ȃ��ꍇ�́C�Ȃɂ��o�͂��Ȃ��D�܂��C�o�̓t���O��false�̏ꍇ���Ȃɂ��o�͂��Ȃ��D
	if (not init_success_)
	{
		dlio::Output("���ʏo�͐�̃t�H���_�̏������Ɏ��s���Ă��邽�߁CMapState���o�͂ł��܂���", OutputDetail::kError);
		return;
	}

	if (not do_export_)
	{
		dlio::Output("���ʏo�̓t���O��false�̂��߁CMapState���o�͂��܂���", OutputDetail::kInfo);
		return;
	}

	dlio::Output("MapState���o�͂��܂��D", OutputDetail::kInfo);

	//�o�͐�t�@�C�����쐬����D
	std::string output_file_name = ResultFileConst::kDirectoryPath + "/" + folder_name_ + "/" + ResultFileConst::kMapStateName + std::to_string(result_list_.size()) + ".csv";

	std::ofstream ofs(output_file_name);

	//�t�@�C�����쐬�ł��Ȃ������ꍇ�́C�Ȃɂ��o�͂��Ȃ��D
	if (not ofs)
	{
		dlio::Output("�t�@�C�� " + output_file_name + " ���쐬�ł��܂���ł����D", OutputDetail::kError);
		return;
	}

	ofs << result_list_.back().map_state;	//�}�b�v��Ԃ��o�͂���D

	ofs.close();	//�t�@�C�������D

	dlio::Output("�o�͊��� : " + output_file_name, OutputDetail::kInfo);
}


void ResultFileExporter::ExportResult() const
{
	//���������ł��Ă��Ȃ��ꍇ�́C�Ȃɂ��o�͂��Ȃ��D�܂��C�o�̓t���O��false�̏ꍇ���Ȃɂ��o�͂��Ȃ��D
	if (not init_success_) 
	{
		dlio::Output("���ʏo�͐�̃t�H���_�̏������Ɏ��s���Ă��邽�߁C���ʂ��o�͂ł��܂���", OutputDetail::kError);
		return; 
	}

	if (not do_export_) 
	{
		dlio::Output("���ʏo�̓t���O��false�̂��߁C���ʂ��o�͂��܂���", OutputDetail::kInfo);
		return; 
	}

	dlio::Output("���ʂ��o�͂��܂��D�V�~�����[�V������ : " + std::to_string(result_list_.size()), OutputDetail::kInfo);

	for (int i = 0; i < result_list_.size(); i++)
	{
		if (OutputResultDetail(result_list_[i], i)) 
		{
			dlio::Output("�o�͊��� : �V�~�����[�V�����ԍ� " + std::to_string(i + 1), OutputDetail::kInfo);
		}
		else 
		{
			dlio::Output("�o�͎��s : �V�~�����[�V�����ԍ� " + std::to_string(i + 1), OutputDetail::kInfo);
		}
	}
}

bool ResultFileExporter::OutputResultDetail(const SimulationResultRecorder& recoder, const int index) const
{
	//�o�͐�t�@�C�����쐬����D
	std::string output_file_name = 
		ResultFileConst::kDirectoryPath + "/" + folder_name_ + "/" + ResultFileConst::kFileName + std::to_string(index + 1) + ".csv";

	std::ofstream ofs(output_file_name);

	//�t�@�C�����쐬�ł��Ȃ������ꍇ�́C�Ȃɂ��o�͂��Ȃ��D
	if (not ofs) { return false; }

	//���ʂ��o�͂���D
	ofs << recoder << std::endl;


	//���Ԃ̓��v���o�͂���D
	double max_time = recoder.graph_search_result_recoder[1].computation_time;	//�ŏ��̃m�[�h�͏���(�v�Z����0�ŌŒ�̂���)
	double min_time = max_time;
	double sum_time = 0.0;

	for (const auto& i : recoder.graph_search_result_recoder)
	{
		if (i.computation_time > max_time) { max_time = i.computation_time; }

		if (i.computation_time < min_time) { min_time = i.computation_time; }

		sum_time += i.computation_time;
	}

	const double average_time = sum_time / static_cast<double>(recoder.graph_search_result_recoder.size());

	ofs << "�ő�T������," << dlm::ConvertDoubleToString(max_time) << ",[msec]" << std::endl;
	ofs << "�ŏ��T������," << dlm::ConvertDoubleToString(min_time) << ",[msec]" << std::endl;
	ofs << "�����T������," << dlm::ConvertDoubleToString(sum_time) << ",[msec]" << std::endl;
	ofs << "���ϒT������," << dlm::ConvertDoubleToString(average_time) << ",[msec]" << std::endl;


	// �ړ������̓��v���o�͂���
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

		const double x_move_average = x_move_sum / static_cast<double>(recoder.graph_search_result_recoder.size() - 1);
		const double y_move_average = y_move_sum / static_cast<double>(recoder.graph_search_result_recoder.size() - 1);
		const double z_move_average = z_move_sum / static_cast<double>(recoder.graph_search_result_recoder.size() - 1);

		ofs << "X�������ړ�����," << dlm::ConvertDoubleToString(x_move_sum) << ",[mm]" << std::endl;
		ofs << "Y�������ړ�����," << dlm::ConvertDoubleToString(y_move_sum) << ",[mm]" << std::endl;
		ofs << "Z�������ړ�����," << dlm::ConvertDoubleToString(z_move_sum) << ",[mm]" << std::endl;
		ofs << "X�������ψړ�����," << dlm::ConvertDoubleToString(x_move_average) << ",[mm/����]" << std::endl;
		ofs << "Y�������ψړ�����," << dlm::ConvertDoubleToString(y_move_average) << ",[mm/����]" << std::endl;
		ofs << "Z�������ψړ�����," << dlm::ConvertDoubleToString(z_move_average) << ",[mm/����]" << std::endl;
	}

	ofs.close();

	return true;
}
