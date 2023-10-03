#include "result_viewer_system_main.h"

#include <filesystem>

#include <boost/thread.hpp>

#include "cmdio_util.h"
#include "file_tree.h"
#include "map_state.h"
#include "result_file_exporter.h"


namespace dlio = designlab::cmdio;


ResultViewerSystemMain::ResultViewerSystemMain(
	std::unique_ptr<IGraphicMain>&& graphic_ptr,
	const std::shared_ptr<GraphicDataBroker>& broker_ptr,
	const std::shared_ptr<const ApplicationSettingRecorder> setting_ptr
) :
	graphic_system_(std::move(graphic_ptr), setting_ptr),
	broker_ptr_(broker_ptr)
{
}

void ResultViewerSystemMain::Main()
{
	dlio::OutputTitle("Result Viewer System");

	// GUI��\������
	boost::thread graphic_thread(&GraphicSystem::Main, &graphic_system_);

	while (true)
	{
		// �t�@�C���c���[��\�����C�t�@�C����I������
		FileTree file_tree;

		std::string res_path;
		
		if (not file_tree.SelectFile(ResultFileConst::kDirectoryPath, -1, "csv", ResultFileConst::kNodeListName, &res_path)) 
		{
			dlio::Output("�Y���̃f�[�^������܂���ł����D�I�����܂��D", OutputDetail::kSystem);

			break;
		}


		// �t�@�C����ǂݍ���

		std::vector<RobotStateNode> graph;		// �f�[�^���󂯎�邽�߂̕ϐ�
		MapState map_state;

		if (result_importer_.ImportNodeListAndMapState(res_path, &graph, &map_state)) 
		{
			// �f�[�^�𒇉�l�ɓn��
			broker_ptr_->graph.SetData(graph);
			broker_ptr_->map_state.SetData(map_state);

			// �f�[�^��\������
			dlio::Output("�f�[�^��\�����܂��D", OutputDetail::kSystem);
			dlio::OutputNewLine(1, OutputDetail::kSystem);
			dlio::WaitAnyKey();
			dlio::OutputNewLine(1, OutputDetail::kSystem);
			dlio::OutputHorizontalLine(true, OutputDetail::kSystem);
		}
		else 
		{
			dlio::Output("�t�@�C���̓ǂݍ��݂Ɏ��s���܂����D�I�����܂��D", OutputDetail::kSystem);
		}

		// �I�����邩�ǂ�����I��

		if (dlio::InputYesNo("�I�����܂����H"))
		{
			dlio::OutputNewLine(1, OutputDetail::kSystem);

			break;
		}

		dlio::OutputNewLine(1, OutputDetail::kSystem);
	}


	// GUI�̏I����҂�
	dlio::OutputHorizontalLine(true, OutputDetail::kSystem);
	dlio::OutputNewLine(1, OutputDetail::kSystem);
	dlio::Output("GUI�̏I����҂��Ă��܂�", OutputDetail::kSystem);

	graphic_thread.join();
}