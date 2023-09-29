#include "result_viewer_system_main.h"

#include <filesystem>

#include <boost/thread.hpp>

#include "cmdio_util.h"
#include "file_tree.h"
#include "map_state.h"


namespace dlio = designlab::cmdio;


ResultViewerSystemMain::ResultViewerSystemMain(
	std::unique_ptr<IGraphicMain>&& graphic_ptr,
	const std::shared_ptr<const ApplicationSettingRecorder> setting_ptr
) :
	kDirectoryName("result"),
	kFileTreePath(std::filesystem::current_path().string() + "/" + kDirectoryName),
	graphic_system_(std::move(graphic_ptr), setting_ptr)
{
}

void ResultViewerSystemMain::Main()
{
	dlio::OutputTitle("Result Viewer System");

	// GUI��\������
	boost::thread graphic_thread(&GraphicSystem::Main, &graphic_system_);

	while (true)
	{
		dlio::Output("�t�@�C����I�����Ă��������D", OutputDetail::kSystem);


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

void ResultViewerSystemMain::Read()
{

}
