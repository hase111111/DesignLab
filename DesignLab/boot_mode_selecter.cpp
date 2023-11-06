#include "boot_mode_selecter.h"

#include <string>

#include <magic_enum.hpp>

#include "cmdio_util.h"
#include "designlab_string_util.h"


namespace dlio = designlab::cmdio;
namespace dlsu = designlab::string_util;


BootModeSelecter::BootModeSelecter() :
	kBootModeNum(static_cast<int>(magic_enum::enum_count<BootMode>())),
	default_mode_(BootMode::kSimulation)
{
}


BootMode BootModeSelecter::SelectBootMode()
{
	const OutputDetail output_detail = OutputDetail::kSystem;	// kSystem �ɂ���ƁA�ݒ�ɂ�����炸�K���\�������

	dlio::Output("�N�����[�h��I�����Ă�������", output_detail);

	//�N�����[�h�̖��O��\������
	for (int i = 0; i < kBootModeNum; i++)
	{
		const BootMode boot_mode = static_cast<BootMode>(i);

		const std::string boot_mode_name = dlsu::MyEnumToString(boot_mode);

		dlio::Output(std::to_string(i) + " : " + boot_mode_name, output_detail);
	}


	std::string default_mode_name = dlsu::MyEnumToString(default_mode_);

	dlio::Output("other : �f�t�H���g�̃��[�h ( " + default_mode_name + " )", output_detail);


	dlio::OutputNewLine(1, output_detail);

	//0����BootMode�̐��܂ł̐�������͂�����
	int default_mode_num = static_cast<int>(default_mode_);

	const int input = dlio::InputInt(0, kBootModeNum - 1, default_mode_num);

	//�󂯎�����l��magic_enum��enum_cast��BootMode�ɕϊ�����
	if (magic_enum::enum_cast<BootMode>(input).has_value()) 
	{
		return magic_enum::enum_cast<BootMode>(input).value();
	}
	else
	{
		// ���͂��s���ȏꍇ�̓f�t�H���g�̃��[�h��Ԃ�

		dlio::Output("���͂��s���ł��B�f�t�H���g�̃��[�h��I�����܂��B", OutputDetail::kSystem);
		
		return default_mode_;
	}
}
