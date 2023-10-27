#include "pch.h"

#include <thread>

#include "../DesignLab/stopwatch.h"
#include "../DesignLab/stopwatch.cpp"	//�ǂ���1�̃t�@�C���ŁCcpp�t�@�C�����C���N���[�h����K�v������


// ���Ԍv���N���X�̃e�X�g�D���x���ǂ̒��x�o��̂��킩��Ȃ��̂ŁC�����Ńe�X�g����Ӗ�������̂��͕�����Ȃ�...
// �ǂ�����΂悢���m���Ă�l�͋����Ă���D


namespace designlab::test::common 
{
	TEST(StopwatchTest, Constructor) 
	{
		//�R���X�g���N�^���쐬�����i�K�ŁC�^�C�}�[���J�n����D
		Stopwatch sw;

		//0.1�b�҂�
		std::this_thread::sleep_for(std::chrono::milliseconds(100));

		sw.End();

		EXPECT_DOUBLE_EQ(sw.GetElapsedMilliSecond(), 100.0) << "���x���łȂ��\������ł��D���̃e�X�g�ɂ͎��s���Ă����Ȃ���������܂���D";
	}
}