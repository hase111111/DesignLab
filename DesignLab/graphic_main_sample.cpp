#include "graphic_main_sample.h"

#include "DxLib.h"

#include "graphic_const.h"
#include "keyboard.h"
#include "mouse.h"


GraphicMainSample::GraphicMainSample(const std::shared_ptr<const SApplicationSettingRecorder>& setting_ptr) :
	kBoxSizeX(setting_ptr ? setting_ptr->window_size_x : 100),
	kBoxSizeY(setting_ptr ? setting_ptr->window_size_y : 100),
	counter_(0)
{
}

bool GraphicMainSample::Update()
{
	counter_++;

	if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_X) == 1 || Mouse::GetIns()->left_pushing_counter() == 1)
	{
		return false;	// X�L�[���}�E�X�̍��N���b�N�������ꂽ�烋�[�v�𔲂���D
	}

	return true;	// ���[�v�𔲂��Ȃ�����true��Ԃ��D
}


void GraphicMainSample::Draw() const
{
	//�w�i�𔒂Ő��߂�D
	DrawBox(0, 0, kBoxSizeX, kBoxSizeY, GetColor(255, 255, 255), TRUE);

	//���b�Z�[�W���E�B���h�E�ɕ\������
	printfDx("GraphicMain�ł͂��̂悤�ɁC�`��݂̂��s��draw�֐��ƁC�l�̍X�V�݂̂������Ȃ�update�֐��ɂ���ăE�B���h�E�𐧌䂷��\n\n");
	printfDx("�N�����Ă���...%lf�b", static_cast<double>(counter_) / 60.0);
	printfDx("\n\nX�L�[���}�E�X�̍��N���b�N�ŏI������D");
}
