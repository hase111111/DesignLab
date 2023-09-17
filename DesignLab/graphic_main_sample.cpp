#include "graphic_main_sample.h"

#include "DxLib.h"

#include "graphic_const.h"


bool GraphicMainSample::Update()
{
	m_counter++;

	return true;
}


void GraphicMainSample::Draw() const
{
	//�w�i�𔒂Ő��߂�D
	DrawBox(0, 0, mp_setting->window_size_x, mp_setting->window_size_y, GetColor(255, 255, 255), TRUE);

	//���b�Z�[�W���E�B���h�E�ɕ\������
	printfDx("GraphicMain�ł͂��̂悤�ɁC�`��݂̂��s��draw�֐��ƁC�l�̍X�V�݂̂������Ȃ�update�֐��ɂ���ăE�B���h�E�𐧌䂵�܂�\n\n");
	printfDx("�N�����Ă���...%lf�b", static_cast<double>(m_counter) / 60.0);
}
