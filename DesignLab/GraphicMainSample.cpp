#include "GraphicMainSample.h"
#include "DxLib.h"
#include "GraphicConst.h"

bool GraphicMainSample::update()
{
	m_counter++;

	return true;
}

void GraphicMainSample::draw() const
{
	//�w�i�𔒂Ő��߂�D
	DrawBox(0, 0, GraphicConst::WIN_X, GraphicConst::WIN_Y, GetColor(255, 255, 255), TRUE);

	//���b�Z�[�W���E�B���h�E�ɕ\������
	printfDx("GraphicMain�ł͂��̂悤�ɁC�`��݂̂��s��draw�֐��ƁC�l�̍X�V�݂̂������Ȃ�update�֐��ɂ���ăE�B���h�E�𐧌䂵�܂�\n\n");
	printfDx("�N�����Ă���...%lf�b", (double)m_counter / 60.0);
}
