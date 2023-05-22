#include "Dxlib3DFunction.h"
#include "GraphicConst.h"
#include "DxLib.h"

void myDxlib3DFunc::initDxlib3D()
{
	//�J�����̕`��͈͂�ݒ肷��
	SetCameraNearFar(GraphicConst::CAMERA_NEAR, GraphicConst::CAMERA_FAR);	

	SetUseLighting(FALSE);					// ���C�e�B���O�̌v�Z�����Ȃ��悤�ɐݒ��ύX	
	SetUseBackCulling(FALSE);				// �|���S���̗��ʂ�`�悷��D
	SetUseZBuffer3D(TRUE);					// �y�o�b�t�@��L���ɂ���
	SetWriteZBuffer3D(TRUE);				// �y�o�b�t�@�ւ̏������݂�L���ɂ���
	SetFogEnable(FALSE);					// �t�H�O�͎g�p���Ȃ��D
	SetBackgroundColor(0xDA, 0xEC, 0xED);	// �w�i�F�̐ݒ�
}
