#pragma once

// GraphicMain�� Dxlib�̏������s���Ă����N���X�ł��D
// Dxlib(�f���b�N�X ���C�u����)�̓E�B���h�E��\�����āC�����R�}���h���C���ɕ�����\�����邾���̎₵���v���O�����ɍʂ��^���Ă�����ł��D
// ��ɃQ�[���v���O���~���O������ۂɁC�E�B���h�E��\�����邽�߂̃��C�u�����Ƃ��Ďg�p����܂��D
// Dxlib�ȊO�ɂ� OpenCV�Ȃǂɂ��E�B���h�E��\������@�\������܂����C����̃v���O�����ł�Dxlib��p���Č��ʂ�\�����܂��D
// Dxlib�� WinAPI �Ƃ���Windows�̃A�v���P�[�V��������邽�߂̋@�\���C�g���₷�����Ă���郉�C�u�����ł��D

class GraphicMain final
{
public:
	GraphicMain() = default;
	~GraphicMain() = default;

	//Dxlib�̏��������s���܂��D���s�����false��Ԃ��܂��D
	bool init();

	// �E�B���h�E�̕\�����s���Ă����֐��ł��Dboost::thread�ɂ��̊֐���n���ĕ��񏈗����s���܂��Dinit�Ɏ��s���Ă���C�܂���init���ĂԑO�Ɏ��s�������͑����ɏI�����܂��D
	void main();

	//Dxlib�̏I���������s���܂��D
	void finalize();

private:

	bool m_is_init_success = false;
};

//GraphicMain _gr;
//if (_gr.init()) { _gr.main(); }
//_gr.finalize();
//return 0;