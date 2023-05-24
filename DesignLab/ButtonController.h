#pragma once
#include <string>

// UI�̃{�^���̑�����s���֐��D
class ButtomController final
{
private:
	bool m_is_mouse_in_buttom = false;	//�{�^���̒��Ƀ}�E�X�����邩�D
	bool m_is_pushed = false;			//�{�^���͉�����Ă��邩�D
	int m_pushing_frame = 0;			//�{�^���������ꑱ���Ă��鎞�ԁD
	
	const int X_POS, Y_POS;	//�{�^���̍��W�D
	const int X_SIZE, Y_SIZE;		//�{�^���̉����Əc��
	std::string m_text;				//�{�^���ɏ�����Ă���e�L�X�g�D

public:
	ButtomController();
	ButtomController(const int _xpos, const int _ypos, const int _xsize, const int _ysize, const std::string _text);
	~ButtomController() = default;

	void update();				//�{�^���̏�Ԃ��X�V����D���t���[�����s���邱�ƁD
	void draw() const;			//�{�^����`�悷��D�f�U�C����ύX�������Ȃ�΂�����ύX���Ă��������D

	bool isPushedNow() const;	//�{�^���������ꂽ�u�Ԃ�true��Ԃ��D
	bool isPushed() const;		//�{�^����������Ă���Ȃ��true��Ԃ��D
	int getPushingFlame() const;//�{�^�������t���[���ɓn���ĉ����ꑱ���Ă���̂���Ԃ��D

};
