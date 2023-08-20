#include "mouse.h"

#include "DxLib.h"


Mouse::Mouse()
{
	//�ϐ�������������
	m_posx = 0;
	m_poy = 0;
	m_past_posx = 0;
	m_past_posy = 0;
	m_pushing_count_left = 0;
	m_pushing_count_middle = 0;
	m_pushing_count_right = 0;
	m_releasing_count_left = 0;
	m_releasing_count_middle = 0;
	m_releasing_count_right = 0;
	m_wheel_rot = 0;
}


void Mouse::update()
{
	//�}�E�X�̈ʒu�擾
	m_past_posx = m_posx;
	m_past_posy = m_poy;
	GetMousePoint(&m_posx, &m_poy);

	//���N���b�N
	if ((GetMouseInput() & MOUSE_INPUT_LEFT) != 0)
	{
		//������Ă���Ȃ�
		m_pushing_count_left++;
		m_releasing_count_left = 0;
	}
	else
	{
		//������Ă���Ȃ�
		m_pushing_count_left = 0;
		m_releasing_count_left++;
	}

	//�E�N���b�N
	if ((GetMouseInput() & MOUSE_INPUT_RIGHT) != 0)
	{
		//������Ă���Ȃ�
		m_pushing_count_right++;
		m_releasing_count_right = 0;
	}
	else
	{
		//������Ă���Ȃ�
		m_pushing_count_right = 0;
		m_releasing_count_right++;
	}

	//�z�[���h�{�^��
	if ((GetMouseInput() & MOUSE_INPUT_MIDDLE) != 0)
	{
		//������Ă���Ȃ�
		m_pushing_count_middle++;
		m_releasing_count_middle = 0;
	}
	else
	{
		//������Ă���Ȃ�
		m_pushing_count_middle = 0;
		m_releasing_count_middle++;
	}

	//�z�C�[����]
	m_wheel_rot = GetMouseWheelRotVol();
}


int Mouse::getPosX() const
{
	return m_posx;
}


int Mouse::getDiffPosX() const
{
	return m_posx - m_past_posx;
}


int Mouse::getPosY() const
{
	return m_poy;
}


int Mouse::getDiffPosY() const
{
	return m_poy - m_past_posy;
}


int Mouse::getPushingCountLeft() const
{
	return m_pushing_count_left;
}


int Mouse::getPushingCountRight() const
{
	return m_pushing_count_right;
}


int Mouse::getPushingCountMiddle() const
{
	return m_pushing_count_middle;
}


int Mouse::getReleasingCountLeft() const
{
	return m_releasing_count_left;
}


int Mouse::getReleasingCountRight() const
{
	return m_releasing_count_right;
}


int Mouse::getReleasingCountMiddle() const
{
	return m_releasing_count_middle;
}


int Mouse::getWheelRot() const
{
	return m_wheel_rot;
}
