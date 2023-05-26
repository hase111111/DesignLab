#include "GUIController.h"
#include "DxLib.h"
#include "LegState.h"
#include <string>
#include "Keyboard.h"

GUIController::GUIController()
{
	const int RIGHTX = GraphicConst::WIN_X - (CENTER_X + BOX_X / 2);
	const int RIGHTY = CENTER_Y - BOX_Y / 2;

	const int _camera_buttomx = 100;
	const int _camera_buttomy = 50;
	const int _camera_y = RIGHTY + 150;
	m_buttom.push_back(std::make_unique<ButtomController>(RIGHTX + _camera_buttomx * 6 / 4, _camera_y, _camera_buttomx, _camera_buttomy, "�^��J����"));
	m_buttom.push_back(std::make_unique<ButtomController>(RIGHTX + _camera_buttomx * 9 / 4, _camera_y + _camera_buttomy * 5 / 4, _camera_buttomx, _camera_buttomy, "���ՃJ����"));
	m_buttom.push_back(std::make_unique<ButtomController>(RIGHTX + _camera_buttomx * 9 / 4, _camera_y + _camera_buttomy * 10 / 4, _camera_buttomx, _camera_buttomy, "�^���J����"));
	m_buttom.push_back(std::make_unique<ButtomController>(RIGHTX + _camera_buttomx * 3 / 4, _camera_y + _camera_buttomy * 5 / 4, _camera_buttomx, _camera_buttomy, "����(���])"));
	m_buttom.push_back(std::make_unique<ButtomController>(RIGHTX + _camera_buttomx * 3 / 4, _camera_y + _camera_buttomy * 10 / 4, _camera_buttomx, _camera_buttomy, "�^��(���])"));
}

void GUIController::update(CameraController& _camera)
{
	//�\���ؑփL�[�������ꂽ�Ȃ�΁C�\���؂�Ԃ��t���O�̒l���������܂ɂ���D
	if (Keyboard::getIns()->getPressingCount(KEY_INPUT_Z) == 1) { m_is_displayed = !m_is_displayed; }

	//�\�����Ă��Ȃ����Ƀ{�^���̏������s��Ȃ�
	if (m_is_displayed == false) { return; }

	//�{�^�����X�V����D
	for (int i = 0; i < (int)m_buttom.size(); i++)
	{
		m_buttom.at(i)->update();

		//�{�^���̒ǉ����ꂽ���Ԃ��Ƃɏ���������D�N�\�݂����Ȏ����ŃX�}�\
		switch (i)
		{
		case 0:
			if (m_buttom.at(i)->isPushedNow() == true) { _camera.setCameraMode(ECameraMode::TopView); }
			break;

		case 1:
			if (m_buttom.at(i)->isPushedNow() == true) { _camera.setCameraMode(ECameraMode::OverheadView); }
			break;

		case 2:
			if (m_buttom.at(i)->isPushedNow() == true) { _camera.setCameraMode(ECameraMode::SideView); }
			break;

		case 3:
			if (m_buttom.at(i)->isPushedNow() == true) { _camera.setCameraMode(ECameraMode::OverheadViewFlip); }
			break;

		case 4:
			if (m_buttom.at(i)->isPushedNow() == true) { _camera.setCameraMode(ECameraMode::SideViewFlip); }
			break;

		default:
			break;
		}
	}
}

void GUIController::draw(const SNode _node) const
{
	//�\�����Ȃ��Ȃ瑦�I��
	if (m_is_displayed == false) { return; }

	const int _base_color = GetColor(255, 255, 255);

	//�������ɕ`�悷��D
	SetDrawBlendMode(DX_BLENDMODE_ALPHA, 200);

	//���n��`�悷��
	DrawBox(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2, CENTER_X + BOX_X / 2, CENTER_Y + BOX_Y / 2, _base_color, TRUE);

	//�E��
	DrawBox(GraphicConst::WIN_X - (CENTER_X - BOX_X / 2), CENTER_Y - BOX_Y / 2, GraphicConst::WIN_X - (CENTER_X + BOX_X / 2), CENTER_Y + BOX_Y / 2, _base_color, TRUE);

	//���������猳�ɖ߂��D����Y���ƕ`�悪���󂷂�̂Œ���
	SetDrawBlendMode(DX_BLENDMODE_NOBLEND, 0);

	//�m�[�h�̏�Ԃ𕶎��ŏo�͂���D
	drawNodeByStr(_node);

	drawExplanationByStr();

	//�{�^����`�悷��D
	for (const auto& i : m_buttom)
	{
		i->draw();
	}
}

void GUIController::drawNodeByStr(const SNode _node) const
{
	const int _str_color = GetColor(54, 54, 54);
	const int _line_height = 28;

	//���݂̍s������\������n�_������o���Ă����֐��I�u�W�F�N�g�D
	int _now_line = 0;
	auto line = [_line_height, &_now_line]() -> int { return _line_height * (_now_line++); };

	//�����Ȃ����̂ŃX�R�[�v�̒��ɂ���Ă����C�ʂɓ���Ȃ��Ƃ������͕ς��Ȃ��̂����ǁCvisual stdio�̋@�\�Ő܂肽���߂�̂Ō����ڂ𐮗��ł���D

	//�r���
	{
		std::string _leg_state_bit_str = "";

		//int �^��32bit�Ȃ̂� 32�񃋁[�v����D
		const int _INTEGER_BIT_SIZE = 32;

		for (int i = 4; i < _INTEGER_BIT_SIZE; i++)
		{
			//�\������s����C���bit���璲�ׂ�D
			if (_node.leg_state & 1 << (_INTEGER_BIT_SIZE - i - 1))
			{
				//bit�������Ă���Ȃ�
				_leg_state_bit_str += "1";
			}
			else
			{
				//bit�������Ă��Ȃ��Ȃ�
				_leg_state_bit_str += "0";
			}

			// 4�Ŋ������]�肪3�Ȃ�C4bit���ƂɃX�y�[�X�����ꂽ���D
			if (i % 4 == 3)
			{
				_leg_state_bit_str += " ";
			}
		}

		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), _str_color, "�r���");
		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), _str_color, _leg_state_bit_str.c_str());
		line();		//���s
	}

	//�r�ׂ̍������D
	{
		std::string temp_str = "";
		auto _ground_or_lift = [](int _state, int _num) -> std::string
		{
			if (LegState::isGrounded(_state, _num) == true) { return "�ڒn"; }
			else { return "�V�r"; }
		};

		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), _str_color, "�r�T�@�@�@�@�@�@�r�O");
		temp_str = "�@" + _ground_or_lift(_node.leg_state, 5) + "�@�@�@�@�@�@" + _ground_or_lift(_node.leg_state, 0);
		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), _str_color, temp_str.c_str());
		temp_str = "�@���U���ʒu�F" + std::to_string(LegState::getLegState(_node.leg_state, 5)) + " ���U���ʒu�F" + std::to_string(LegState::getLegState(_node.leg_state, 0));
		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), _str_color, temp_str.c_str());

		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), _str_color, "�r�S�@�@�@�@�@�@�r�P");
		temp_str = "�@" + _ground_or_lift(_node.leg_state, 4) + "�@�@�@�@�@�@" + _ground_or_lift(_node.leg_state, 1);
		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), _str_color, temp_str.c_str());
		temp_str = "�@���U���ʒu�F" + std::to_string(LegState::getLegState(_node.leg_state, 4)) + " ���U���ʒu�F" + std::to_string(LegState::getLegState(_node.leg_state, 1));
		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), _str_color, temp_str.c_str());

		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), _str_color, "�r�R�@�@�@�@�@�@�r�Q");
		temp_str = "�@" + _ground_or_lift(_node.leg_state, 3) + "�@�@�@�@�@�@" + _ground_or_lift(_node.leg_state, 2);
		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), _str_color, temp_str.c_str());
		temp_str = "�@���U���ʒu�F" + std::to_string(LegState::getLegState(_node.leg_state, 3)) + " ���U���ʒu�F" + std::to_string(LegState::getLegState(_node.leg_state, 2));
		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), _str_color, temp_str.c_str());
	}

	//�d�S�ɂ���
	{
		std::string str = "�d�S�p�^�[��" + std::to_string(LegState::getComPatternState(_node.leg_state));

		line();	//���s
		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), _str_color, str.c_str());

		str = "�d�S���W";
		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), _str_color, str.c_str());
		str = "�@x = " + std::to_string(_node.global_center_of_mass.x);
		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), _str_color, str.c_str());
		str = "�@y = " + std::to_string(_node.global_center_of_mass.y);
		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), _str_color, str.c_str());
		str = "�@z = " + std::to_string(_node.global_center_of_mass.z);
		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), _str_color, str.c_str());
	}

	//��]�ɂ���
	{
		std::string str = "��]";

		line();	//���s
		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), _str_color, str.c_str());

		str = "�@pitch = " + std::to_string(_node.pitch);
		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), _str_color, str.c_str());
		str = "�@roll  = " + std::to_string(_node.roll);
		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), _str_color, str.c_str());
		str = "�@yaw   = " + std::to_string(_node.yaw);
		DrawFormatString(CENTER_X - BOX_X / 2, CENTER_Y - BOX_Y / 2 + line(), _str_color, str.c_str());
	}
}

void GUIController::drawExplanationByStr() const
{
	const int RIGHTX = GraphicConst::WIN_X - (CENTER_X + BOX_X / 2);
	const int _str_color = GetColor(54, 54, 54);
	const int _line_height = 28;

	//���݂̍s������\������n�_������o���Ă����֐��I�u�W�F�N�g�D
	int _now_line = 0;
	auto line = [_line_height, &_now_line]() -> int { return _line_height * (_now_line++); };

	DrawFormatString(RIGHTX, CENTER_Y - BOX_Y / 2 + line(), _str_color, "Z�L�[���������Ƃ�UI�̕\���������܂��D");
	DrawFormatString(RIGHTX, CENTER_Y - BOX_Y / 2 + line(), _str_color, "������xZ�L�[���������ƂŖ߂�܂�.");
	DrawFormatString(RIGHTX, CENTER_Y - BOX_Y / 2 + line(), _str_color, "��{�I�ɂ̓N���b�N�ő�������܂��D");
	DrawFormatString(RIGHTX, CENTER_Y - BOX_Y / 2 + line(), _str_color, "�ȉ��̃{�^���ŃJ�����̎��_��ύX�D");
}
