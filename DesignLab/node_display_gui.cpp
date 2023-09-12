#include "node_display_gui.h"

#include "DxLib.h"

#include "leg_state.h"


const int NodeDisplayGUI::BOX_SIZE_X = 450;
const int NodeDisplayGUI::BOX_SIZE_Y = 550;
const int NodeDisplayGUI::BOX_SIZE_Y_CLOSED = 50;



NodeDisplayGUI::NodeDisplayGUI(const int x_pos, const int y_pos, std::shared_ptr<AbstractHexapodStateCalculator> calc) :
	kGUILeftPosX(x_pos), kGUITopPosY(y_pos), mp_calculator(calc)
{
	//�{�^�����쐬����
	const int kButtonSizeX = 100;
	const int kButtonSizeY = 30;

	m_buttons[EButtonType::OPEN_CLOSE] = std::make_unique<ButtomController>(kGUILeftPosX + BOX_SIZE_X - kButtonSizeX / 2 - 10, kGUITopPosY + 10 + kButtonSizeY / 2,
		kButtonSizeX, kButtonSizeY, "�ő�/����");
	m_buttons[EButtonType::SWITCHING] = std::make_unique<ButtomController>(kGUILeftPosX + BOX_SIZE_X - kButtonSizeX / 2 - 10, kGUITopPosY + BOX_SIZE_Y - kButtonSizeY / 2 - 10,
		kButtonSizeX, kButtonSizeY, "�؂�ւ�");

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		m_joint_state[i].global_joint_position.resize(4);
		m_joint_state[i].local_joint_position.resize(4);
		m_joint_state[i].joint_angle.resize(3);
	}
}


void NodeDisplayGUI::setDisplayNode(const SNode& node)
{
	//�m�[�h���Z�b�g����
	m_node = node;

	if (!mp_calculator) { return; }

	// �֐߂̊p�x���Z�b�g����
	mp_calculator->calculateAllJointState(m_node, m_joint_state);
}


void NodeDisplayGUI::update()
{
	//�{�^���̍X�V���s��
	for (auto& button : m_buttons)
	{
		button.second->update();

		if (button.first == EButtonType::OPEN_CLOSE && button.second->isPushedNow())
		{
			m_is_closed = !m_is_closed;
		}
		else if (button.first == EButtonType::SWITCHING && !m_is_closed && button.second->isPushedNow())
		{
			if (m_display_type == EDisplayType::DEFUALT)
			{
				m_display_type = EDisplayType::LEG_STATE;
			}
			else
			{
				m_display_type = EDisplayType::DEFUALT;
			}
		}
	}
}


void NodeDisplayGUI::draw() const
{
	// �g
	drawBackground();

	// �e�L�X�g
	if (!m_is_closed)
	{
		if (m_display_type == EDisplayType::DEFUALT)
		{
			drawNodeInfo();
		}
		else
		{
			drawJointInfo();
		}
	}


	//�{�^����`�悷��
	for (auto& button : m_buttons)
	{
		if (!(button.first == EButtonType::SWITCHING && m_is_closed))
		{
			button.second->draw();
		}
	}
}


void NodeDisplayGUI::drawBackground() const
{
	const unsigned int kBoxColor = GetColor(255, 255, 255);
	const unsigned int kBoxAlpha = 200;

	if (m_is_closed)
	{
		SetDrawBlendMode(DX_BLENDMODE_ALPHA, kBoxAlpha);
		DrawBox(kGUILeftPosX, kGUITopPosY, kGUILeftPosX + BOX_SIZE_X, kGUITopPosY + BOX_SIZE_Y_CLOSED, kBoxColor, TRUE);
		SetDrawBlendMode(DX_BLENDMODE_NOBLEND, 0);
	}
	else
	{
		SetDrawBlendMode(DX_BLENDMODE_ALPHA, kBoxAlpha);
		DrawBox(kGUILeftPosX, kGUITopPosY, kGUILeftPosX + BOX_SIZE_X, kGUITopPosY + BOX_SIZE_Y, kBoxColor, TRUE);
		SetDrawBlendMode(DX_BLENDMODE_NOBLEND, 0);
	}

}

void NodeDisplayGUI::drawNodeInfo() const
{
	const unsigned int kTextColor = GetColor(10, 10, 10);
	const unsigned int kBaseTextColor = GetColor(80, 80, 80);
	const int kTextXPos = kGUILeftPosX + 10;
	const int kTextYMinPos = kGUITopPosY + 10;
	const int kTextYInterval = 30;

	int text_line = 0;

	DrawFormatString(kTextXPos, kTextYMinPos + kTextYInterval * (text_line++), kTextColor, "�d�S�F%d�C�r�ʒu�F%d,%d,%d,%d,%d,%d", dl_leg::getComPatternState(m_node.leg_state),
		dl_leg::getLegState(m_node.leg_state, 0), dl_leg::getLegState(m_node.leg_state, 1), dl_leg::getLegState(m_node.leg_state, 2),
		dl_leg::getLegState(m_node.leg_state, 3), dl_leg::getLegState(m_node.leg_state, 4), dl_leg::getLegState(m_node.leg_state, 5));

	// �d�S��\������
	DrawFormatString(kTextXPos, kTextYMinPos + kTextYInterval * (text_line++), kTextColor,
		"�d�S�ʒu(x:%5.3f,y:%5.3f,z:%5.3f)", m_node.global_center_of_mass.x, m_node.global_center_of_mass.y, m_node.global_center_of_mass.z);

	// ��]��\������
	DrawFormatString(kTextXPos, kTextYMinPos + kTextYInterval * (text_line++), kTextColor,
		"��](roll:%5.3f,pitch:%5.3f,yaw:%5.3f)", m_node.rot.roll, m_node.rot.pitch, m_node.rot.yaw);

	//�V�r���ڒn�r��
	std::string str = "";
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		if (dl_leg::isGrounded(m_node.leg_state, i)) { str += "�ڒn,"; }
		else { str += "�V�r,"; }
	}
	DrawFormatString(kTextXPos, kTextYMinPos + kTextYInterval * (text_line++), kTextColor, "�r�̏�ԁF%s", str.c_str());

	// �r�̈ʒu��\������
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		DrawFormatString(kTextXPos, kTextYMinPos + kTextYInterval * (text_line++), kTextColor,
			"%d�ԋr�̈ʒu(x:%5.3f,y:%5.3f,z:%5.3f)", i, m_node.leg_pos[i].x, m_node.leg_pos[i].y, m_node.leg_pos[i].z);
	}

	// �r�̊���W��\������
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		DrawFormatString(kTextXPos, kTextYMinPos + kTextYInterval * (text_line++), kBaseTextColor,
			" %d�ԋr�̊���W(x:%5.3f,y:%5.3f,z:%5.3f)", i, m_node.leg_base_pos[i].x, m_node.leg_base_pos[i].y, m_node.leg_base_pos[i].z);
	}

	// �[���Ǝ��̓����\������
	DrawFormatString(kTextXPos, kTextYMinPos + kTextYInterval * (text_line++), kTextColor,
		"�[���F%d, ���̓��� : %s", m_node.depth, std::to_string(m_node.next_move).c_str());

	DrawFormatString(kTextXPos, kTextYMinPos + kTextYInterval * (text_line++), kTextColor, "�P�ʂ͒�����[mm]�C�p�x��[rad]");
}


void NodeDisplayGUI::drawJointInfo() const
{
	if (!mp_calculator) { return; }


	const unsigned int kTextColor = GetColor(10, 10, 10);
	const unsigned int kErrorTextColor = GetColor(128, 10, 10);
	const int kTextXPos = kGUILeftPosX + 10;
	const int kTextYMinPos = kGUITopPosY + 50;
	const int kTextYInterval = 30;


	int text_line = 0;


	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		DrawFormatString(kTextXPos, kTextYMinPos + kTextYInterval * (text_line++), kTextColor, "[%d] c %3.3f[deg],f %3.3f[deg],t %3.3f[deg]", i,
			dl_math::convertRadToDeg(m_joint_state[i].joint_angle[0]), dl_math::convertRadToDeg(m_joint_state[i].joint_angle[1]), dl_math::convertRadToDeg(m_joint_state[i].joint_angle[2]));

		DrawFormatString(kTextXPos, kTextYMinPos + kTextYInterval * (text_line++), kTextColor, "    c %3.3f[mm],f %3.3f[mm],t %3.3f[mm]",
			(m_joint_state[i].local_joint_position[0] - m_joint_state[i].local_joint_position[1]).length(),
			(m_joint_state[i].local_joint_position[1] - m_joint_state[i].local_joint_position[2]).length(),
			(m_joint_state[i].local_joint_position[2] - m_joint_state[i].local_joint_position[3]).length()
		);


		if (mp_calculator->isLegInRange(i, m_joint_state[i].local_joint_position[3]))
		{
			DrawFormatString(kTextXPos, kTextYMinPos + kTextYInterval * (text_line++), kTextColor, "    �ߎ��l true");
		}
		else
		{
			DrawFormatString(kTextXPos, kTextYMinPos + kTextYInterval * (text_line++), kErrorTextColor, "    �ߎ��l false");
		}


		std::string str = "";
		if (m_joint_state[i].joint_angle[0] < HexapodConst::PHANTOMX_COXA_DEFAULT_ANGLE[i] + HexapodConst::PHANTOMX_COXA_ANGLE_MIN) { str += "coxa_min "; }
		if (m_joint_state[i].joint_angle[0] > HexapodConst::PHANTOMX_COXA_DEFAULT_ANGLE[i] + HexapodConst::PHANTOMX_COXA_ANGLE_MAX) { str += "coxa_max "; }
		if (m_joint_state[i].joint_angle[1] < HexapodConst::PHANTOMX_FEMUR_ANGLE_MIN) { str += "femur_min "; }
		if (m_joint_state[i].joint_angle[1] > HexapodConst::PHANTOMX_FEMUR_ANGLE_MAX) { str += "femur_max "; }
		if (m_joint_state[i].joint_angle[2] < HexapodConst::PHANTOMX_TIBIA_ANGLE_MIN) { str += "tibia_min "; }
		if (m_joint_state[i].joint_angle[2] > HexapodConst::PHANTOMX_TIBIA_ANGLE_MAX) { str += "tibia_max "; }

		if (!str.empty())
		{
			DrawFormatString(kTextXPos, kTextYMinPos + kTextYInterval * (text_line++), kErrorTextColor, "error %s", str.c_str());
		}
	}
}
