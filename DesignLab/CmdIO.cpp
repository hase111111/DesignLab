#include "CmdIO.h"
#include <iostream>
#include "LegState.h"

using namespace LegStateEdit;

void CmdIO::outputString(const std::string _str)
{
	std::cout << _str << std::endl;
}

void CmdIO::outputNode(const SNode& _node, const int _num) const
{
	std::cout << std::endl;
	if (_num < 0) { std::cout << "Node parameter is ..." << std::endl; }
	else if (_num == 0) { std::cout << "First node parameter is ..." << std::endl; }
	else { std::cout << "[Node Number " << _num << " ] Node parameter is ..." << std::endl; }

	std::cout << _node << std::endl;

	////�d�S�p�^�[��
	//std::cout << "\tCOM_type = " << LegStateEdit::getComPatternState(_node.leg_state) << std::endl;
	//std::cout << std::endl;

	////�r�̗V�r�E�ڒn���
	//std::cout << "\tLegs(0,1,2,3,4,5)" << std::endl;
	//std::cout << "\tGround : ";
	//for (int i = 0; i < HexapodConst::LEG_NUM; ++i) { std::cout << (isGrounded(_node.leg_state, i) ? "ground " : "lifted "); }
	//std::cout << std::endl;

	////�r�̊K�w
	//std::cout << "\tHierarchy : ";
	//for (int i = 0; i < HexapodConst::LEG_NUM; ++i) { std::cout << LegStateEdit::getLegState(_node.leg_state, i); }
	//std::cout << std::endl;

	////�r�ʒu
	//std::cout << "\tLeg Position : " << std::endl;
	//for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	//{
	//	std::cout << "\t\tLeg[" << i << "] = " << _node.leg_pos[i] << "\t\tLeg2[" << i << "] = " << _node.Leg2[i] << std::endl;
	//}
	//std::cout << std::endl;

	////�d�S�ʒu
	//std::cout << "\tglobal_center_of_mass = " << _node.global_center_of_mass << std::endl;

	////��]�p��
	//std::cout << "\tRotate : roll = " << _node.rot.roll << ", pitch = " << _node.rot.pitch << ", yaw = " << _node.rot.yaw << std::endl;

	////������
	//std::cout << std::endl;
	//std::cout << "\t(Next Move : " << std::to_string(_node.next_move) << ")" << std::endl;
	//std::cout << "\t(Depth : " << static_cast<int>(_node.depth) << ")" << std::endl;
	//std::cout << "\t(parent number : " << _node.parent_num << ")" << std::endl;


	//std::cout << "\tnode_height = " << _node.node_height << "\n";
	//std::cout << "\tdebug = " << _node.debug << "\n";
	//if (_node.debug % 100 / 10 == 2) std::cout << "\t����2 �r�̓��݂�������" << std::endl;
	//if (_node.debug % 100 / 10 == 3) std::cout << "\t����3 �d�S�̐����ړ�" << std::endl;
	//if (_node.debug % 100 / 10 == 4) std::cout << "\t����4 �r�̐����ړ�" << std::endl;
	//if (_node.debug % 100 / 10 == 6) std::cout << "\t����6 �d�S�̏㉺�ړ�" << std::endl;
	//if (_node.debug % 10 == 2) std::cout << "\t�O��2 �r�̓��݂�������\t";
	//if (_node.debug % 10 == 3) std::cout << "\t�O��3 �d�S�̐����ړ�\t";
	//if (_node.debug % 10 == 4) std::cout << "\t�O��4 �r�̐����ړ�\t";
	//if (_node.debug % 10 == 6) std::cout << "\t�O��6 �d�S�̏㉺�ړ�\t";
	//std::cout << "\t�K�v�ȍ����ړ��� = " << _node.target_delta_comz << std::endl;
	//std::cout << "\t���ۂ̍����ړ��� = " << _node.delta_comz << std::endl;
	//std::cout << "\t�T�������m�[�h�� = " << _node.last_node_num << std::endl;
	//std::cout << "\t�T������ = " << _node.time << std::endl;
	std::cout << std::endl;
}

void CmdIO::outputErrorMessageInGraphSearch(const std::string _err_mes) const
{
	std::cout << "Graph search failed." << std::endl;
	std::cout << "\t" << "by " << _err_mes << std::endl;
	std::cout << std::endl;
}

void CmdIO::outputGraphSearchStaretMessage(const int _simu_num) const
{
	std::cout << std::endl;
	std::cout << "---------------------- Starting new graph search ----------------------" << std::endl;
	std::cout << "This is the " << getOrdinalNumber(_simu_num) << " simulation." << std::endl;
	std::cout << std::endl;
	std::cout << std::endl;
}

void CmdIO::outputSimulateResult(const int _loop_num, const SimulateResult& _res)
{
	std::cout << "���s�̐�����\t" << _res.getClearNum() * 100 / _loop_num << "��" << std::endl;
	std::cout << "1�V�~�����[�V����������̕��ϓ��B����\t" << _res.getDistanceMoveYSum() / _loop_num << "[mm/�V�~�����[�V����]" << std::endl;
	std::cout << "�ő哞�B����\t" << _res.getDistanceMoveYMax() << "[mm]" << std::endl;
	std::cout << "�ŏ����B����\t" << _res.getDistanceMoveYMin() << "[mm]" << std::endl;
	std::cout << "1���e�p�^�[������������̕��ϒT������\t" << _res.getGatePatternGenerateTimeSum() / _res.getGatePatternGenerateSum() << "[s]" << std::endl;
	std::cout << "�ő�T������\t" << _res.getGatePatternGenerateTimeMax() << "[s]" << std::endl;
	std::cout << "�ŏ��T������\t" << _res.getGatePatternGenerateTimeMin() << "[s]" << std::endl << std::endl;

	std::cout << "�J��Ԃ�����Œ��f��������\t" << _res.getFailedByGatePatternLoop() * 100 / _loop_num << "��" << std::endl;
	std::cout << "���e�p�^�[��������ꂸ���f��������\t" << _res.getFailedByNoGatePattern() * 100 / _loop_num << "��" << std::endl;
	std::cout << "���e�p�^�[���𐶐���������\t" << _res.getGatePatternGenerateSum() << "��" << std::endl;
	std::cout << "���e�p�^�[�������Ɋ|������������\t" << _res.getGatePatternGenerateTimeSum() << "�b" << std::endl;
	std::cout << "Y�����ւ̍��v�ړ�����\t" << _res.getDistanceMoveYSum() << "mm" << std::endl;
	std::cout << "1���e�p�^�[������������̈ړ�����\t" << _res.getDistanceMoveYSum() / _res.getGatePatternGenerateSum() << "mm/����" << std::endl;
	std::cout << "Y�����ւ̕��ψړ����x\t" << _res.getDistanceMoveYSum() / _res.getGatePatternGenerateTimeSum() << "mm/�b" << std::endl;
}

std::string CmdIO::getOrdinalNumber(const int _num) const
{
	//�Q�l https://qiita.com/Seiten_Minagawa/items/58a0b2b0cfd578c9bedc

	std::string res = std::to_string(_num);

	if (_num / 10 % 10 == 1) { res += "th"; }
	else
	{
		switch (_num % 10)
		{
		case 1:
			res += "st";
			break;

		case 2:
			res += "nd";
			break;

		case 3:
			res += "rd";
			break;

		default:
			res += "th";
			break;
		}
	}

	return res;
}
