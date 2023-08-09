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

void dl_io::setOutputPriority(const EOutputPriority priority)
{
	CmdIO::setOutputPriority(priority);
}

void dl_io::output(const std::string str, const EOutputPriority priority)
{
}

EOutputPriority CmdIO::m_output_priority = EOutputPriority::INFO;