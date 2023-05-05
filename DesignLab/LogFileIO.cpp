#include "LogFileIO.h"
#include "LegState.h"

LogFileIO::LogFileIO()
{
    //�J���t�@�C���̖��O���Z�b�g����
    m_file_name = LOG_NAME + std::to_string(m_log_num) + LOG_EXTENSION;

    //���Ƃ̃v���N�����ɂ������`�ʁC�K�v�����킩���
    m_log_num++;
}

bool LogFileIO::openLogFile()
{
	m_all_log.open(m_file_name);

	if (!m_all_log) 
	{
		//�t�@�C���쐬�Ɏ��s����Ƃ��������s����
		return false;
	}

	//�ŏ��Ƀ��O�ɓ��͂���
	m_all_log << "number, v,,,,,,kaisou,,,,,, COMTYPE, center_of_mass, thP, thR, thY, leg[0],,, Leg[1],,, leg[2],,, leg[3],,, leg[4],,, leg[5],,, leg2[0],,, Leg2[1],,, leg2[2],,, leg2[3],,, leg2[4],,, leg2[5],,, global_center_of_mass,,, debug, kaisoubit,,,,,last_node_num,time" << std::endl;
	m_all_log << ", leg0, leg1, leg2, leg3, leg4, leg5, leg0, leg1, leg2, leg3, leg4, leg5, , , , , , x, y, z, x, y, z, x, y, z, x, y, z, x, y, z, x, y, z, x, y, z, x, y, z, x, y, z, x, y, z, x, y, z, x, y, z, x, y, z," << std::endl;

    return true;
}

void LogFileIO::closeLogFile()
{
	m_all_log.close();
}

void LogFileIO::addLogStringWithInt(const int _data)
{
	m_all_log << _data << std::endl;
}

void LogFileIO::addLogStringWithNode(const int num, const LNODE& node_log)
{
	m_all_log << num << ",";

	for (int j = 0; j < 6; ++j) 
	{
		m_all_log << LegState::isGrounded(node_log.leg_state, j) << ",";
	}
	for (int j = 0; j < 6; ++j) 
	{
		m_all_log << numOfLegPosi(node_log.leg_state, j) << ",";
	}

	m_all_log << numOfCOMType(node_log.leg_state) << "," << node_log.center_of_mass << "," << node_log.pitch << "," << node_log.roll << "," << node_log.yaw << ",";

	for (int j = 0; j < 6; ++j) 
	{
		m_all_log << node_log.Leg[j].x << "," << node_log.Leg[j].y << "," << node_log.Leg[j].z << ",";
	}

	for (int j = 0; j < 6; ++j) 
	{
		m_all_log << node_log.Leg2[j].x << "," << node_log.Leg2[j].y << "," << node_log.Leg2[j].z << ",";
	}

	m_all_log << node_log.global_center_of_mass.x << "," << node_log.global_center_of_mass.y << "," << node_log.global_center_of_mass.z << "," << node_log.debug << ",";

	for (int j = 0; j < 6; ++j) 
	{
		m_all_log << std::bitset<4>(numOfLegPosi(node_log.leg_state, j)) << ",";
	}

	m_all_log << node_log.last_node_num << ",";

	m_all_log << node_log.time << ",";

	m_all_log << std::endl;
}

void LogFileIO::addLogString(const std::string _str)
{
	m_all_log << _str << std::endl;
}

void LogFileIO::addLogStringSimulation(const int _loop_num, const SimulateResult& _res)
{
	m_all_log << "���s�̐�����," << _res.getClearNum() * 100 / _loop_num << ",��" << std::endl;
	m_all_log << "1�V�~�����[�V����������̕��ϓ��B����," << _res.getDistanceMoveYSum() / _loop_num << ",[mm/�V�~�����[�V����]" << std::endl;
	m_all_log << "�ő哞�B����," << _res.getDistanceMoveYMax() << ",[mm]" << std::endl;
	m_all_log << "�ŏ����B����," << _res.getDistanceMoveYMin() << ",[mm]" << std::endl;
	m_all_log << "1���e�p�^�[������������̕��ϒT������," << _res.getGatePatternGenerateTimeSum() / _res.getGatePatternGenerateSum() << ",[s]" << std::endl;
	m_all_log << "�ő�T������," << _res.getGatePatternGenerateTimeMax() << ",[s]" << std::endl;
	m_all_log << "�ŏ��T������," << _res.getGatePatternGenerateTimeMin() << ",[s]" << std::endl << std::endl;

	m_all_log << "�J��Ԃ�����Œ��f��������," << _res.getFailedByGatePatternLoop() * 100 / _loop_num << ",��" << std::endl;
	m_all_log << "���e�p�^�[��������ꂸ���f��������," << _res.getFailedByNoGatePattern() * 100 / _loop_num << ",��" << std::endl;
	m_all_log << "���e�p�^�[���𐶐���������," << _res.getGatePatternGenerateSum() << ",��" << std::endl;
	m_all_log << "���e�p�^�[�������Ɋ|������������," << _res.getGatePatternGenerateTimeSum() << ",�b" << std::endl;
	m_all_log << "Y�����ւ̍��v�ړ�����," << _res.getDistanceMoveYSum() << "mm" << std::endl;
	m_all_log << "1���e�p�^�[������������̈ړ�����," << _res.getDistanceMoveYSum() / _res.getGatePatternGenerateSum() << "mm/����" << std::endl;
	m_all_log << "Y�����ւ̕��ψړ����x," << _res.getDistanceMoveYSum() / _res.getGatePatternGenerateTimeSum() << "mm/�b" << std::endl;
}
