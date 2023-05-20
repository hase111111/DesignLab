#include "SimulateResult.h"
#include <limits>

// limits�͊e�ϐ��̌^���Ƃ̍ő�E�ŏ��l�������Ă����MAX_???��ǉ�����

SimulateResult::SimulateResult()
{
	//�e�f�[�^��0�ŏ������D�ő�E�ŏ��͋t�̒l��������
	m_clear_num = 0;						
	m_failed_by_gate_pattern_loop = 0;		
	m_failed_by_no_gate_pattern = 0;		
	m_gate_pattern_generate_sum = 1;	//�S�V�~�����[�V�����ŏo�͂��ꂽ���e�p�^�[���̑���

	//���v�l��0�ɁC�ő�ŏ��͂��̔��΂̒l�ŏ��������Ă���
	m_distance_move_Y_sum = 0;
	m_distance_move_Y_max = INT_MIN;
	m_distance_move_Y_min = INT_MAX;

	//����
	m_gate_parttern_generate_time_sum = 0.0;
	m_gate_parttern_generate_time_max = -100000.0;
	m_gate_parttern_generate_time_min = 100000.0;
}


int SimulateResult::getClearNum() const { return m_clear_num; }

int SimulateResult::getFailedByGatePatternLoop() const { return m_failed_by_gate_pattern_loop; }

int SimulateResult::getFailedByNoGatePattern() const { return m_failed_by_no_gate_pattern; }

int SimulateResult::getGatePatternGenerateSum() const { return m_gate_pattern_generate_sum; }

int SimulateResult::getDistanceMoveYSum() const { return m_distance_move_Y_sum; }

int SimulateResult::getDistanceMoveYMax() const { return m_distance_move_Y_max; }

int SimulateResult::getDistanceMoveYMin() const { return m_distance_move_Y_min; }

double SimulateResult::getGatePatternGenerateTimeSum() const{	return m_gate_parttern_generate_time_sum;}

double SimulateResult::getGatePatternGenerateTimeMax() const { return m_gate_parttern_generate_time_max; }

double SimulateResult::getGatePatternGenerateTimeMin() const { return m_gate_parttern_generate_time_min; }


void SimulateResult::countupClearNum()
{
	m_clear_num++;
}

void SimulateResult::countupGatePatternGenerateSum()
{
	m_gate_pattern_generate_sum++;
}

void SimulateResult::countupFailedByGatePatternLoop()
{
	m_failed_by_gate_pattern_loop++;
}

void SimulateResult::countupFailedByNoGatePattern()
{
	m_failed_by_no_gate_pattern++;
}

void SimulateResult::updateDistanceMoveY(const int _y)
{
	m_distance_move_Y_sum += _y; //���B�����̍��v
	if (m_distance_move_Y_max < _y) m_distance_move_Y_max = _y;//�ő哞�B�������X�V
	if (m_distance_move_Y_min > _y) m_distance_move_Y_min = _y;//�ŏ����B�������X�V
}

void SimulateResult::updateGatePatternGenerateTime(const double _time)
{
	if (m_gate_parttern_generate_time_max < _time) m_gate_parttern_generate_time_max = _time;
	if (m_gate_parttern_generate_time_min > _time) m_gate_parttern_generate_time_min = _time;
	m_gate_parttern_generate_time_sum += _time;
}
