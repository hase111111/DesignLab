#pragma once

//�V�~�����[�V�����̌��ʂ��i�[����N���X�D�ϐ��������Ⴒ���Ⴓ�������Ȃ��̂ō쐬

class SimulateResult final
{
public:
	SimulateResult();
	~SimulateResult() = default;

	//�e��ϐ��̒l���擾����ϐ�
	int getClearNum() const;
	int getFailedByGatePatternLoop() const;
	int getFailedByNoGatePattern() const;
	int getGatePatternGenerateSum() const;

	int getDistanceMoveYSum() const;
	int getDistanceMoveYMax() const;
	int getDistanceMoveYMin() const;

	double getGatePatternGenerateTimeSum() const;
	double getGatePatternGenerateTimeMax() const;
	double getGatePatternGenerateTimeMin() const;

	// clear_num�̒l��+1����
	void countupClearNum();

	// gate_pattern_generate_sum�̒l��+1����
	void countupGatePatternGenerateSum();

	// failed_by_gate_pattern_loop�̒l��+1����
	void countupFailedByGatePatternLoop();

	// failed_by_no_gate_pattern�̒l��+1����
	void countupFailedByNoGatePattern();

	// �����̒l����ő�E�ŏ��l�Ƒ��a���X�V����
	void updateDistanceMoveY(const int _y);

	// �����̒l����ő�E�ŏ��l�Ƒ��a���X�V����
	void updateGatePatternGenerateTime(const double _time);

private:
	// C++�ł͕ϐ��̒l��private�ɂ���ׂ��ŁC�l�̎擾��get???�֐����g��.
	// �������C��������getter��setter���g���悤�ȃv���O�����ɂ��ׂ�����Ȃ��DDon't ask. Tell�̌����ɏ]����

	int m_clear_num;							//��苗�����s�ł��ăV�~�����[�V�������I��������
	int m_failed_by_gate_pattern_loop;			//����������J��Ԃ��ăV�~�����[�V�������I��������
	int m_failed_by_no_gate_pattern;			//���e�p�^�[��������ꂸ�ɃV�~�����[�V�������I��������
	int m_gate_pattern_generate_sum;			//�S�V�~�����[�V�����ŏo�͂��ꂽ���e�p�^�[���̑���

	int m_distance_move_Y_sum;					//�S�V�~�����[�V�����Ői�񂾋���
	int m_distance_move_Y_max;
	int m_distance_move_Y_min;
	
	double m_gate_parttern_generate_time_sum;	//�S�V�~�����[�V�����ŕ��e�p�^�[�������ɂ����������Ԃ̑��a[s]
	double m_gate_parttern_generate_time_max;
	double m_gate_parttern_generate_time_min;
};
