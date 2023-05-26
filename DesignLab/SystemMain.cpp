#include "SystemMain.h"
#include <boost/thread.hpp>
#include <boost/timer/timer.hpp>
#include "Define.h"
#include "GraphSearch.h"
#include "CmdIO.h"
#include "hexapod.h"

SystemMain::SystemMain()
{
	//���{�b�g�̃f�[�^������������D
	Hexapod::makeLegROM_r();

	//�}�b�v�𐶐�����D
	m_Map.init(EMapCreateMode::Flat, MapCreator::OPTION_PERFORATED | MapCreator::OPTION_STEP, false);

	//����l�Ƀ}�b�v��n���D
	m_Broker.setMapState(m_Map);

	//�摜�E�B���h�E��\������N���X�ɒ���l�̃A�h���X��n���āC����������������D
	m_Graphic.init(&m_Broker);

	//���̒T���ł̖ڕW��ݒ肷��D
	m_target.TargetDirection = myvector::VGet(0, 1, 0);					//�ڕW����x,y,z(���i�ړ�)
	m_target.TargetPosition = myvector::VGet(0, 10000, 0);				//�ڕW�ʒu
	m_target.TargetRotation = myvector::VGet(0, 0, 1);					//�ڕW�������P,R,Y(����Y���������l���Ă��Ȃ��l��1������]�A-1���E��])
	m_target.TargetAngle = myvector::VGet(0, 0, Define::MY_PI / 2.0f);	//�ڕW����p�x(���̂̊p�x)
	m_target.RotationCenter = myvector::VGet(-10000, 0, 0);				//��]���Sx,y,z
	m_target.TargetMode = ETargetMode::TURN_DIRECTION;					// ETargetMode�̒�����ڕW�̕]�����@��ݒ肷��D
	m_target.TurningRadius = abs(m_target.RotationCenter.x);
	m_target.TurningRadius = myvector::VMag(m_target.RotationCenter);	//���񔼌a ���ꂾ�ƁA���_�Ɛ��񒆐S�Ƃ̋���,����X�V���ĂȂ����猻��ł͈ꉞ�悳���Ay����𒼐i������Ȃ�A���̋����܂�A���񒆐S��y���Ƃ̋���

}

void SystemMain::main()
{
	//�摜�\���E�B���h�E��ʃX���b�h�ŗ����グ��D�������Ɏ��s������C���������摜�\�������Ȃ��ݒ�ɂȂ��Ă���Ɨ����オ��Ȃ��D
	boost::thread _thread_graphic(&GraphicSystem::main, &m_Graphic);

	CmdIO _cmd;	//�R�}���h���C���ɕ�����`�悷��N���X��p�ӂ���D


	//�V�~�����[�V�������s���񐔕����[�v����D
	for (int i = 0; i < Define::SIMURATE_NUM; i++)
	{
		SNode _current_node;									//���݂̃m�[�h�̏�Ԃ��i�[����ϐ��D
		const bool _do_random_init = (i == 0) ? false : true;	// i �̒l�� 0 �Ȃ�΃����_���ȏꏊ�ɏ������͂��Ȃ��D(i == 0)��]�����āCtrue�Ȃ�ΑO��(false)�Cfalse�Ȃ�Ό��(true)��������D
		initNode(_current_node, _do_random_init);				//�m�[�h�̈ʒu������������D

		if (Define::FLAG_GRAPHIC_AVAILABLE == true) { m_Broker.pushNode(_current_node); }	//�O���t�B�b�N���L���Ȃ�΁C����l�ɍŏ��̃m�[�h�̏�Ԃ�ʒB����D

		_cmd.outputGraphSearchStaretMessage(i + 1);	//�R�}���h���C���ɊJ�n���̃��b�Z�[�W���o�͂���D
		_cmd.outputNode(_current_node, 0);			//�R�}���h���C���ɍŏ��̃m�[�h�̏�Ԃ��o�͂���D


		//�ő���e�����񐔕��܂Ń��[�v����D
		for (int i = 0; i < Define::GATE_PATTERN_GENERATE_NUM; i++)
		{
			GraphSearch _GraphSearch;		//�O���t�T���p�N���X��p�ӂ���D
			SNode _result_node;				//�O���t�T���̌��ʂ��i�[����ϐ��D
			boost::timer::cpu_timer _timer;	//�O���t�T���ɂ����������Ԃ��o�͂��邽�߂̃^�C�}�[


			_timer.start();		//�^�C�}�[�X�^�[�g�D
			bool _is_sucess = _GraphSearch.getNextNodebyGraphSearch(_current_node, &m_Map, m_target, _result_node);		//�O���t�T�����s���D
			_timer.stop();		//�^�C�}�[�X�g�b�v�D


			if (_is_sucess == false) 
			{
				//���̕��e�������ł��Ȃ�������C���[�v��������C���̃V�~�����[�V�����֐i�ށD
				_cmd.outputErrorMessageInGraphSearch("Failed to generate the next gait.");
				break;
			}

			_current_node = _result_node;		//���̕��e�������ł��Ă���Ȃ�΁C�m�[�h���X�V����D

			if (Define::FLAG_GRAPHIC_AVAILABLE == true) { m_Broker.pushNode(_current_node); }	//�O���t�B�b�N���L���Ȃ�Β���l�Ɍ��ʂ�ʒB����D
			_cmd.outputNode(_current_node, i + 1);												//�R�}���h���C���Ɍ��݂̃m�[�h���o�͂���D


			m_Checker.setNode(_current_node);		//����`�F�b�J�[�ɂ��m�[�h��ʒB����D

			if (m_Checker.isLoopMove() == true) 
			{
				//���삪���[�v���Ă��܂��Ă���Ȃ�΁C���[�v��������C���̃V�~�����[�V�����֐i�ށD
				_cmd.outputErrorMessageInGraphSearch("Motion stuck in a loop.");
				break;
			}
		}


	}

	//�摜�\���E�B���h�E�̏I����҂D
	_thread_graphic.join();
}
