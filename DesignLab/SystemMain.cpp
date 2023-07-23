#include "SystemMain.h"
#include <boost/thread.hpp>
#include <boost/timer/timer.hpp>
#include "Define.h"
#include "MyMath.h"
#include "CmdIO.h"
#include "hexapod.h"
#include "HexapodStateCalculator.h"
#include "NodeValidityChecker.h"
#include "GraphicMainBasic.h"
#include "GraphicMainTest.h"

SystemMain::SystemMain(std::unique_ptr<IGraphSearch>&& _graph_search)
{
	//���{�b�g�̃f�[�^������������D
	Hexapod::makeLegROM_r();
	HexapodStateCalclator::initLegR();

	//�}�b�v�𐶐�����D
	m_Map.init(EMapCreateMode::Flat, MapCreator::OPTION_NONE, true);

	//����l�Ƀ}�b�v��n���D
	m_Broker.setMapState(m_Map);

	//�O���t�T���N���X���Z�b�g����
	m_GraphSearch = std::move(_graph_search);

	//�摜�E�B���h�E��\������N���X�ɒ���l�̃A�h���X��n���āC����������������D
	m_Graphic.init(std::make_unique<GraphicMainBasic>(&m_Broker));

	//���̒T���ł̖ڕW��ݒ肷��D
	m_target.TargetMode = ETargetMode::StraightPosition;
	m_target.TargetPosition = { 100000,0,0 };
}

void SystemMain::main()
{
	if (!m_GraphSearch)
	{
		//�O���t�T���N���X���Z�b�g����Ă��Ȃ��ꍇ�́C�G���[���o�͂��ďI������D
		std::cout << "GraphSearch is not set." << std::endl;
		return;
	}

	CmdIO _cmd;	//�R�}���h���C���ɕ�����`�悷��N���X��p�ӂ���D

	NodeValidityChecker _node_checker;	//�m�[�h�̑Ó������`�F�b�N����N���X��p�ӂ���D

	//�摜�\���E�B���h�E��ʃX���b�h�ŗ����グ��D�������Ɏ��s������C���������摜�\�������Ȃ��ݒ�ɂȂ��Ă���Ɨ����オ��Ȃ��D
	boost::thread _thread_graphic(&GraphicSystem::main, &m_Graphic);

	//�V�~�����[�V�������s���񐔕����[�v����D
	for (int i = 0; i < Define::SIMURATE_NUM; i++)
	{
		SNode _current_node;									//���݂̃m�[�h�̏�Ԃ��i�[����ϐ��D
		const bool _do_random_init = (i == 0) ? false : true;	// i �̒l�� 0 �Ȃ�΃����_���ȏꏊ�ɏ������͂��Ȃ��D(i == 0)��]�����āCtrue�Ȃ�ΑO��(false)�Cfalse�Ȃ�Ό��(true)��������D
		_current_node.init(_do_random_init);

		if (Define::FLAG_GRAPHIC_AVAILABLE == true) { m_Broker.pushNode(_current_node); }	//�O���t�B�b�N���L���Ȃ�΁C����l�ɍŏ��̃m�[�h�̏�Ԃ�ʒB����D

		_cmd.outputGraphSearchStaretMessage(i + 1);	//�R�}���h���C���ɊJ�n���̃��b�Z�[�W���o�͂���D
		_cmd.outputNode(_current_node, 0);			//�R�}���h���C���ɍŏ��̃m�[�h�̏�Ԃ��o�͂���D


		//�ő���e�����񐔕��܂Ń��[�v����D
		for (int i = 0; i < Define::GATE_PATTERN_GENERATE_NUM; i++)
		{
			SNode _result_node;				//�O���t�T���̌��ʂ��i�[����ϐ��D
			boost::timer::cpu_timer _timer;	//�O���t�T���ɂ����������Ԃ��o�͂��邽�߂̃^�C�}�[

			_timer.start();		//�^�C�}�[�X�^�[�g�D
			EGraphSearchResult _result_state = m_GraphSearch->getNextNodebyGraphSearch(_current_node, &m_Map, m_target, _result_node);		//�O���t�T�����s���D
			_timer.stop();		//�^�C�}�[�X�g�b�v�D


			if (graphSeachResultIsSuccessful(_result_state) == false)
			{
				//���̕��e�������ł��Ȃ�������C���[�v��������C���̃V�~�����[�V�����֐i�ށD
				_cmd.outputErrorMessageInGraphSearch("Failed to generate the next gait.");
				break;
			}

			_current_node = _result_node;		//���̕��e�������ł��Ă���Ȃ�΁C�m�[�h���X�V����D

			if (Define::FLAG_GRAPHIC_AVAILABLE == true) { m_Broker.pushNode(_current_node); }	//�O���t�B�b�N���L���Ȃ�Β���l�Ɍ��ʂ�ʒB����D
			_cmd.outputNode(_current_node, i + 1);												//�R�}���h���C���Ɍ��݂̃m�[�h���o�͂���D


			_node_checker.setNode(_current_node);		//����`�F�b�J�[�ɂ��m�[�h��ʒB����D

			if (_node_checker.isLoopMove() == true)
			{
				//���삪���[�v���Ă��܂��Ă���Ȃ�΁C���[�v��������C���̃V�~�����[�V�����֐i�ށD
				_cmd.outputErrorMessageInGraphSearch("Motion stuck in a loop.");
				break;
			}
		}


	}

	//�摜�\���E�B���h�E�̏I����҂D
	std::cout << "Waiting for dxlib to finish." << std::endl;
	_thread_graphic.join();
}
