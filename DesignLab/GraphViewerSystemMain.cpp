#include "GraphViewerSystemMain.h"
#include "HexapodStateCalculator.h"
#include <iostream>
#include <boost/thread.hpp>


//��x�ƒǋL���Ȃ����낤�ƑS�Ă��ׂ��������Ă��܂��D
//�߂��Ⴍ����ǂ݂Â炢���낤�Ǝv�����ǁC�����āD


GraphViewerSystemMain::GraphViewerSystemMain()
{
	std::cout << "--------------------------------------------------" << std::endl;
	std::cout << std::endl;
	std::cout << "GraphViewerSystemMain : GraphViewer�N��" << std::endl;
	std::cout << std::endl;
	std::cout << "--------------------------------------------------" << std::endl;
	std::cout << std::endl;

	//���{�b�g�̃f�[�^������������D
	HexapodStateCalclator::initLegR();

	//�}�b�v�𐶐�����
	std::cout << "GraphViewerSystemMain : �}�b�v�𐶐����܂��D" << std::endl;
	std::cout << "GraphViewerSystemMain : �I�v�V��������͂��Ă�������" << std::endl;
	MapCreator::printAllMapCreateMode();
	int _mode = 0;
	std::cout << std::endl << "input : ";
	std::cin >> _mode;
	std::cout << std::endl;

	MapCreator::printAllMapCreateOption();
	int _option = 0;
	std::cout << std::endl << "input : ";
	std::cin >> _option;
	std::cout << std::endl;
	m_MapState.init(static_cast<EMapCreateMode>(_mode), _option, false);
	std::cout << "MapCreator : �}�b�v�𐶐����܂����D" << std::endl << std::endl;

	//����l������������
	std::cout << "GraphicDataBroker : ����l�����������܂��D" << std::endl << std::endl;
	m_GraphicDataBroker.setMapState(m_MapState);

	//�O���t�B�b�N�V�X�e��������������
	std::cout << "GraphicSystem : �O���t�B�b�N�V�X�e�������������܂��D" << std::endl << std::endl;
	m_GraphicSystem.init(&m_GraphicDataBroker);

	//�������I��
	std::cout << "GraphViewerSystemMain : GraphViewer�������I���D�N�����܂�" << std::endl << std::endl;

	std::cout << "--------------------------------------------------" << std::endl;
	std::cout << std::endl;
	std::cout << "                   GraphViewer" << std::endl;
	std::cout << std::endl;
	std::cout << "--------------------------------------------------" << std::endl;
	std::cout << std::endl;
}


void GraphViewerSystemMain::main()
{
	//�O���t�B�b�N�V�X�e�����N������
	std::cout << "GraphicSystem : �ʃX���b�h�ŃO���t�B�b�N�V�X�e�����N�����܂��D" << std::endl << std::endl;
	boost::thread _thread_graphic(&GraphicSystem::main, &m_GraphicSystem);

	//�m�[�h������������
	std::cout << "GraphViewerSystemMain : �m�[�h�����������܂��D" << std::endl << std::endl;
	SNode _node;
	_node.init(false);
	std::cout << _node;
	std::cout << std::endl;

	std::vector<SNode> _graph;

	while (true)
	{
		showGraphStatus(_graph);

		if (_graph.size() == 0)
		{
			std::cout << "GraphViewerSystemMain : �܂��O���t�𐶐����Ă��܂���" << std::endl;
			if (askYesNo("GraphViewerSystemMain : �O���t���쐬���܂����H")) { break; }
		}
		else
		{
			std::cout << "GraphViewerSystemMain : �O���t�𑀍삵�܂�" << std::endl;
		}

		//�I�����邩���₷��
		if (askYesNo("GraphViewerSystemMain : �I�����܂����H")) { break; }
	}
}

bool GraphViewerSystemMain::askYesNo(const std::string& question) const
{
	std::cout << question << " ( y / n )" << std::endl;
	std::cout << "input : ";
	char _input;
	std::cin >> _input;
	std::cout << std::endl;

	if (_input == 'y') { return true; }
	return false;
}

void GraphViewerSystemMain::showGraphStatus(const std::vector<SNode>& _graph) const
{
	std::cout << "GraphViewerSystemMain : �O���t�̏�Ԃ�\�����܂��D" << std::endl;
	std::cout << "GraphViewerSystemMain : �O���t�̃m�[�h�� : " << _graph.size() << std::endl;
	std::cout << std::endl;
}
