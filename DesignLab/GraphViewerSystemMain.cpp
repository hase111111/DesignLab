#include "GraphViewerSystemMain.h"
#include "GraphicMainGraphViewer.h"
#include "GraphTreeCreatorHato.h"
#include "HexapodStateCalculator.h"
#include <iostream>
#include <boost/thread.hpp>
#include "Define.h"
#include "MyTimer.h"
#include "StringToValue.h"

//��x�ƒǋL���Ȃ����낤�ƑS�Ă��ׂ��������Ă��܂��D
//�߂��Ⴍ����ǂ݂Â炢���낤�Ǝv�����ǁC�����āD

using StrtoVal::StrToInt;

GraphViewerSystemMain::GraphViewerSystemMain()
{
	std::cout << "--------------------------------------------------" << std::endl;
	std::cout << std::endl;
	std::cout << "     GraphViewerSystemMain : GraphViewer�N��" << std::endl;
	std::cout << std::endl;
	std::cout << "--------------------------------------------------" << std::endl;
	std::cout << std::endl;

	//���{�b�g�̃f�[�^������������D
	HexapodStateCalclator::initLegR();

	//�}�b�v�𐶐�����
	std::cout << "GraphViewerSystemMain : �}�b�v�𐶐����܂��D" << std::endl;
	std::cout << "GraphViewerSystemMain : �I�v�V��������͂��Ă�������" << std::endl;
	MapCreator::printAllMapCreateMode();
	std::string _mode;
	std::cout << std::endl << "input : ";
	std::cin >> _mode;
	std::cout << std::endl;

	MapCreator::printAllMapCreateOption();
	std::string _option;
	std::cout << std::endl << "input : ";
	std::cin >> _option;
	std::cout << std::endl;
	m_MapState.init(static_cast<EMapCreateMode>(StrToInt(_mode)), StrToInt(_option), false);
	std::cout << "MapCreator : �}�b�v�𐶐����܂����D" << std::endl << std::endl;

	//����l������������
	std::cout << "GraphicDataBroker : ����l�����������܂��D" << std::endl << std::endl;
	m_GraphicDataBroker.setMapState(m_MapState);

	//�O���t�B�b�N�V�X�e��������������
	std::cout << "GraphicSystem : �O���t�B�b�N�V�X�e�������������܂��D" << std::endl << std::endl;
	m_GraphicSystem.init(std::make_unique<GraphicMainGraphViewer>(&m_GraphicDataBroker));

	//�O���t�؍쐬�N���X������������
	std::cout << "GraphCreator : �O���t�؍쐬�N���X�����������܂��D" << std::endl << std::endl;
	mp_GraphTreeCreator = std::make_unique<GraphTreeCreatorHato>();

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
		std::cout << "--------------------------------------------------" << std::endl;
		std::cout << std::endl;
		showGraphStatus(_graph);

		if (_graph.size() == 0)
		{
			std::cout << "GraphViewerSystemMain : �܂��O���t�𐶐����Ă��܂���" << std::endl;

			if (askYesNo("GraphViewerSystemMain : �O���t���쐬���܂����H"))
			{
				if (!mp_GraphTreeCreator)
				{
					std::cout << "GraphViewerSystemMain : �O���t�؍쐬�N���X������������Ă��܂���" << std::endl;
					std::cout << "GraphViewerSystemMain : �v���O�������I�����܂�" << std::endl;
					break;
				}

				std::cout << "IGraphTreeCreator : �O���t���쐬���܂�" << std::endl;

				MyTimer _timer;
				_timer.start();
				createGraph(_node, _graph);
				_timer.end();
				setGraphToBroker(_graph);
				std::cout << "IGraphTreeCreator : �O���t���쐬���܂���" << std::endl;
				std::cout << "IGraphTreeCreator : �O���t�쐬�ɂ����������� : " << _timer.getMilliSecond() << " [ms]" << std::endl;
				std::cout << std::endl;
			}
			else
			{
				//�I�����邩���₷��
				if (askYesNo("GraphViewerSystemMain : �I�����܂����H")) { break; }
			}
		}
		else
		{
			std::cout << "GraphViewerSystemMain : �O���t�𑀍삵�܂�" << std::endl;

			//���상�j���[��\������
			std::cout << "GraphViewerSystemMain : ���상�j���[��\�����܂�" << std::endl;
			std::cout << "GraphViewerSystemMain : 1 : �m�[�h�I�����C���̃m�[�h��e�ɂ��ăO���t�𐶐�����" << std::endl;
			std::cout << "GraphViewerSystemMain : 2 : �m�[�h�I�����ĕ\������" << std::endl;
			std::cout << "GraphViewerSystemMain : 3 : �O���t��S�폜����" << std::endl;
			std::cout << "GraphViewerSystemMain : other : �I������" << std::endl;
			std::cout << std::endl;
			std::cout << "input : ";
			std::string _str;
			std::cin >> _str;
			int _menu = StrToInt(_str);
			std::cout << std::endl;

			if (_menu == 1 || _menu == 2)
			{
				std::cout << "GraphViewerSystemMain : �m�[�h��I�����Ă�������" << std::endl;
				std::cout << "GraphViewerSystemMain : 0 �` " << _graph.size() - 1 << " �̐�������͂��Ă�������" << std::endl;
				std::cout << std::endl;
				std::cout << "input : ";

				std::string _str_node;
				std::cin >> _str_node;
				int _node_num = StrToInt(_str_node);
				std::cout << std::endl;

				if (_node_num < 0 || _node_num >= _graph.size())
				{
					std::cout << "GraphViewerSystemMain : �����ȃm�[�h�ԍ��ł�" << std::endl;
					std::cout << std::endl;
					continue;
				}
				else
				{
					if (_menu == 1)
					{
						std::cout << "--------------------------------------------------" << std::endl;
						std::cout << "GraphViewerSystemMain : �m�[�h��I�����C���̃m�[�h��e�ɂ��ăO���t�𐶐����܂�" << std::endl;
						std::cout << std::endl;
						std::cout << _graph[_node_num];
						std::cout << std::endl;
						std::cout << "IGraphTreeCreator : �O���t���쐬���܂�" << std::endl;

						MyTimer _timer;
						_timer.start();
						createGraph(_graph[_node_num], _graph);
						_timer.end();
						setGraphToBroker(_graph);
						std::cout << "IGraphTreeCreator : �O���t���쐬���܂���" << std::endl;
						std::cout << "IGraphTreeCreator : �O���t�쐬�ɂ����������� : " << _timer.getMilliSecond() << " [ms]" << std::endl;
						std::cout << std::endl;
					}
					else
					{
						std::cout << "--------------------------------------------------" << std::endl;
						std::cout << "GraphViewerSystemMain : �m�[�h��\�����܂�" << std::endl;
						std::cout << std::endl;
						std::cout << _graph[_node_num];
						std::cout << std::endl;
					}
				}
			}
			else if (_menu == 3)
			{
				m_GraphicDataBroker.deleteAllNode();
				_graph.clear();
				std::cout << "GraphViewerSystemMain : �O���t��S�폜���܂���" << std::endl;
				std::cout << std::endl;
			}
			else
			{
				//�I�����邩���₷��
				if (askYesNo("GraphViewerSystemMain : �I�����܂����H")) { break; }
			}
		}
	}
}

void GraphViewerSystemMain::createGraph(const SNode _parent, std::vector<SNode>& _graph)
{
	SNode _temp_node = _parent;
	int _output_num = 0;

	_temp_node.changeParentNode();

	mp_GraphTreeCreator->createGraphTree(_temp_node, &m_MapState, _graph, _output_num);
}

void GraphViewerSystemMain::setGraphToBroker(const std::vector<SNode>& _graph)
{
	m_GraphicDataBroker.deleteAllNode();

	for (auto& i : _graph)
	{
		m_GraphicDataBroker.pushNode(i);
	}
}

bool GraphViewerSystemMain::askYesNo(const std::string& question) const
{
	std::cout << question << " ( y / n )" << std::endl;
	std::cout << std::endl;
	std::cout << "input : ";
	std::string _input;
	std::cin >> _input;
	std::cout << std::endl;

	if (_input == "y" || _input == "Y" || _input == "yes" || _input == "Yes") { return true; }
	return false;
}

void GraphViewerSystemMain::showGraphStatus(const std::vector<SNode>& _graph) const
{
	std::cout << "GraphViewerSystemMain : �O���t�̏�Ԃ�\�����܂��D" << std::endl;
	std::cout << "GraphViewerSystemMain : �O���t�̃m�[�h�� : " << _graph.size() << std::endl;

	if (_graph.size() > 0)
	{
		std::vector<int> _depth_num((size_t)Define::GRAPH_SEARCH_DEPTH + 1);

		std::cout << "GraphViewerSystemMain : �O���t�T���̍ő�[�� : " << (int)Define::GRAPH_SEARCH_DEPTH << std::endl;

		for (const auto& i : _graph)
		{
			_depth_num.at(static_cast<size_t>(i.depth))++;
		}

		int _cnt = 0;

		for (const auto& i : _depth_num)
		{
			std::cout << "GraphViewerSystemMain : �[��" << _cnt << " : " << i << std::endl;
			_cnt++;
		}
	}

	std::cout << std::endl;
}
