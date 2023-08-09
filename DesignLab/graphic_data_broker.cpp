#include "graphic_data_broker.h"


void GraphicDataBroker::setMapState(const MapState& map)
{
	//�������ݗp�̃��b�N��������D�܂��́Cupgrade_lock��p�ӂ��āC�����unique_lock�ɕύX����D
	boost::upgrade_lock<boost::shared_mutex> upgrade_lock(m_mtx);

	{
		boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(upgrade_lock);

		//�l���Z�b�g����D
		m_Map = map;
	}
}


MapState GraphicDataBroker::getMapState() const
{
	//�ǂݎ��p�̃��b�N��������D���̃X�R�[�v { } �𔲂���܂Ń��b�N��������D(�܂肱�̊֐����I���܂�)
	boost::shared_lock<boost::shared_mutex> read_lock(m_mtx);

	return m_Map;
}


void GraphicDataBroker::pushNode(const SNode& node)
{
	//�������ݗp�̃��b�N��������D
	boost::upgrade_lock<boost::shared_mutex> upgrade_lock(m_mtx);

	{
		boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(upgrade_lock);

		//�l��push����D
		m_node.push_back(node);
	}
}


void GraphicDataBroker::copyAllNode(std::vector<SNode>* node_vec) const
{
	//�ǂݎ��p�̃��b�N��������D
	boost::shared_lock<boost::shared_mutex> read_lock(m_mtx);

	(*node_vec).clear();

	for (auto& i : m_node)
	{
		(*node_vec).push_back(i);
	}
}


void GraphicDataBroker::copyOnlyNewNode(std::vector<SNode>* node_vec) const
{
	//�ǂݎ��p�̃��b�N��������D
	boost::shared_lock<boost::shared_mutex> read_lock(m_mtx);

	if ((*node_vec).size() < m_node.size())
	{
		const size_t start_num = (*node_vec).size();

		for (size_t i = start_num; i < m_node.size(); ++i)
		{
			(*node_vec).push_back(m_node.at(i));
		}
	}
}


size_t GraphicDataBroker::getNodeNum() const
{
	//�ǂݎ��p�̃��b�N��������D
	boost::shared_lock<boost::shared_mutex> read_lock(m_mtx);

	return m_node.size();
}


void GraphicDataBroker::deleteAllNode()
{
	//�������ݗp�̃��b�N��������D
	boost::upgrade_lock<boost::shared_mutex> upgrade_lock(m_mtx);

	{
		boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(upgrade_lock);

		//�l��push����D
		m_node.clear();
	}
}
