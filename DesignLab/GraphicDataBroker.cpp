#include "GraphicDataBroker.h"

void GraphicDataBroker::setMapState(const MapState& _map)
{
    //�������ݗp�̃��b�N��������D�܂��́Cupgrade_lock��p�ӂ��āC�����unique_lock�ɕύX����D�Q�l https://iorate.hatenablog.com/entry/20130222/1361538198
    boost::upgrade_lock<boost::shared_mutex> upgrade_lock(m_mtx);

    {
        boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(upgrade_lock);

        //�l���Z�b�g����D
        m_Map = _map;
    }
}

MapState GraphicDataBroker::getMapState() const
{
    //�ǂݎ��p�̃��b�N��������D���̃X�R�[�v { } �𔲂���܂Ń��b�N��������D(�܂肱�̊֐����I���܂�)
    boost::shared_lock<boost::shared_mutex> read_lock(m_mtx);

    return m_Map;
}

void GraphicDataBroker::pushNode(const SNode _node)
{
    //�������ݗp�̃��b�N��������D
    boost::upgrade_lock<boost::shared_mutex> upgrade_lock(m_mtx);

    {
        boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(upgrade_lock);

        //�l��push����D
        m_node.push_back(_node);
    }
}

void GraphicDataBroker::copyNode(std::vector<SNode>& _node_vec) const
{
    //�ǂݎ��p�̃��b�N��������D
    boost::shared_lock<boost::shared_mutex> read_lock(m_mtx);

    _node_vec.clear();

    for (auto &i : m_node)
    {
        _node_vec.push_back(i);
    }
}

void GraphicDataBroker::copyOnlyNewNode(std::vector<SNode>& _node_vec) const
{
    //�ǂݎ��p�̃��b�N��������D
    boost::shared_lock<boost::shared_mutex> read_lock(m_mtx);

    if (_node_vec.size() < m_node.size()) 
    {
        const size_t start_num = _node_vec.size();
        const size_t loop_num = m_node.size() - _node_vec.size();

        for (size_t i = start_num; i < loop_num; i++)
        {
            _node_vec.push_back(m_node.at(i));
        }
    }
}
