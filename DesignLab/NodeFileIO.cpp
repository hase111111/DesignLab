#include "NodeFileIO.h"

NodeFileIO::NodeFileIO()
{
	m_file_name = "last_node_num.csv";
}

bool NodeFileIO::outputNodeFile(const std::vector<int> _node)
{
	//�t�@�C�����J��
	std::ofstream _node_file;
	_node_file.open(m_file_name);

	//�J���Ȃ����false
	if (!_node_file) { return false; }

	//�o�͂���
	for (auto &i : _node) 
	{
		_node_file << i << std::endl;
	}

	//�t�@�C�������
	_node_file.close();	

	return true;
}
