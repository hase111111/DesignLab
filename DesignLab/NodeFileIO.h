#pragma once
#include <string>
#include <fstream>
#include <vector>

class NodeFileIO final
{
public:
	NodeFileIO();
	~NodeFileIO() = default;

	//�m�[�h�̐��̓�����vector���t�@�C���ɏo�͂���
	bool outputNodeFile(const std::vector<int> _node);

private:
	std::string m_file_name;
};
