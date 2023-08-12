#pragma once
#include <string>
#include <fstream>
#include <vector>

class NodeFileIO final
{
public:
	NodeFileIO();
	~NodeFileIO() = default;

	//! @brief �m�[�h�̐��̓�����vector���t�@�C���ɏo�͂���
	//! @param [in] _node �m�[�h�̐��̓�����vector
	//! @return bool true:�o�͐��� false:�o�͎��s
	bool outputNodeFile(const std::vector<int> _node);

private:
	std::string m_file_name;
};


//! @file NodeFileIO.h
//! @brief �m�[�h�̐����t�@�C���ɏo�͂���N���X�̃w�b�_�t�@�C��
//! @date 2022/06/17
//! @author ���J��

//! @class NodeFileIO
//! @brief �m�[�h�̐����t�@�C���ɏo�͂���N���X
//! @date 2022/06/17
//! @author ���J��
