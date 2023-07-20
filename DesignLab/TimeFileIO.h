#pragma once
#include <string>
#include <fstream>
#include <vector>

class TimeFileIO final 
{
public:
	TimeFileIO();
	~TimeFileIO() = default;

	//! @brief �v���ɂ����������Ԃ̓�����vector���t�@�C���ɏo�͂���
	//! @param [in] _time �v���ɂ����������Ԃ̓�����vector
	//! @return bool true:�o�͐��� false:�o�͎��s
	bool outputTimeFile(const std::vector<double> _time);

private:
	std::string m_file_name;
};


//! @file TimeFileIO.h
//! @brief �v���ɂ����������Ԃ��t�@�C���ɏo�͂���N���X�̃w�b�_�t�@�C��
//! @date 2022/06/17
//! @auther ���J��

//! @class TimeFileIO
//! @brief �v���ɂ����������Ԃ��t�@�C���ɏo�͂���N���X
//! @date 2022/06/17
//! @auther ���J��
