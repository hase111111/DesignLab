#pragma once
#include <string>
#include <fstream>
#include <vector>

class TimeFileIO final 
{
public:
	TimeFileIO();
	~TimeFileIO() = default;

	//�v���ɂ����������Ԃ̓�����vector���t�@�C���ɏo�͂���
	bool outputTimeFile(const std::vector<double> _time);

private:
	std::string m_file_name;
};
