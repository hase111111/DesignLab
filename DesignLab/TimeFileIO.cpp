#include "TimeFileIO.h"

TimeFileIO::TimeFileIO()
{
	m_file_name = "time.csv";
}

bool TimeFileIO::outputTimeFile(const std::vector<double> _time)
{
	//�t�@�C�����J��
	std::ofstream _time_file;
	_time_file.open(m_file_name);

	//�J���Ȃ���ΏI��
	if (!_time_file) { return false; }

	for (auto &i : _time)
	{
		_time_file << i << std::endl;
	}

	//�t�@�C�������
	_time_file.close();

    return true;
}
