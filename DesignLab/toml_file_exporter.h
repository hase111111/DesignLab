//! @file toml_file_exporter.h
//! @brief TOML�t�@�C�����o�͂���e���v���[�g�N���X�D

#ifndef DESIGNLAB_TOML_FILE_EXPORTER_H_
#define DESIGNLAB_TOML_FILE_EXPORTER_H_

#include <fstream>
#include <filesystem>
#include <map>

#include "cmdio_util.h"
#include "toml11_define.h"


template <typename T, typename = void>
struct has_into_toml : std::false_type {};

template <typename T>
struct has_into_toml<T, std::void_t<decltype(toml::into<T>())> > : std::true_type {};


template <typename T, typename = std::enable_if_t<std::is_default_constructible_v<T>&& has_into_toml<T>::value> >
class TomlFileExporter final
{
public:

	void Export(const std::string& file_path, const T& data)
	{
		const toml::basic_value<toml::preserve_comments, std::map> value(data);
		const std::string res_str = toml::format(value);	// �ݒ�𕶎���ɕϊ�

		std::ofstream ofs;
		ofs.open(file_path);

		// �t�@�C�����J���Ȃ������牽�����Ȃ�
		if (!ofs)
		{
			::designlab::cmdio::Output("�ݒ�t�@�C���̏o�͂Ɏ��s���܂����Dfile_path : " + file_path, OutputDetail::kSystem);
			return;
		}

		ofs.write(res_str.c_str(), res_str.length());	// �t�@�C���ɏ�������

		ofs.close();	// �t�@�C�������

		::designlab::cmdio::Output("�ݒ�t�@�C�����o�͂��܂����Dfile_path : " + file_path, OutputDetail::kSystem);
	}
};


#endif // DESIGNLAB_TOML_FILE_EXPORTER_H_