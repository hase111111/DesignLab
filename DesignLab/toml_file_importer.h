//! @file toml_file_importer.h
//! @brief toml�t�@�C����ǂݍ���ō\���̂ɕϊ�����e���v���[�g�N���X�D

#ifndef DESIGNLAB_TOML_FILE_IMPORTER_H_
#define DESIGNLAB_TOML_FILE_IMPORTER_H_

#include <fstream>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>

#include "cmdio_util.h"
#include "interface_toml_data_validator.h"
#include "toml11_define.h"
#include "toml_data_validator_always_true.h"


template <typename T, typename = void>
struct has_from_toml : std::false_type {};

template <typename T>
struct has_from_toml<T, std::void_t<decltype(toml::from<T>())> > : std::true_type {};


//T�̓f�t�H���g�R���X�g���N�^�������Ă���K�v������D
template <typename T, typename = std::enable_if_t<std::is_default_constructible_v<T> && has_from_toml<T>::value> >
class TomlFileImporter final
{
public:

	TomlFileImporter() : validator_(std::make_unique<TomlDataValidatorAlwaysTrue<T>>()) {}

	TomlFileImporter(std::unique_ptr<ITomlDataValidator<T>>&& validator) : validator_(std::move(validator)) {}


	std::optional<T> Import(const std::string& file_path) const
	{
		if (do_output_message_)
		{
			const std::string type_name = typeid(*this).name();
			::designlab::cmdio::OutputNewLine(1, OutputDetail::kSystem);
			::designlab::cmdio::Output("[" + type_name + "]", OutputDetail::kSystem);
			::designlab::cmdio::Output("�t�@�C����ǂݍ��݂܂��Dfile_path : " + file_path, OutputDetail::kSystem);
		}

		if (!std::filesystem::exists(file_path))
		{
			::designlab::cmdio::Output("�t�@�C�������݂��܂���D", OutputDetail::kSystem);
			::designlab::cmdio::OutputNewLine(1, OutputDetail::kSystem);
			return std::nullopt;
		}

		if (do_output_message_) { ::designlab::cmdio::Output("�ݒ�t�@�C����������܂����D�p�[�X���J�n���܂��D", OutputDetail::kSystem); }

		toml::value toml_value;

		try
		{
			std::ifstream ifs(file_path, std::ios::binary);		//�o�C�i�����[�h�œǂݍ���

			toml_value = toml::parse(ifs, file_path);
		}
		catch (toml::syntax_error err)
		{
			if (do_output_message_)
			{
				::designlab::cmdio::Output("�ݒ�t�@�C���̃p�[�X�Ɏ��s���܂����D", OutputDetail::kSystem);
				::designlab::cmdio::OutputNewLine(1, OutputDetail::kSystem);
				::designlab::cmdio::Output("<�p�[�X�Ɏ��s�����ӏ�>", OutputDetail::kSystem);
				::designlab::cmdio::Output(err.what(), OutputDetail::kSystem);
				::designlab::cmdio::OutputNewLine(1, OutputDetail::kSystem);
			}

			return std::nullopt;
		}

		if (do_output_message_) { ::designlab::cmdio::Output("�ݒ�t�@�C���̃p�[�X�ɐ������܂����D�f�[�^���V���A���C�Y���܂��D", OutputDetail::kSystem); }

		T data;

		try
		{
			data = toml::from<T>::from_toml(toml_value);
		}
		catch (...)
		{
			if (do_output_message_) { ::designlab::cmdio::Output("�f�[�^�̃V���A���C�Y�Ɏ��s���܂����D", OutputDetail::kSystem); }

			return std::nullopt;
		}

		if (do_output_message_) { ::designlab::cmdio::Output("�f�[�^�̃V���A���C�Y�ɐ������܂����D�f�[�^�̌��؂��J�n���܂��D", OutputDetail::kSystem); }

		const auto [is_valid, error_message] = validator_->Validate(data);

		if (!is_valid)
		{
			if (do_output_message_)
			{
				::designlab::cmdio::Output("�f�[�^�̌��؂Ɏ��s���܂����D", OutputDetail::kSystem);
				::designlab::cmdio::OutputNewLine(1, OutputDetail::kSystem);
				::designlab::cmdio::Output("<���؂Ɏ��s�������R>", OutputDetail::kSystem);
				::designlab::cmdio::Output(error_message, OutputDetail::kSystem);
				::designlab::cmdio::OutputNewLine(1, OutputDetail::kSystem);
			}

			return std::nullopt;
		}

		if (do_output_message_) { ::designlab::cmdio::Output("�f�[�^�̌��؂ɐ������܂����D", OutputDetail::kSystem); }
		if (do_output_message_) { ::designlab::cmdio::Output("�ǂݍ��݂͐���Ɋ������܂����D", OutputDetail::kSystem); }

		return data;
	}

private:

	bool do_output_message_{ true };

	const std::unique_ptr<ITomlDataValidator<T>> validator_;
};


#endif // DESIGNLAB_TOML_FILE_IMPORTER_H_