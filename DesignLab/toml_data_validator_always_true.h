//! @file toml_data_validator_always_true.h
//! @brief ���true��Ԃ�ITomlDataValidator�̎����N���X�D

#ifndef DESIGNLAB_TOML_DATA_VALIDATOR_ALWAYS_TRUE_H
#define DESIGNLAB_TOML_DATA_VALIDATOR_ALWAYS_TRUE_H

#include "cmdio_util.h"
#include "interface_toml_data_validator.h"


template <typename T>
class TomlDataValidatorAlwaysTrue final : public ITomlDataValidator<T>
{
public:
	std::tuple<bool, std::string> Validate([[maybe_unused]] const T& toml_data) const override 
	{
		::designlab::cmdio::Output("(���݂̐ݒ�ł͌��؂͍s���܂���D)", OutputDetail::kSystem);
		return { true ,"" };
	}

};


#endif // !DESIGNLAB_TOML_DATA_VALIDATOR_ALWAYS_TRUE_H