#pragma once

#include <doctest.h>

#include "simulation_map_parameter_validator.h"


TEST_SUITE("SimulationMapParameterValidator")
{
	SimulationMapParameterValidator validator;

	TEST_CASE("Validate_�z�[������0�����̎�_false���Ԃ�ׂ�")
	{
		SUBCASE("�z�[������-1�̎�_false���Ԃ�ׂ�")
		{
			SimulationMapParameter parameter;
			parameter.hole_rate = -1;
			const auto [result, message] = validator.Validate(parameter);

			CHECK_FALSE(result);
		}

		SUBCASE("�z�[������0�̎�_true���Ԃ�ׂ�")
		{
			SimulationMapParameter parameter;
			parameter.hole_rate = 0;
			const auto [result, message] = validator.Validate(parameter);

			CHECK(result);
		}
	}

	TEST_CASE("Validate_�z�[������100���傫����_false���Ԃ�ׂ�")
	{
		SUBCASE("�z�[������101�̎�_false���Ԃ�ׂ�")
		{
			SimulationMapParameter parameter;
			parameter.hole_rate = 101;
			const auto [result, message] = validator.Validate(parameter);

			CHECK_FALSE(result);
		}

		SUBCASE("�z�[������100�̎�_true���Ԃ�ׂ�")
		{
			SimulationMapParameter parameter;
			parameter.hole_rate = 100;
			const auto [result, message] = validator.Validate(parameter);

			CHECK(result);
		}
	}
}
