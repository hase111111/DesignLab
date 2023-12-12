﻿#include "map_creator_factory.h"

#include <memory>

#include "cassert_define.h"
#include "map_creator_by_csv.h"
#include "map_creator_for_simulation.h"
#include "simulation_map_parameter_validator.h"
#include "toml_file_importer.h"
#include "toml_file_setupper.h"

namespace dle = ::designlab::enums;


std::unique_ptr<IMapCreator> MapCreatorFactory::Create(const SimulationSettingRecord& record)
{
	std::unique_ptr<IMapCreator> map_creator;

	if (record.map_create_mode == dle::MapCreateMode::kFromFile)
	{
		map_creator = std::make_unique<MapCreatorByCsv>(TomlFileSetupper::kTomlFileDirPath + record.map_file_name);
	}
	else if (record.map_create_mode == dle::MapCreateMode::kForSimulation)
	{
		auto validator_ptr = std::make_unique<SimulationMapParameterValidator>();

		const TomlFileImporter<SimulationMapParameter> simulation_map_parameter_importer(std::move(validator_ptr));

		const std::string simulation_map_param_file_path = TomlFileSetupper::kTomlFileDirPath + record.simulation_map_param_file_name;

		const auto simulation_map_parameter = simulation_map_parameter_importer.ImportOrUseDefault(simulation_map_param_file_path);

		map_creator = std::make_unique<MapCreatorForSimulation>(simulation_map_parameter);
	}
	else
	{	
		assert(false && "MapCreateMode is not supported.");
	}

	return std::move(map_creator);
}