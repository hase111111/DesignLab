﻿#include "map_file_exporter.h"

#include <fstream>

#include "cmdio_util.h"

namespace dle = ::designlab::enums;
namespace dlio = ::designlab::cmdio;


bool MapFileExporter::ExportMap(const std::string& file_path, const MapState& map_state) const noexcept
{
	// ファイルを開く
	std::ofstream ofs(file_path);

	// ファイルが開けないならばfalseを返す．
	if (not ofs.is_open())
	{
		dlio::Output("ファイルを開けませんでした．", dle::OutputDetail::kError);

		return false;
	}

	// ファイルを1行ずつ書き込む．
	const size_t map_point_size = map_state.GetMapPointSize();

	for (size_t i = 0; i < map_point_size; ++i)
	{
		ofs << map_state.GetMapPoint(i) << std::endl;
	}

	return true;
}
