﻿#include "simulation_setting_importer.h"

#include <fstream>
#include <filesystem>

#include "cmdio_util.h"

namespace dlio = ::designlab::cmdio;


const std::string SimulationSettingImporter::kFilePath = "./simulation_condition/";


SimulationSettingRecord SimulationSettingImporter::ImportOrUseAndOutputDefault() const
{
	dlio::Output("シミュレーションの設定ファイルを読み込みます．", OutputDetail::kInfo);

	//ファイルを探す，存在しなかったらデフォルトの設定を出力して終了，fsystemはC++17から，実行できない場合は設定を見直してみてください
	if (!std::filesystem::is_regular_file(kFilePath + kFileName))
	{
		dlio::Output(kFilePath + kFileName + "が見つかりませんでした．デフォルトの設定を出力し，そのデータを使用します．", OutputDetail::kWarning);
		OutputDefault();
		return SimulationSettingRecord();
	}

	//ファイルを開く
	toml::value data;

	try
	{
		std::ifstream ifs(kFilePath + kFileName, std::ios::binary);		//バイナリモードで読み込む

		data = toml::parse(ifs, kFilePath + kFileName);					//ファイルをパース(読みこみ&解析)する
	}
	catch (toml::syntax_error err)
	{
		dlio::Output("設定ファイルのパースに失敗しました．デフォルトの設定ファイルを出力し，そのデータを使用します．", OutputDetail::kError);
		dlio::Output(err.what(), OutputDetail::kError);

		OutputDefault();
		return SimulationSettingRecord();
	}

	//パースしたデータをSimulationSettingRecordに変換
	SimulationSettingRecord setting;

	try
	{
		setting = toml::from<SimulationSettingRecord>::from_toml(data);
	}
	catch (toml::type_error err)
	{
		dlio::Output("データの変換に失敗しました．デフォルトの設定ファイルを出力し，そのデータを使用します．", OutputDetail::kError);
		dlio::Output(err.what(), OutputDetail::kError);

		OutputDefault();
		return SimulationSettingRecord();
	}

	dlio::Output("シミュレーションの設定ファイルを読み込みました．", OutputDetail::kInfo);

	return setting;
}

void SimulationSettingImporter::OutputDefault() const
{
	const SimulationSettingRecord kDefaultSetting;

	toml::basic_value<toml::preserve_comments, std::map> value(kDefaultSetting);
	std::string res_str = toml::format(value);	// 設定を文字列に変換

	std::ofstream ofs;
	ofs.open(kFilePath + kFileName);

	// ファイルが開けなかったら何もしない
	if (!ofs)
	{
		dlio::Output(kFilePath + kFileName + "を開くことができませんでした．", OutputDetail::kError);
		return;
	}

	ofs.write(res_str.c_str(), res_str.length());	// ファイルに書き込む

	ofs.close();	// ファイルを閉じる

	return;
}
