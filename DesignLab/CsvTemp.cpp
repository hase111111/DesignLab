#include "CsvTemp.h"

#include <fstream>
#include <filesystem>
#include <string>

#include "result_file_importer.h"
#include "interpolated_node_creator.h"
#include "phantomx_state_calculator_hato.h"


void CsvTemp::WriteCsv()
{
	using designlab::leg_func::IsGrounded;
	using designlab::Vector3;

	//ディレクトリを指定する
	//std::filesystem::path path = "./result/シミュレーション_平面";
	//std::filesystem::path path = "./result/シミュレーション_15[deg]";
	std::filesystem::path path = "./result/シミュレーション_-15[deg]";
	//std::filesystem::path path = "./result/シミュレーション_100[mm]";
	//std::filesystem::path path = "./result/シミュレーション_-100[mm]";



	//std::filesystem::path path = "./result/実機試験_平面";
	//std::filesystem::path path = "./result/実機試験_100[mm]";
	//std::filesystem::path path = "./result/実機試験_-100[mm]";
	//std::filesystem::path path = "./result/実機試験_-15[deg]";
	//std::filesystem::path path = "./result/実機試験_15[deg]";

	//ディレクトリが存在しない場合は終了
	if (!std::filesystem::exists(path)) { return; }


	int node_sum = 0;						//ノードの総数
	int faild_node_sum = 0;					//失敗したノードの総数
	int interpolated_faild_node_sum = 0;	//失敗したノードの総数
	int interpolated_faild_lift_node_sum = 0;
	int interpolated_faild_move_node_sum = 0;
	int interpolated_faild_ground_node_sum = 0;

	int success_count = 0;

	std::vector<std::tuple<int, Vector3>> faild_leg_pos;

	//ディレクトリ内のnode_list1.csvからnode_list5.csvまでのファイルを読み込む
	for (int n = 1; n <= 5; n++)
	{
		//ファイル名を指定する
		std::string file_name = "node_list" + std::to_string(n) + ".csv";
		std::string sim_result = "sim_result" + std::to_string(n) + ".csv";

		//sim_resultファイルを開く
		std::ifstream ifs_res(path / sim_result);
		
		//最初の1行を読む
		std::string str = "";
		std::getline(ifs_res, str);
		if (str != "Simulation Result,kSuccess") 
		{
			std::cout << "シミュレーションが失敗しています" << std::endl;
			continue;
		}

		//ファイルを開く
		std::ifstream ifs(path / file_name);

		//ファイルが開けなかった場合は終了
		if (!ifs) 
		{
			std::cout << "ファイルが開けませんでした" << std::endl;
			return; 
		}

		ResultFileImporter result_file_importer;
		std::vector<RobotStateNode> node_list;
		MapState map_state;

		//ファイルからノードリストとマップの状態を読み込む
		if (!result_file_importer.ImportNodeListAndMapState(path.string() + std::string("/") + file_name, &node_list, &map_state)) 
		{
			std::cout << "ファイルの読み込みに失敗しました" << std::endl;
			return; 
		}

		node_sum += (int)node_list.size();

		InterpolatedNodeCreator interpolated_node_creator;
		PhantomXStateCalclator_Hato phantomx_state_calculator(130);
		bool past_node_is_vaild[6] = { true,true,true,true,true,true };

		for (size_t i = 0; i < node_list.size(); i++)
		{
			std::array<HexapodJointState, HexapodConst::kLegNum> joint_state;
			phantomx_state_calculator.CalculateAllJointState(node_list[i], &joint_state);

			bool current_node_is_vaild[6] = { true,true,true,true,true,true };
			bool is_faild = false;

			for (int w = 0; w < 6;w++)
			{
				current_node_is_vaild[w] = phantomx_state_calculator.IsVaildJointStateOneLeg(w, node_list[i], joint_state[w]);

				if (!is_faild && past_node_is_vaild[w] && ! current_node_is_vaild[w])
				{
					faild_node_sum++;
					std::cout << "\t" << w << "脚の" << i << "番目のノードが失敗しました" << std::endl;
					faild_leg_pos.push_back(std::make_tuple(w, node_list[i].leg_pos[w]));
					is_faild = true;
				}
			}

			//past_node_is_vaildを更新するラムダ式
			auto update_past_node_is_vaild = [&past_node_is_vaild, &current_node_is_vaild]()
				{
					for (int w = 0; w < 6; w++)
					{
						past_node_is_vaild[w] = current_node_is_vaild[w];
					}
				};

			if (is_faild) {
				update_past_node_is_vaild();
				continue; 
			}

			//補間するノードを生成する
			
			// i  - 1番目のノードがなければ続行
			if (i == 0) { 
				update_past_node_is_vaild();
				continue; 
			}

			std::vector<RobotStateNode> interpolated_node;

			interpolated_node_creator.CreateInterpolatedNode(node_list[i - 1], node_list[i], &interpolated_node);

			for (int w = 0; w < 6; w++)
			{
				// 過去または現在のノードが失敗している場合は無視して続行
				if (!past_node_is_vaild[w] || !current_node_is_vaild[w]) { continue; }

				//すでに失敗しているなら終了
				if (is_faild) { break; }

				for (size_t j = 0; j < interpolated_node.size(); j++)
				{
					std::array<HexapodJointState, HexapodConst::kLegNum> joint_state_inter;
					phantomx_state_calculator.CalculateAllJointState(interpolated_node[j], &joint_state_inter);

					if (!phantomx_state_calculator.IsVaildJointStateOneLeg(w, interpolated_node[j], joint_state_inter[w]))
					{
						interpolated_faild_node_sum++;

						if (IsGrounded(node_list[i - 1].leg_state, w) && IsGrounded(node_list[i].leg_state, w))
						{
							interpolated_faild_move_node_sum++;

						}
						else if (IsGrounded(node_list[i - 1].leg_state, w))
						{
							interpolated_faild_lift_node_sum++;
						}
						else if (IsGrounded(node_list[i].leg_state, w))
						{
							interpolated_faild_ground_node_sum++;
						}

						is_faild = true;
						std::cout << "\t" << w << "脚の" << i << "番目のノードが補完に失敗しました" << std::endl;
						break;
					}
				}
			}

			update_past_node_is_vaild();
		}

		//ファイルを閉じる
		ifs.close();
		ifs_res.close();

		success_count++;

		std::cout << "node_list" << n << ".csvの読み込みが完了しました" << std::endl;
	}

	//結果の出力
	std::cout << std::endl;
	std::cout << "成功したシミュレーションの数 : " << success_count << std::endl;
	std::cout << "ノードの総数 : " << node_sum << std::endl;
	std::cout << "失敗したノードの総数 : " << faild_node_sum + interpolated_faild_node_sum  << std::endl;
	std::cout << "通常時に失敗したノードの総数 : " << faild_node_sum << std::endl;
	std::cout << "補間に失敗したノードの総数 : " << interpolated_faild_node_sum << std::endl;
	std::cout << "補間に失敗したノードのうち，移動中に失敗したノードの総数 : " << interpolated_faild_move_node_sum << std::endl;
	std::cout << "補間に失敗したノードのうち，持ち上げ中に失敗したノードの総数 : " << interpolated_faild_lift_node_sum << std::endl;
	std::cout << "補間に失敗したノードのうち，着地中に失敗したノードの総数 : " << interpolated_faild_ground_node_sum << std::endl;
	std::cout << std::endl;	

	//結果を * 5 / success_count して出力
	if (success_count != 5) 
	{
		std::cout << "成功したシミュレーションの数 : " << success_count << std::endl;
		std::cout << "ノードの総数 : " << node_sum * 5 / success_count << std::endl;
		std::cout << "失敗したノードの総数 : " << (faild_node_sum + interpolated_faild_node_sum) * 5 / success_count << std::endl;
		std::cout << "通常時に失敗したノードの総数 : " << faild_node_sum * 5 / success_count << std::endl;
		std::cout << "補間に失敗したノードの総数 : " << interpolated_faild_node_sum * 5 / success_count << std::endl;
		std::cout << "補間に失敗したノードのうち，移動中に失敗したノードの総数 : " << interpolated_faild_move_node_sum * 5 / success_count << std::endl;
		std::cout << "補間に失敗したノードのうち，持ち上げ中に失敗したノードの総数 : " << interpolated_faild_lift_node_sum * 5 / success_count << std::endl;
		std::cout << "補間に失敗したノードのうち，着地中に失敗したノードの総数 : " << interpolated_faild_ground_node_sum * 5 / success_count << std::endl;
		std::cout << std::endl;
	}

	//失敗したノードの座標を出力 for python
	std::cout << "失敗したノードの座標 for python" << std::endl << std::endl;
	std::cout << "\tx_falture = [";
	for (size_t i = 0; i < faild_leg_pos.size(); i++)
	{
		if (std::get<1>(faild_leg_pos[i]).z < kZMax) 
		{
			std::cout << std::get<1>(faild_leg_pos[i]).ProjectedXY().GetLength() << ",";
		}
	}

	std::cout << "] " << std::endl;
	std::cout << "\tz_falture = [";

	for (size_t i = 0; i < faild_leg_pos.size(); i++)
	{
		if (std::get<1>(faild_leg_pos[i]).z < kZMax)
		{
			float z = std::get<1>(faild_leg_pos[i]).z;
			if(abs(z -30) < 2.f)z = 30.f;
			std::cout << z << ",";
		}
	}
	std::cout << "] " << std::endl;

}
