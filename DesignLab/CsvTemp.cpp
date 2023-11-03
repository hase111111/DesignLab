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

	//�f�B���N�g�����w�肷��
	//std::filesystem::path path = "./result/�V�~�����[�V����_����";
	//std::filesystem::path path = "./result/�V�~�����[�V����_15[deg]";
	std::filesystem::path path = "./result/�V�~�����[�V����_-15[deg]";
	//std::filesystem::path path = "./result/�V�~�����[�V����_100[mm]";
	//std::filesystem::path path = "./result/�V�~�����[�V����_-100[mm]";



	//std::filesystem::path path = "./result/���@����_����";
	//std::filesystem::path path = "./result/���@����_100[mm]";
	//std::filesystem::path path = "./result/���@����_-100[mm]";
	//std::filesystem::path path = "./result/���@����_-15[deg]";
	//std::filesystem::path path = "./result/���@����_15[deg]";

	//�f�B���N�g�������݂��Ȃ��ꍇ�͏I��
	if (!std::filesystem::exists(path)) { return; }


	int node_sum = 0;						//�m�[�h�̑���
	int faild_node_sum = 0;					//���s�����m�[�h�̑���
	int interpolated_faild_node_sum = 0;	//���s�����m�[�h�̑���
	int interpolated_faild_lift_node_sum = 0;
	int interpolated_faild_move_node_sum = 0;
	int interpolated_faild_ground_node_sum = 0;

	int success_count = 0;

	std::vector<std::tuple<int, Vector3>> faild_leg_pos;

	//�f�B���N�g������node_list1.csv����node_list5.csv�܂ł̃t�@�C����ǂݍ���
	for (int n = 1; n <= 5; n++)
	{
		//�t�@�C�������w�肷��
		std::string file_name = "node_list" + std::to_string(n) + ".csv";
		std::string sim_result = "sim_result" + std::to_string(n) + ".csv";

		//sim_result�t�@�C�����J��
		std::ifstream ifs_res(path / sim_result);
		
		//�ŏ���1�s��ǂ�
		std::string str = "";
		std::getline(ifs_res, str);
		if (str != "Simulation Result,kSuccess") 
		{
			std::cout << "�V�~�����[�V���������s���Ă��܂�" << std::endl;
			continue;
		}

		//�t�@�C�����J��
		std::ifstream ifs(path / file_name);

		//�t�@�C�����J���Ȃ������ꍇ�͏I��
		if (!ifs) 
		{
			std::cout << "�t�@�C�����J���܂���ł���" << std::endl;
			return; 
		}

		ResultFileImporter result_file_importer;
		std::vector<RobotStateNode> node_list;
		MapState map_state;

		//�t�@�C������m�[�h���X�g�ƃ}�b�v�̏�Ԃ�ǂݍ���
		if (!result_file_importer.ImportNodeListAndMapState(path.string() + std::string("/") + file_name, &node_list, &map_state)) 
		{
			std::cout << "�t�@�C���̓ǂݍ��݂Ɏ��s���܂���" << std::endl;
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
					std::cout << "\t" << w << "�r��" << i << "�Ԗڂ̃m�[�h�����s���܂���" << std::endl;
					faild_leg_pos.push_back(std::make_tuple(w, node_list[i].leg_pos[w]));
					is_faild = true;
				}
			}

			//past_node_is_vaild���X�V���郉���_��
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

			//��Ԃ���m�[�h�𐶐�����
			
			// i  - 1�Ԗڂ̃m�[�h���Ȃ���Α��s
			if (i == 0) { 
				update_past_node_is_vaild();
				continue; 
			}

			std::vector<RobotStateNode> interpolated_node;

			interpolated_node_creator.CreateInterpolatedNode(node_list[i - 1], node_list[i], &interpolated_node);

			for (int w = 0; w < 6; w++)
			{
				// �ߋ��܂��͌��݂̃m�[�h�����s���Ă���ꍇ�͖������đ��s
				if (!past_node_is_vaild[w] || !current_node_is_vaild[w]) { continue; }

				//���łɎ��s���Ă���Ȃ�I��
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
						std::cout << "\t" << w << "�r��" << i << "�Ԗڂ̃m�[�h���⊮�Ɏ��s���܂���" << std::endl;
						break;
					}
				}
			}

			update_past_node_is_vaild();
		}

		//�t�@�C�������
		ifs.close();
		ifs_res.close();

		success_count++;

		std::cout << "node_list" << n << ".csv�̓ǂݍ��݂��������܂���" << std::endl;
	}

	//���ʂ̏o��
	std::cout << std::endl;
	std::cout << "���������V�~�����[�V�����̐� : " << success_count << std::endl;
	std::cout << "�m�[�h�̑��� : " << node_sum << std::endl;
	std::cout << "���s�����m�[�h�̑��� : " << faild_node_sum + interpolated_faild_node_sum  << std::endl;
	std::cout << "�ʏ펞�Ɏ��s�����m�[�h�̑��� : " << faild_node_sum << std::endl;
	std::cout << "��ԂɎ��s�����m�[�h�̑��� : " << interpolated_faild_node_sum << std::endl;
	std::cout << "��ԂɎ��s�����m�[�h�̂����C�ړ����Ɏ��s�����m�[�h�̑��� : " << interpolated_faild_move_node_sum << std::endl;
	std::cout << "��ԂɎ��s�����m�[�h�̂����C�����グ���Ɏ��s�����m�[�h�̑��� : " << interpolated_faild_lift_node_sum << std::endl;
	std::cout << "��ԂɎ��s�����m�[�h�̂����C���n���Ɏ��s�����m�[�h�̑��� : " << interpolated_faild_ground_node_sum << std::endl;
	std::cout << std::endl;	

	//���ʂ� * 5 / success_count ���ďo��
	if (success_count != 5) 
	{
		std::cout << "���������V�~�����[�V�����̐� : " << success_count << std::endl;
		std::cout << "�m�[�h�̑��� : " << node_sum * 5 / success_count << std::endl;
		std::cout << "���s�����m�[�h�̑��� : " << (faild_node_sum + interpolated_faild_node_sum) * 5 / success_count << std::endl;
		std::cout << "�ʏ펞�Ɏ��s�����m�[�h�̑��� : " << faild_node_sum * 5 / success_count << std::endl;
		std::cout << "��ԂɎ��s�����m�[�h�̑��� : " << interpolated_faild_node_sum * 5 / success_count << std::endl;
		std::cout << "��ԂɎ��s�����m�[�h�̂����C�ړ����Ɏ��s�����m�[�h�̑��� : " << interpolated_faild_move_node_sum * 5 / success_count << std::endl;
		std::cout << "��ԂɎ��s�����m�[�h�̂����C�����グ���Ɏ��s�����m�[�h�̑��� : " << interpolated_faild_lift_node_sum * 5 / success_count << std::endl;
		std::cout << "��ԂɎ��s�����m�[�h�̂����C���n���Ɏ��s�����m�[�h�̑��� : " << interpolated_faild_ground_node_sum * 5 / success_count << std::endl;
		std::cout << std::endl;
	}

	//���s�����m�[�h�̍��W���o�� for python
	std::cout << "���s�����m�[�h�̍��W for python" << std::endl << std::endl;
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
