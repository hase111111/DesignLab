#include "result_file_importer.h"

#include <filesystem>
#include <fstream>

#include "cassert_define.h"
#include "cmdio_util.h"


namespace dlio = designlab::cmdio;
namespace fs = std::filesystem;


bool ResultFileImporter::ImportNodeListAndMapState(const std::string& file_path, std::vector<RobotStateNode>* node_list, MapState* map_state) const
{
    // �����̊m�F
    assert(node_list != nullptr);
    assert(node_list->empty());
    assert(map_state != nullptr);
    assert(map_state->GetMapPointSize() == 0);


    // �t�@�C�������݂��邩�ǂ������m�F�D�Ȃ��Ȃ��false��Ԃ��D
    if (not fs::exists(file_path)) 
    {
        dlio::Output("�t�@�C�������݂��܂���ł����D", OutputDetail::kSystem);
		return false;
    }

    if (not fs::exists(file_path))
    {
        dlio::Output("�t�@�C�������݂��܂���ł����D", OutputDetail::kSystem);
        return false;
    }

    // �t�@�C�����J��
    std::ifstream ifs(file_path);

    // �t�@�C�����J���Ȃ��Ȃ��false��Ԃ��D
    if (not ifs.is_open())
    {
		dlio::Output("�t�@�C�����J���܂���ł����D", OutputDetail::kSystem);

        return false;
    }


    // �t�@�C����ǂݍ���
    std::string str;
    std::vector<std::string> str_list;

    while (std::getline(ifs, str))
    {
		str_list.push_back(str);
	}

    // �t�@�C�������
    ifs.close();

    // �t�@�C���̓��e����͂���
    // �m�[�h���X�g�̓ǂݍ���


    

    return false;
}
