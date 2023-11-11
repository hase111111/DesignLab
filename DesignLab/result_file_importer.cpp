#include "result_file_importer.h"

#include <filesystem>
#include <fstream>
#include <sstream>

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
    if (! fs::exists(file_path)) 
    {
        dlio::Output("NodeList�t�@�C�������݂��܂���ł����D", OutputDetail::kError);
		return false;
    }

    // node_list1.csv �Ȃ�� map_state1.csv���ǂݍ���
    std::string map_file_path = file_path;
    map_file_path.replace(map_file_path.find(ResultFileConst::kNodeListName), ResultFileConst::kNodeListName.size(), ResultFileConst::kMapStateName);

    if (! fs::exists(map_file_path))
    {
        dlio::Output("MapState�t�@�C�������݂��܂���ł����D", OutputDetail::kError);
        return false;
    }


    if (! ImportNodeList(file_path, node_list) || ! ImportMapState(map_file_path, map_state)) 
    {
        dlio::Output("�t�@�C���ǂݍ��ݒ��ɃG���[���������܂����D", OutputDetail::kError);
    }

    return true;
}

bool ResultFileImporter::ImportNodeList(const std::string& file_path, [[maybe_unused]]std::vector<RobotStateNode>* node_list) const
{
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
    for (const auto &i : str_list)
    {
        RobotStateNode node;
        std::stringstream ss(i);

        node = RobotStateNode::FromString(ss.str());

		(*node_list).push_back(node);
    }

    return true;
}

bool ResultFileImporter::ImportMapState(const std::string& file_path, MapState* map_state) const
{
    assert(map_state != nullptr);   

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
    (*map_state).ClearMapPoint();

    for (const auto &i : str_list)
    {
        designlab::Vector3 point;
        std::stringstream ss(i);

        ss >> point;

		(*map_state).AddMapPoint(point);
    }

    return true;
}