#include "file_tree.h"

#include <filesystem>

#include "cassert_define.h"
#include "cmdio_util.h"


namespace dlio = designlab::cmdio;


void FileTree::DisplayFileTree(const std::string& path, int max_depth) const
{
	FileTreeData tree = MakeFileTree(path, max_depth, "", "sim");

    int count = 0;
    OutputFileTree(tree, 0, true, &count);
}

bool FileTree::SelectFile(const std::string& path, int max_depth, const std::string& extension, const std::string keyword, std::string* output)
{
    // �t�@�C���c���[���쐬
    FileTreeData tree = MakeFileTree(path, max_depth, extension, keyword);

    // �t�@�C���c���[��\��
    dlio::OutputHorizontalLine(true,OutputDetail::kSystem);

    int count = 0;
    OutputFileTree(tree, 0, true, &count);
    dlio::OutputNewLine(1, OutputDetail::kSystem);
    dlio::OutputHorizontalLine(true, OutputDetail::kSystem);
    dlio::OutputNewLine(1, OutputDetail::kSystem);

    // �t�@�C����I��
    std::vector<std::string> file_list = MakeFileList(tree);

    if (file_list.empty())
	{
        dlio::Output("�t�@�C�������݂��܂���ł����D",OutputDetail::kSystem);
		return false;
	}

    while (true)
    {
        int select_index = dlio::InputInt(0, static_cast<int>(file_list.size()) - 1, 0, "�t�@�C����I�����Ă��������D�����œ��͂��Ă��������D");

        dlio::OutputNewLine(1, OutputDetail::kSystem);
        dlio::Output("�I�������t�@�C����" + file_list[select_index] + "�ł��D", OutputDetail::kSystem);

		if (dlio::InputYesNo("��낵���ł����H (yes / no)"))
		{
			*output = file_list[select_index];
            break;
		}

        dlio::OutputNewLine(1, OutputDetail::kSystem);
    }

    return true;
}

FileTree::FileTreeData FileTree::MakeFileTree(const std::string& path, int max_depth, const std::string& extension, const std::string keyword) const
{
    FileTreeData tree;

    tree.path = path;

    assert(std::filesystem::exists(tree.path));

    for (const auto& entry : std::filesystem::directory_iterator(path)) 
    {
        if (entry.is_directory()) 
        {
            // �f�B���N�g���̏ꍇ�A�ċA�I�Ƀt�@�C���c���[���쐬
            if (max_depth == 0) 
            {
                tree.directory.push_back(FileTreeData{ entry.path().string(), {}, {} });
            }
            else
            {
                tree.directory.push_back(MakeFileTree(entry.path().string(), max_depth - 1, extension, keyword));
            }
        }
        else if (entry.is_regular_file()) 
        {
            // �t�@�C���̏ꍇ�A�t�@�C������ǉ�

            if (not extension.empty()) 
            {
                if (entry.path().extension().string() != extension)
                {
					continue;
				}
            }

            if (not keyword.empty()) 
            {
                if (entry.path().filename().string().find(keyword) == std::string::npos)
                {
                    continue;
                }
            }

            tree.file.push_back(entry.path().filename().string());
        }
    }

    return tree;
}

void FileTree::OutputFileTree(const FileTreeData& tree, int depth, bool not_display_empty, int* file_count) const
{
    assert(file_count != nullptr);

    std::string indent = "";
    for (int i = 0; i < depth; ++i)
    {
		indent += "| ";
	}

    // �� ���� ��̃f�B���N�g����\�����Ȃ��ݒ� �łȂ��Ȃ�Ε\������
    if(not (not_display_empty && tree.file.empty() && tree.directory.empty()))
    {
        // �f�B���N�g�������o�͂���ۂɁC�p�X�̊K�w���폜����

        dlio::Output(indent, OutputDetail::kSystem);

        std::string::size_type pos = tree.path.find_last_of("/\\");
        std::string dir_name = ((depth == 0) ? "" : "- ");
        dir_name += std::string("[ ") + tree.path.substr(pos + 1) + std::string(" ]");

        dlio::Output(indent + dir_name, OutputDetail::kSystem);
    }

    for (const auto& directory : tree.directory)
    {
		OutputFileTree(directory, depth + 1, not_display_empty, file_count);
	}

    if(not tree.file.empty())
    {
        dlio::Output(indent + "|", OutputDetail::kSystem);

        for (const auto& file : tree.file)
        {
            dlio::Output(indent + "|- " + file + " [-" + std::to_string(*file_count) + "-]", OutputDetail::kSystem);

            (*file_count)++;
        }
    }
}

std::vector<std::string> FileTree::MakeFileList(const FileTreeData& tree) const
{
    std::vector<std::string> file_list;

	for (const auto& directory : tree.directory)
	{
		std::vector<std::string> tmp = MakeFileList(directory);

		file_list.insert(file_list.end(), tmp.begin(), tmp.end());
	}

	for (const auto& file : tree.file)
	{
		file_list.push_back(tree.path + "\\" + file);
	}

	return file_list;
}
