//! @file file_tree.h
//! @brief �t�@�C���c���[���쐬����N���X�D

#ifndef DESIGNLAB_FILE_TREE_H_
#define DESIGNLAB_FILE_TREE_H_


#include <string>
#include <vector>


//! @class FileTree
//! @brief �t�@�C���c���[���쐬����N���X�D
class FileTree final
{
public:

	//! @brief �t�@�C���c���[��\������
	//! @param [in] path �t�@�C���c���[���쐬����f�B���N�g���̃p�X
	//! @param [in] max_depth �t�@�C���c���[�̐[���C-1�Ȃ�Ζ����ɍs��
	void DisplayFileTree(const std::string& path, int max_depth) const;

	//! @brief �f�B���N�g���̒�����C�t�@�C������I������
	//! @param [in] path �t�@�C���c���[���쐬����f�B���N�g���̃p�X
	//! @param [in] max_depth �t�@�C���c���[�̐[���C-1�Ȃ�Ζ����ɍs��
	//! @param [in] extension �t�@�C���̊g���q( ".txt" ".csv" �Ȃ�)�C�󕶎��Ȃ�ΑS�Ẵt�@�C����ΏۂƂ���
	//! @param [in] keyword �t�@�C�����Ɋ܂܂��L�[���[�h�C�󕶎��Ȃ�ΑS�Ẵt�@�C����ΏۂƂ���
	//! @param [out] output �I�����ꂽ�t�@�C���̃p�X
	//! @return bool �t�@�C�����I�����ꂽ���ǂ���
	bool SelectFile(const std::string& path, int max_depth, const std::string& extension, const std::string keyword, std::string *output) const;

private:

	//! @struct FileTreeData
	//! @brief �t�@�C���c���[�̃f�[�^�D
	//! @n �f�B���N�g����\�����Ă���D�ċA�I�ɏ��������邽�ߎ������g�����L����D
	struct FileTreeData final
	{
		FileTreeData() : path(""), directory({}), file({}) {}
		FileTreeData(const std::string& _path, const std::vector<FileTreeData>& _directory, const std::vector<std::string>& _file) : 
			path(_path), 
			directory(_directory), 
			file(_file) 
		{
		}

		std::string path;					//!< �������g�ւ̃p�X
		std::vector<FileTreeData> directory;//!< �f�B���N�g��
		std::vector<std::string> file;		//!< �f�B���N�g���̒��̃t�@�C��
	};

	//! @brief �t�@�C���c���[���쐬����
	//! @param [in] path �t�@�C���c���[���쐬����f�B���N�g���̃p�X
	//! @param [in] max_depth �t�@�C���c���[�̐[���C-1�Ȃ�Ζ����ɍs��
	//! @param [in] extension �t�@�C���̊g���q( ".txt" ".csv" �Ȃ�)�C�󕶎��Ȃ�ΑS�Ẵt�@�C����ΏۂƂ���
	//! @param [in] keyword �t�@�C�����Ɋ܂܂��L�[���[�h�C�󕶎��Ȃ�ΑS�Ẵt�@�C����ΏۂƂ���
	//! @return FileTreeData �t�@�C���c���[�̃f�[�^
	FileTreeData MakeFileTree(const std::string& path, int max_depth, const std::string& extension, const std::string keyword) const;

	//! @brief �t�@�C���c���[���R�}���h���C���֏o�͂���
	//! @n �t�@�C���c���[�̃f�[�^���ċA�I�ɏo�͂���
	//! @param [in] tree �t�@�C���c���[�̃f�[�^
	//! @param [in] depth �f�B���N�g���̊K�w�̐[���D
	//! @param [in] not_display_empty ��̃f�B���N�g����\�����Ȃ��Ȃ��true���w�肷��D
	//! @param [in] file_count �t�@�C���̔ԍ������蓖�Ă邽�߂̃J�E���^�D
	void OutputFileTree(const FileTreeData& tree, int depth, bool not_display_empty, int* file_count) const;

	//! @brief �t�@�C���c���[�̃f�[�^����C�t�@�C���̃��X�g���쐬����
	//! @param [in] tree �t�@�C���c���[�̃f�[�^
	std::vector<std::string> MakeFileList(const FileTreeData& tree) const;
};


#endif	// DESIGNLAB_FILE_TREE_H_