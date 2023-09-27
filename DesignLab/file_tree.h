//! @file file_tree.h
//! @brief �t�@�C���c���[���쐬����N���X�D

#ifndef DESIGNLAB_FILE_TREE_H_
#define DESIGNLAB_FILE_TREE_H_

#include <string>
#include <vector>

class FileTree
{
public:

	//! @brief �t�@�C���c���[���쐬����
	//! @param [in] path �t�@�C���c���[���쐬����f�B���N�g���̃p�X
	//! @param [in] max_depth �t�@�C���c���[�̐[���C-1�Ȃ�Ζ����ɍs��
	void DisplayFileTree(const std::string& path, int max_depth) const;

private:

	struct FileTreeData
	{
		FileTreeData() : path(""), directory({}), file({}) {}
		FileTreeData(const std::string& _path, const std::vector<FileTreeData>& _directory, const std::vector<std::string>& _file) : 
			path(_path), 
			directory(_directory), 
			file(_file) 
		{
		}

		std::string path;					//!< �p�X
		std::vector<FileTreeData> directory; //!< �f�B���N�g��
		std::vector<std::string> file;		//!< �t�@�C��
	};

	//! @brief �t�@�C���c���[���쐬����
	//! @param [in] path �t�@�C���c���[���쐬����f�B���N�g���̃p�X
	//! @param [in] max_depth �t�@�C���c���[�̐[���C-1�Ȃ�Ζ����ɍs��
	//! @return FileTreeData �t�@�C���c���[�̃f�[�^
	FileTreeData MakeFileTree(const std::string& path, int max_depth) const;

	void OutputFileTree(const FileTreeData& tree,int depth) const;
};


#endif	// DESIGNLAB_FILE_TREE_H_