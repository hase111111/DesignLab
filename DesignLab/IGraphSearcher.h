#pragma once
#include <vector>
#include "Node.h"
#include "Target.h"

//�O���t�؂��쐬����N���X�̃C���^�[�t�F�[�X�D���͍̂쐬�ł��Ȃ��̂ł�����p�����Ă��N���X���g���Ă��������D
//�p���̎d����g������������Ȃ��ꍇ�́CGraphSearcherSample�����Ă݂Ă��������D
class IGraphSearcher
{
public:
	IGraphSearcher() = default;
	virtual ~IGraphSearcher() = default;		//�p��������N���X�̃f�X�g���N�^��virtual�ɂ��Ă����D�Q�l https://www.yunabe.jp/docs/cpp_virtual_destructor.html

	//�O���t���󂯎��C���̒�����œK�Ȏ��̓�����o�͂���D
	virtual bool searchGraphTree(const std::vector<SNode>& _graph, const STarget& _target, SNode& _output_result) = 0;

};
