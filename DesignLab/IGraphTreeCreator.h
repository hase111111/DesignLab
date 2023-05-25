#pragma once
#include "MapState.h"
#include "listFunc.h"
#include <vector>

//�O���t�؂��쐬����N���X�̃C���^�[�t�F�[�X�D���͍̂쐬�ł��Ȃ��̂ł�����p�����Ă��N���X���g���Ă��������D
class IGraphTreeCreator
{
public:
	IGraphTreeCreator() = default;
	virtual ~IGraphTreeCreator() = default;		//�p��������N���X�̃f�X�g���N�^��virtual�ɂ��Ă����D�Q�l https://www.yunabe.jp/docs/cpp_virtual_destructor.html

	//�O���t�؂��쐬����N���X�D
	virtual bool createGraphTree(const SNode& _current_node, const MapState* const _p_map, std::vector<SNode>& _output_graph) = 0;

private:

};
