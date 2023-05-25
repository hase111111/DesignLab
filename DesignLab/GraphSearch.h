#pragma once
#include "MapState.h"
#include "listFunc.h"
#include "IGraphTreeCreator.h"
#include <memory>

//�O���t�T�����s���N���X�D���O�ʂ�
class GraphSearch final
{
public:
	GraphSearch() = default;
	~GraphSearch() = default;

	//�O���t�T�����s���C���̓���Ƃ��čœK�ȃm�[�h��Ԃ��D���ʂ� output_node �ŎQ�Ɠn�����C�߂�l�̓O���t�T���ɐ����������ǂ�����Ԃ�(false�Ȃ玸�s�D)
	// _current_node�@�c�@���݂̏�Ԃ�\���m�[�h�D _map�@�c�@�}�b�v�̏�Ԃւ̃|�C���^�D�R�s�[�œn���Əd�����̂Ń|�C���^��n���D
	//       _target�@�c�@����̖ڕW��ݒ肵�����́D
	bool getNextNodebyGraphSearch(const SNode& _current_node, const MapState* const _p_map, const STarget& _target, SNode& _output_node);

private:
	std::unique_ptr<IGraphTreeCreator> mp_GraphTreeCreator;
};
