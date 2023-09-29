#pragma once

#include <deque>

#include "robot_state_node.h"


//! @class NodeValidityChecker
//! @author ���J��
//! @date 2023/08/06
//! @brief ����̃��[�v�����m����N���X
//! @details �O���t�T����p���ă��{�b�g�̕��e���������Ă���ƁC�őP�̍s�����Ƃ�ƁC���l�̑�����������ē��삪���[�v���Ă��܂��ꍇ������D
//! @n �����T�m���āC���e�����Ɏ��s���Ă��邱�Ƃ�ʒB����N���X�D
//! @n [deque(�f�b�N)�ɂ���] @n std::vector�̈���D@n vector�Ƃ̈Ⴂ�Ƃ��āC�擪�Ɩ����̗v�f�̒ǉ��E�폜�������ł���D
//! @n �Q�l : https://cpprefjp.github.io/reference/deque/deque.html
class NodeValidityChecker final
{
public:
	NodeValidityChecker() = default;
	~NodeValidityChecker() = default;

	//! ���{�b�g���s����������Z�b�g����D
	//! @param [in] _node ���{�b�g�̌��݂̏��
	void setNode(const RobotStateNode& node);

	//!���߂ɍs����������r���āC���{�b�g������������ł��Ă��邩���f���܂��D
	//! @return bool ���삪���[�v���Ă���C�܂萳�������삵�Ă��Ȃ�����true��Ԃ��D
	bool isLoopMove() const;

private:
	std::deque<RobotStateNode> m_node;		//���e������������ێ�����D

	const int kMaxDataNum = 20;		//�S�Ẵf�[�^��ێ���������Ɣ��ɏd�����Ȃ�̂ŁC�̕ϐ��̒l�̐��܂Ńf�[�^�����D��y�̃v���O�����ł�30�����悤�ɂ��Ă����̂ŁC���̂��炢�̐��ɂ��Ƃ� 
};


//! @file node_validity_checker.h
//! @author ���J��
//! @date 2023/08/06
//! @brief ����̃��[�v�����m����N���X�D
//! @n �s�� : @lineinfo