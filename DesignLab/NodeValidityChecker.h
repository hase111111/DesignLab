#pragma once
#include "Node.h"
#include <deque>


class NodeValidityChecker final
{
public:
	NodeValidityChecker() = default;
	~NodeValidityChecker() = default;

	//! ���{�b�g���s����������Z�b�g����D
	//! @param [in] _node ���{�b�g�̌��݂̏��
	void setNode(const SNode& _node);

	//!���߂ɍs����������r���āC���{�b�g������������ł��Ă��邩���f���܂��D
	//! @return bool ���삪���[�v���Ă���C�܂萳�������삵�Ă��Ȃ�����true��Ԃ��D
	bool isLoopMove() const;

private:
	std::deque<SNode> m_node;		//���e������������ێ�����D
	const int MAX_DATA_NUM = 20;	//�S�Ẵf�[�^��ێ���������Ɣ��ɏd�����Ȃ�̂ŁC�̕ϐ��̒l�̐��܂Ńf�[�^�����D��y�̃v���O�����ł�30�����悤�ɂ��Ă����̂ŁC���̂��炢�̐��ɂ��Ƃ� 
};


//! @file NodeValidityChecker.h
//! @brief ����̃��[�v�����m����N���X�̎����D
//! @author ���J��

//! @class NodeValidityChecker
//! @brief ����̃��[�v�����m����N���X
//! @details �O���t�T����p���ă��{�b�g�̕��e���������Ă���ƁC�őP�̍s�����Ƃ�ƁC���l�̑�����������ē��삪���[�v���Ă��܂��ꍇ������D<br>
//! �����T�m���āC���e�����Ɏ��s���Ă��邱�Ƃ�ʒB����N���X�D
//! @author ���J��
