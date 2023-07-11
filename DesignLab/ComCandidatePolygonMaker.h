#pragma once
#include "Node.h"
#include "MyPolygon.h"
#include <vector>

//! @brief �d�S�ʒu�̌��n�_���������p�`���쐬����N���X
//! @details ���݂̃��{�b�g�̏�Ԃ�\���m�[�h����C�d�S�ʒu�̌��n�_���������p�`���쐬����<br>
//! ��@�͔g������̑��ƌ������Q�l�ɂ��Ă��邽�߁C�ڍׂ͂�������Q�Ƃ̂���<br>
//! ���R�����C���̎�@�ł̓��{�b�g�̎p���ύX���l�����Ă��Ȃ��̂ł��̃N���X���g�p����ꍇ�́C<br>
//! ���{�b�g�̉�]�E����͍s�����Ƃ��ł��Ȃ�
class ComCandidatePolygonMaker final
{
public:
	ComCandidatePolygonMaker() = default;

	//! @brief ���݂̃��{�b�g�̏�Ԃ�\���m�[�h����C�d�S�ʒu�̌��n�_���������p�`���쐬����
	//! @param[in] node ���݂̃��{�b�g�̏�Ԃ�\���m�[�h
	//! @param[out] _output_poly �d�S�ʒu�̌��n�_���������p�`
	void makeCandidatePolygon(const SNode& node, std::vector<my_vec::SPolygon2>& _output_poly);

private:

};