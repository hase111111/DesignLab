#pragma once
#include "Node.h"
#include "MyPolygon.h"
#include "ComType.h"
#include <vector>

//! @brief �d�S�ʒu�̌��n�_���������p�`���쐬����N���X
//! @details ���݂̃��{�b�g�̏�Ԃ�\���m�[�h����C�d�S�ʒu�̌��n�_���������p�`���쐬����<br>
//! ��@�͔g������̑��ƌ������Q�l�ɂ��Ă��邽�߁C�ڍׂ͂�������Q�Ƃ̂���<br>
//! ���R�����C���̎�@�ł̓��{�b�g�̎p���ύX���l�����Ă��Ȃ��̂ł��̃N���X���g�p����ꍇ�́C<br>
//! ���{�b�g�̉�]�E����͍s�����Ƃ��ł��Ȃ�
class ComCandidatePolygonMaker final
{
private:

	//�f�o�b�O�p�ɏo�͂��s���ꍇ��true�ɂ���D�e�X�g�R�[�h�������������ǒ��ۉ��ł��Ă��Ȃ�...
	const bool DO_DEBUG_OUTPUT = false;

public:
	ComCandidatePolygonMaker() = default;

	//! @brief ���݂̃��{�b�g�̏�Ԃ�\���m�[�h����C�d�S�ʒu�̌��n�_���������p�`���쐬����
	//! @param[in] node ���݂̃��{�b�g�̏�Ԃ�\���m�[�h
	//! @param[out] _output_poly �d�S�ʒu�̌��n�_���������p�`
	void makeCandidatePolygon(const SNode& node, std::vector<std::pair<my_vec::SPolygon2, ComType::EComPattern>>& _output_poly) const;

private:

	//! @brief �d�S�ʒu�̌��n�_���������p�`���쐬����D���S����̐}�`��4�p�`��5�p�`��p���ĕ\������D
	my_vec::SPolygon2 makeCandidateBox(const my_vec::SVector2 _leg_pos[HexapodConst::LEG_NUM], const int _start_leg_num) const;

	//! @brief �d�S�ʒu�̌��n�_���������p�`���쐬����D���S����̐}�`��3�p�`��p���ĕ\������D
	//! @param[in] _leg_pos �r�̈ʒu
	void makeCandidateTriangle(const my_vec::SVector2 _leg_pos[HexapodConst::LEG_NUM], my_vec::SPolygon2& _out_poly, ComType::EComPattern& _out_com_pattern) const;

	//! @brief ���������p�`����������Ă��邩���m�F����
	//! @param[in] _poly �m�F���鑽�p�`
	//! @return ���������p�`����������Ă��邩
	bool checkPolygon(const my_vec::SPolygon2& _poly) const;
};