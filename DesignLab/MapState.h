#pragma once
#include "vectorFunc.h"
#include <vector>
#include <map>
#include "MapCreator.h"

class MapState final
{
public:
	MapState() = delete;	//�f�t�H���g�R���X�g���N�^�͏������āC�K�����̃R���X�g���N�^�Ń}�b�v���������Ȃ��Ǝ��̂𐶐��ł��Ȃ��悤�ɂ���D

	MapState(const EMapCreateMode _mode, const int _option, const bool _do_output);
	MapState(const MapState& _other);	//�R�s�[�R���X�g���N�^


private:

	//���{�b�g�����邭�}�b�v�D�r�ݒu�\�_�̏W���ŕ\�������D
	std::vector<myvector::SVector> m_map_data;

	//�}�b�v�����݂���̈�𐳕��`�ɐ؂蕪���āC���̒��ɑ��݂���r�ݒu�\�_���W�߂����́D
	std::map< std::pair<int, int>, std::vector<myvector::SVector> > m_devide_map;
};
