#include "ComSelecter.h"
#include <algorithm>
#include <iostream>

bool ComSelecter::getComFromPolygon(const my_vec::SPolygon2& polygon, const ComType::EComPattern _com_pattren, my_vec::SVector& _output_com) const
{
	std::vector<my_vec::SVector2> _coms;

	//���_�𐶐�����
	makeComCandidatePoint(polygon, _coms);

	//���_�����݂̏d�S����ł������Ɉړ��ł��鏇�Ƀ\�[�g����D��3�����̂������̂̓����_���C�ȒP�Ɍ����Ɗ֐����֐��̒��Ő錾�ł����D�ނ����̂ŗ������Ȃ��Ă悢
	//�Q�l�Fhttps://qiita.com/kemkemG0/items/76988e8e62c8a2a9c90a

	std::sort(_coms.begin(), _coms.end(),
		[&](const my_vec::SVector2& _v1, const my_vec::SVector2& _v2)
		{
			return (_v1 - m_current_node.global_center_of_mass.projectedXY()).lengthSquare() > (_v2 - m_current_node.global_center_of_mass.projectedXY()).lengthSquare();
		}
	);

	//���_�����ԂɃ`�F�b�N���C�ړ���̏d�S������]�T�𖞂����Ȃ�΁C���̓_���d�S�Ƃ��č̗p����D
	for (const auto& i : _coms)
	{
		//���݂̏d�S���ړ����������̂��쐬����
		SNode _temp = m_current_node;
		my_vec::SVector _next_com = { i.x ,i.y,m_current_node.global_center_of_mass.z };
		_temp.changeGlobalCenterOfMass(_next_com);

		//if (isInMargin(polygon, i) == false) { continue; }	//����]�T�𖞂����Ȃ���Ύ��̌��_��

		if (m_calclator.isLegInterfering(_temp) == true) { continue; }	//�r�������Ă���Ύ��̌��_��

		if (m_calclator.isAllLegInRange(_temp) == false) { continue; }	//�r�����͈͊O�Ȃ�Ύ��̌��_��

		if (m_calclator.isAblePause(_temp) == false) { continue; }		//�p��������ł��Ȃ��Ȃ�Ύ��̌��_��

		//�����܂ŗ�����C����]�T�𖞂����C�r�������Ă��炸�C�r�����͈͓��ŁC�p��������ł���Ƃ������ƂȂ̂ŁC���̓_���d�S�Ƃ��č̗p����
		_output_com = _next_com;
		return true;
	}


	if (DO_DEBUG_PRINT)
	{
		std::cout << ComType::convertComPatternToBit(_com_pattren) << "�̏d�S�����炸" << std::endl;
	}

	//�Y��������̂��Ȃ����false��Ԃ�
	return false;
}

void ComSelecter::makeComCandidatePoint(const my_vec::SPolygon2& polygon, std::vector<my_vec::SVector2>& _coms) const
{
	//�g������̏����ł͑��p�`���͂ނ悤�Ȏl�p�`�����̂ŁC�܂��͂�������
	const float _min_x = polygon.getMinX();
	const float _max_x = polygon.getMaxX();
	const float _min_y = polygon.getMinY();
	const float _max_y = polygon.getMaxY();

	const float _width = _max_x - _min_x;
	const float _height = _max_y - _min_y;

	const float _delta_width = _width / (float)DISCRETIZATION_NUM;
	const float _delta_height = _height / (float)DISCRETIZATION_NUM;

	//��L�̎l�p�`�̒��ɂ���_��S�Č��ɒǉ�����D�������C���p�`�̒��ɂ���_�̂݁D
	for (int _x = 0; _x <= DISCRETIZATION_NUM; _x++)
	{
		for (int _y = 0; _y <= DISCRETIZATION_NUM; _y++)
		{
			my_vec::SVector2 _temp(_min_x + _delta_width * _x, _min_y + _delta_height * _y);

			if (polygon.isInside(_temp))
			{
				_coms.push_back(_temp);
			}
		}
	}

	if (DO_DEBUG_PRINT == true) { std::cout << "CandidatePointNum is " << _coms.size() << std::endl; }
}

bool ComSelecter::isInMargin(const my_vec::SPolygon2& polygon, const my_vec::SVector2& _com) const
{
	// @todo ���삪���������̂ŏC�����邱��

	const int _vertex_num = polygon.getVertexNum();	//���_���D���x���g�p����̂ŁC��Ɍv�Z���Ă������ƂŌy������D

	for (int j = 0; j < _vertex_num; j++)
	{
		my_vec::SVector2 _v1 = polygon.getVertex((j + 1) % _vertex_num) - polygon.getVertex(j);
		_v1 = _v1.normalized();

		my_vec::SVector2 _v_map = _com - polygon.getVertex(j);

		if (_v_map.cross(_v1) < STABILITY_MARGIN)
		{
			//����]�T�𖞂����Ȃ��Ȃ�Ό�₩��폜����D
			return false;
		}
	}

	return true;
}
