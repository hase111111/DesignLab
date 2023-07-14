#include "MyPolygon.h"

bool my_vec::SPolygon2::isConvex() const
{
	const int n = getVertexNum();

	//�������^�[��.���_����3�����̏ꍇ�͑��p�`�ł͂Ȃ�
	if (n < 3)
	{
		return false;
	}

	//�E��肩����肩�𒲂ׂ�
	const auto _v1 = vertex[1] - vertex[0];
	const auto _v2 = vertex[2] - vertex[1];

	bool _left_turn = _v1.cross(_v2) > 0.0f;

	for (int i = 1; i < n; ++i)
	{
		const auto& v1 = vertex[(i + 1) % n] - vertex[i];
		const auto& v2 = vertex[(i + 2) % n] - vertex[(i + 1) % n];

		if (_left_turn && v1.cross(v2) < 0.0f)
		{
			return false;
		}
		else if (!_left_turn && v1.cross(v2) > 0.0f)
		{
			return false;
		}
	}
	return true;
}

bool my_vec::SPolygon2::isInside(const SVector2& _p) const
{
	const int _num = getVertexNum();

	//�������^�[��.���_����3�����̏ꍇ�͑��p�`�ł͂Ȃ�
	if (_num < 3)
	{
		return false;
	}

	int _cnt = 0;

	//���_���E��肩����肩�𒲂ׂ�
	bool _left_turn = (vertex[1] - vertex[0]).cross(vertex[2] - vertex[1]) > 0.0f;

	if (!_left_turn)
	{
		for (int i = 0; i < _num; ++i)
		{
			const auto& v1 = vertex[i] - _p;
			const auto& v2 = vertex[(i + 1) % _num] - _p;

			if (v1.cross(v2) == 0.0f && v1.dot(v2) <= 0.0f)
			{
				return true;	//�_���ӏ�ɂ���
			}

			if (v1.y < v2.y)
			{
				if (v1.y < 0.0f && 0.0f <= v2.y && v1.cross(v2) > 0.0f)
				{
					--_cnt;
				}
			}
			else
			{
				if (v2.y < 0.0f && 0.0f <= v1.y && v1.cross(v2) < 0.0f)
				{
					++_cnt;
				}
			}
		}
	}
	else
	{
		for (int i = 0; i < _num; ++i)
		{
			const auto& v1 = vertex[i] - _p;
			const auto& v2 = vertex[(i + 1) % _num] - _p;

			if (v1.cross(v2) == 0.0f && v1.dot(v2) <= 0.0f)
			{
				return true;	//�_���ӏ�ɂ���
			}

			if (v1.y > v2.y)
			{
				if (v2.y < 0.0f && 0.0f <= v1.y && v1.cross(v2) < 0.0f)
				{
					--_cnt;
				}
			}
			else
			{
				if (v1.y < 0.0f && 0.0f <= v2.y && v1.cross(v2) > 0.0f)
				{
					++_cnt;
				}
			}
		}
	}

	return (_cnt % 2 == 1);
}
