#pragma once
#include <vector>
#include "MyVector2.h"

namespace my_vec
{
	//! @struct my_vec::SPolygon2
	//! @brief 2�����̑��p�`��\���\����
	//! @details 2�����̑��p�`��\���\���́D
	struct SPolygon2 final
	{
		SPolygon2() = default;

		//! @brief ���_��ǉ�����֐�
		//! @param[in] v �ǉ����钸�_
		//! @note ���̒��_�Əd�Ȃ��Ă���ꍇ�ł��ǉ�����D
		inline void addVertex(const SVector2& v)
		{
			vertex.push_back(v);
		}

		//! @brief ���_��ǉ�����֐��D���̒��_�Əd�Ȃ��Ă���ꍇ�͒ǉ����Ȃ�
		//! @param[in] v �ǉ����钸�_
		//! @note ���̒��_�Əd�Ȃ��Ă���ꍇ�͒ǉ����Ȃ��D���̏����̕������x���Ȃ�̂ŁC�d�Ȃ邱�Ƃ��Ȃ��ꍇ��addVertex���g������
		//! @return �ǉ��ł������ǂ���
		inline bool addVertexCheckForDuplicates(const SVector2& v)
		{
			for (const auto& i : vertex)
			{
				if (i == v)
				{
					return false;
				}
			}

			vertex.push_back(v);
			return true;
		}

		//! @brief ���_���폜����֐�
		//! @param[in] i �폜���钸�_�̃C���f�b�N�X
		//! @note ���݂��Ȃ����_���w�肵���ꍇ�͉������Ȃ��D
		//! @note �폜�������_�̃C���f�b�N�X�͕ς��̂Œ��ӁD
		inline void removeVertex(int i)
		{
			if (i < 0 || i >= getVertexNum())
			{
				return;
			}

			vertex.erase(vertex.begin() + i);
		}

		//! @brief 1�ԍŌ�̒��_���폜����֐�
		//! @note ���_��1���Ȃ��ꍇ�͉������Ȃ��D
		inline void removeLastVertex()
		{
			if (getVertexNum() == 0)
			{
				return;
			}

			vertex.pop_back();
		}

		//! @brief ���p�`�̒��_����Ԃ��֐�
		//! @return ���p�`�̒��_��
		inline int getVertexNum() const { return static_cast<int>(vertex.size()); }

		//! @brief ���_�̍��W��Ԃ��֐�
		//! @param[in] i ���_�̃C���f�b�N�X
		//! @return ���_�̍��W
		//! @note ���݂��Ȃ����_���w�肵���ꍇ��(0,0)��Ԃ��D
		inline SVector2 getVertex(int i) const
		{
			if (i < 0 || i >= getVertexNum())
			{
				return SVector2(0, 0);
			}
			return vertex[i];
		}

		//! @brief ���p�`���ʂ��ǂ������ׂ�֐�
		//! @return �ʂȂ�true�C���Ȃ�false
		bool isConvex() const
		{
			const int n = getVertexNum();

			//�������^�[��.���_����3�����̏ꍇ�͑��p�`�ł͂Ȃ�
			if (n < 3)
			{
				return false;
			}

			for (int i = 0; i < n; ++i)
			{
				const auto& v1 = vertex[(i + 1) % n] - vertex[i];
				const auto& v2 = vertex[(i + 2) % n] - vertex[(i + 1) % n];

				if (v1.cross(v2) < 0.0f)
				{
					return false;
				}
			}
			return true;
		}

		//! @brief �_�����p�`�̓����ɂ��邩�ǂ������ׂ�֐�
		//! @param[in] p ���ׂ�_
		//! @return �����ɂ���Ȃ�true�C�O���ɂ���Ȃ�false
		//! @note �_�����p�`�̕ӏ�ɂ���ꍇ�͓����ɂ���Ɣ��肷��D
		//! @note ���p�`���ʂłȂ��ꍇ�͐���������ł��Ȃ��D
		bool isInside(const SVector2& _p) const
		{
			const int _num = getVertexNum();

			//�������^�[��.���_����3�����̏ꍇ�͑��p�`�ł͂Ȃ�
			if (_num < 3)
			{
				return false;
			}

			int _cnt = 0;

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

			return (_cnt % 2 == 1);
		}

	private:
		std::vector<SVector2> vertex;	//!< ���_���W
	};
}