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

		//! @brief ���_�̒��ōő��x���W��Ԃ��֐�
		//! @return ���_�̒��ōő��x���W
		inline float getMaxX() const
		{
			float _max = vertex[0].x;

			for (int i = 1; i < getVertexNum(); ++i)
			{
				_max = std::max(_max, vertex[i].x);
			}

			return _max;
		}

		//! @brief ���_�̒��ōŏ���x���W��Ԃ��֐�
		//! @return ���_�̒��ōŏ���x���W
		inline float getMinX() const
		{
			float _min = vertex[0].x;

			for (int i = 1; i < getVertexNum(); ++i)
			{
				_min = std::min(_min, vertex[i].x);
			}

			return _min;
		}

		//! @brief ���_�̒��ōő��y���W��Ԃ��֐�
		//! @return ���_�̒��ōő��y���W
		inline float getMaxY() const
		{
			float _max = vertex[0].y;

			for (int i = 1; i < getVertexNum(); ++i)
			{
				_max = std::max(_max, vertex[i].y);
			}

			return _max;
		}

		//! @brief ���_�̒��ōŏ���y���W��Ԃ��֐�
		//! @return ���_�̒��ōŏ���y���W
		inline float getMinY() const
		{
			float _min = vertex[0].y;

			for (int i = 1; i < getVertexNum(); ++i)
			{
				_min = std::min(_min, vertex[i].y);
			}

			return _min;
		}

		//! @brief ���p�`���ʂ��ǂ������ׂ�֐�
		//! @return �ʂȂ�true�C���Ȃ�false
		bool isConvex() const;

		//! @brief �_�����p�`�̓����ɂ��邩�ǂ������ׂ�֐��D���p�`���ʂłȂ��ꍇ�͐���������ł��Ȃ��D
		//! @param[in] p ���ׂ�_
		//! @return �����ɂ���Ȃ�true�C�O���ɂ���Ȃ�false
		//! @note �_�����p�`�̕ӏ�ɂ���ꍇ�͓����ɂ���Ɣ��肷��D
		//! @note ���p�`���ʂłȂ��ꍇ�͐���������ł��Ȃ��D
		bool isInside(const SVector2& _p) const;

	private:
		std::vector<SVector2> vertex;	//!< ���_���W
	};

	// �o�̓X�g���[��
	template <class Char>
	inline std::basic_ostream<Char>& operator <<(std::basic_ostream<Char>& os, const SPolygon2& v)
	{
		os << "Vertex Num : " << v.getVertexNum() << "\n";

		for (int i = 0; i < v.getVertexNum(); ++i)
		{
			os << "Vertex " << i << " : " << v.getVertex(i) << "\n";
		}

		os << "Max X : " << v.getMaxX() << "\n";
		os << "Min X : " << v.getMinX() << "\n";
		os << "Max Y : " << v.getMaxY() << "\n";
		os << "Min Y : " << v.getMinY() << "\n";

		os << "Convex :" << (v.isConvex() ? "True" : "False") << "\n";

		return os;
	}
}