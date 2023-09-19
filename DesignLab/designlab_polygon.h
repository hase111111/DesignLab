#pragma once

#include <vector>

#include "designlab_vector2.h"


namespace designlab
{

	static constexpr int MAX_VERTEX_NUM = 6;	//!< ���x�𑁂����邽�߂ɂ��炩���ߍő�T�C�Y�����肵�Ă����D


	//! @struct designlab::SPolygon2
	//! @brief 2�����̑��p�`��\���\����
	//! @details 2�����̑��p�`��\���\���́D
	//! @n ���_�͔����v���C�����͎��v���ɕ���ł���K�v������D
	//! @n �����łȂ��ꍇ�͂������̊֐�������ɓ��삵�Ȃ��D@n
	//! @n �܂��C���_�̐��͍ő��MAX_VERTEX_NUM�܂ł����o�^�ł��Ȃ��D
	//! @n ����͓���̍������̂��߂Ɋm�ۂ����z��̃T�C�Y���Œ肵�Ă��邽�߂ł���D
	//! @n ���̒l��ύX����ꍇ�́C�R�[�h����MAX_VERTEX_NUM�̒l��ύX����K�v������D@n
	//! @n �Ȃ��C�R�[�h����max�֐��Cmin�֐��ɂ��Ă͈ȉ����Q�ƁD
	//! @n �Q�� : https://cpprefjp.github.io/reference/algorithm/max.html
	struct SPolygon2 final
	{

		SPolygon2() { vertex.resize(MAX_VERTEX_NUM); };

		//! @brief ���_��ǉ�����֐�
		//! @param [in] v �ǉ����钸�_
		//! @note ���̒��_�Əd�Ȃ��Ă���ꍇ�ł��ǉ�����D
		inline void addVertex(const SVector2& v)
		{
			vertex[vertex_num] = v;
			++vertex_num;
		}

		//! @brief ���_��ǉ�����֐��D���̒��_�Əd�Ȃ��Ă���ꍇ�͒ǉ����Ȃ�
		//! @param[in] v �ǉ����钸�_
		//! @note ���̒��_�Əd�Ȃ��Ă���ꍇ�͒ǉ����Ȃ��D���̏����̕������x���Ȃ�̂ŁC�d�Ȃ邱�Ƃ��Ȃ��ꍇ��addVertex���g������
		//! @return bool �ǉ��ł������ǂ����C�ǉ��ł����ꍇ��true�C�ǉ��ł��Ȃ������ꍇ��false
		inline bool addVertexCheckForDuplicates(const SVector2& v)
		{
			for (int i = 0; i < vertex_num; i++)
			{
				if (vertex[i] == v)
				{
					return false;
				}
			}

			vertex[vertex_num] = v;
			++vertex_num;
			return true;
		}

		//! @brief ���_���폜����֐��D�x���̂Ŏg�p����ׂ��ł͂Ȃ�
		//! @param [in] i �폜���钸�_�̃C���f�b�N�X
		//! @note ���݂��Ȃ����_���w�肵���ꍇ�͉������Ȃ��D
		//! @note �폜�������_�̃C���f�b�N�X�͕ς��̂Œ��ӁD
		inline void removeVertex(const int i)
		{
			if (i < 0 || i >= getVertexNum())
			{
				return;
			}

			vertex.erase(vertex.begin() + i);
			vertex.push_back({ 0,0 });
			--vertex_num;
		}

		//! @brief 1�ԍŌ�̒��_���폜����֐�
		//! @note ���_��1���Ȃ��ꍇ�͉������Ȃ��D
		inline void removeLastVertex()
		{
			if (getVertexNum() == 0)
			{
				return;
			}

			--vertex_num;
		}

		//! @brief ���p�`�����Z�b�g����֐�
		inline void reset()
		{
			vertex_num = 0;
		}

		//! @brief ���p�`�̒��_����Ԃ��֐�
		//! @return ���p�`�̒��_��
		constexpr int getVertexNum() const { return vertex_num; }

		//! @brief ���_�̍��W��Ԃ��֐�
		//! @param [in] i ���_�̃C���f�b�N�X
		//! @return SVector2 ���_�̍��W
		//! @note ���݂��Ȃ����_���w�肵���ꍇ��(0,0)��Ԃ��D
		inline SVector2 getVertex(int i) const
		{
			if (i < 0 || i >= getVertexNum())
			{
				return SVector2{ 0, 0 };
			}
			return vertex[i];
		}

		//! @brief ���_�̒��ōő��x���W��Ԃ��֐�
		//! @return float ���_�̒��ōő��x���W
		inline float getMaxX() const
		{
			float max_x = vertex[0].x;

			for (int i = 1; i < getVertexNum(); ++i)
			{
				max_x = (std::max)(max_x, vertex[i].x);
			}

			return max_x;
		}

		//! @brief ���_�̒��ōŏ���x���W��Ԃ��֐�
		//! @return float ���_�̒��ōŏ���x���W
		inline float getMinX() const
		{
			float min_x = vertex[0].x;

			for (int i = 1; i < getVertexNum(); ++i)
			{
				min_x = (std::min)(min_x, vertex[i].x);
			}

			return min_x;
		}

		//! @brief ���_�̒��ōő��y���W��Ԃ��֐�
		//! @return float ���_�̒��ōő��y���W
		inline float getMaxY() const
		{
			float max_y = vertex[0].y;

			for (int i = 1; i < getVertexNum(); ++i)
			{
				max_y = (std::max)(max_y, vertex[i].y);
			}

			return max_y;
		}

		//! @brief ���_�̒��ōŏ���y���W��Ԃ��֐�
		//! @return ���_�̒��ōŏ���y���W
		inline float getMinY() const
		{
			float min_y = vertex[0].y;

			for (int i = 1; i < getVertexNum(); ++i)
			{
				min_y = (std::min)(min_y, vertex[i].y);
			}

			return min_y;
		}

		//! @brief ���p�`���ʂ��ǂ������ׂ�֐�
		//! @return bool �ʂȂ�true�C���Ȃ�false
		bool isConvex() const;

		//! @brief �_�����p�`�̓����ɂ��邩�ǂ������ׂ�֐��D���p�`���ʂłȂ��ꍇ�͐���������ł��Ȃ��D
		//! @param [in] point ���ׂ�_
		//! @return bool �����ɂ���Ȃ�true�C�O���ɂ���Ȃ�false
		//! @note �_�����v���C�����v���̂����ꂩ�̏��ԂŒ��_������ł���K�v������D
		//! @note �_�����p�`�̕ӏ�ɂ���ꍇ�͓����ɂ���Ɣ��肷��D
		//! @note ���p�`���ʂłȂ��ꍇ�͐���������ł��Ȃ��D
		bool isInside(const SVector2& point) const;


	private:

		std::vector<SVector2> vertex;	//!< ���_���W
		int vertex_num = 0;				//!< ���_��

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

}	// namespace designlab


//! @file designlab_polygon.h
//! @date 2023/08/06
//! @author ���J��
//! @brief ���p�`��\���\����
//! @n �s�� : @lineinfo
