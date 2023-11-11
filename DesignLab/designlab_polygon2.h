//! @file designlab_polygon2.h
//! @brief ���p�`��\���\����


#ifndef DESIGNLAB_POLYGON2_H
#define DESIGNLAB_POLYGON2_H


#include <array>
#include <string>
#include <optional>
#include <vector>

#include "cassert_define.h"
#include "designlab_vector2.h"


namespace designlab
{

	static constexpr int kMaxVertexNum = 6;	//!< ���x�𑁂����邽�߂ɂ��炩���ߍő�T�C�Y�����肵�Ă����D


	//! @struct designlab::Polygon2
	//! @brief 2�����̑��p�`��\���\����
	//! @details 2�����̑��p�`��\���\���́D
	//! @n ���_�͔����v���C�����͎��v���ɕ���ł���K�v������D
	//! @n �����łȂ��ꍇ�͂������̊֐�������ɓ��삵�Ȃ��D
	//! @n
	//! @n �܂��C���_�̐��͍ő��kMaxVertexNum�܂ł����o�^�ł��Ȃ��D
	//! @n ����͓���̍������̂��߂Ɋm�ۂ����z��̃T�C�Y���Œ肵�Ă��邽�߂ł���D
	//! @n ���̒l��ύX����ꍇ�́C�R�[�h����kMaxVertexNum�̒l��ύX����K�v������D
	//! @n
	//! @n �Ȃ��C�R�[�h����max�֐��Cmin�֐��ɂ��Ă͈ȉ����Q�ƁD
	//! @n �Q�� : https://cpprefjp.github.io/reference/algorithm/max.html
	struct Polygon2 final
	{

		constexpr Polygon2() : vertex_num(0) {};

		Polygon2(std::vector<Vector2> vertex);


		//! @brief ���_��ǉ�����֐�
		//! @param [in] v �ǉ����钸�_
		//! @note ���̒��_�Əd�Ȃ��Ă���ꍇ�ł��ǉ�����D
		constexpr void AddVertex(const Vector2& v)
		{
			vertex[vertex_num] = v;
			++vertex_num;

			assert(vertex_num <= kMaxVertexNum);	// ���_���͍ő�l�𒴂��Ă͂����Ȃ�
		}

		//! @brief ���_��ǉ�����֐��D���̒��_�Əd�Ȃ��Ă���ꍇ�͒ǉ����Ȃ��D
		//! @param[in] v �ǉ����钸�_�̍��W
		//! @note ���̒��_�Əd�Ȃ��Ă���ꍇ�͒ǉ����Ȃ��D���̏����̕������x���Ȃ�̂ŁC�d�Ȃ邱�Ƃ��Ȃ��ꍇ��addVertex���g������
		//! @return bool �ǉ��ł������ǂ����C�ǉ��ł����ꍇ��true�C�ǉ��ł��Ȃ������ꍇ��false
		bool AddVertexCheckForDuplicates(const Vector2& v);


		//! @brief ���_���폜����֐��D�x���̂ő��p����ׂ��ł͂Ȃ�
		//! @param [in] index �폜���钸�_�̃C���f�b�N�X
		//! @note ���݂��Ȃ����_���w�肵���ꍇ�͉������Ȃ��D
		//! @note �폜�������_�̃C���f�b�N�X�͕ς��̂Œ��ӁD
		void RemoveVertex(const int index);

		//! @brief 1�ԍŌ�̒��_���폜����֐�
		//! @note ���_��1���Ȃ��ꍇ�͉������Ȃ��D
		constexpr void RemoveLastVertex()
		{
			if (GetVertexNum() == 0)
			{
				return;
			}

			--vertex_num;
		}

		//! @brief ���p�`�����Z�b�g����֐�
		//! @n ���_���폜���āC���_����0�ɂ���D
		constexpr void Reset() { vertex_num = 0; }

		//! @brief ���p�`�̒��_����Ԃ��֐�
		//! @return int ���p�`�̒��_��
		constexpr int GetVertexNum() const { return vertex_num; }

		//! @brief ���_�̍��W��Ԃ��֐�
		//! @param [in] i ���_�̃C���f�b�N�X
		//! @return Vector2 ���_�̍��W
		//! @n ���݂��Ȃ����_���w�肵���ꍇ��(0,0)��Ԃ��D
		constexpr Vector2 GetVertex(const int i) const
		{
			if (i < 0 || i >= GetVertexNum())
			{
				return Vector2{ 0, 0 };
			}

			return vertex[i];
		}

		//! @brief ���_�̍��W��Ԃ��֐��Dstd::optional���g���Ă���̂ŁC���݂��Ȃ����_���w�肵���ꍇ��std::nullopt��Ԃ��D
		//! @param [in] i ���_�̃C���f�b�N�X
		//! @return std::optional<Vector2> ���_�̍��W
		//! @n ���݂��Ȃ����_���w�肵���ꍇ��std::nullopt��Ԃ��D
		constexpr std::optional<Vector2> GetVertexOpt(const int i) const
		{
			if (i < 0 || i >= GetVertexNum())
			{
				return std::nullopt;
			}

			return vertex[i];
		}

		//! @brief ���_�̒��ōő��x���W��Ԃ��֐�
		//! @return float ���_�̒��ōő��x���W
		constexpr float GetMaxX() const 
		{
			float max_x = vertex[0].x;

			for (int i = 1; i < GetVertexNum(); ++i)
			{
				max_x = (std::max)(max_x, vertex[i].x);
			}

			return max_x;
		}

		//! @brief ���_�̒��ōŏ���x���W��Ԃ��֐�
		//! @return float ���_�̒��ōŏ���x���W
		constexpr float GetMinX() const 
		{
			float min_x = vertex[0].x;

			for (int i = 1; i < GetVertexNum(); ++i)
			{
				min_x = (std::min)(min_x, vertex[i].x);
			}

			return min_x;
		}

		//! @brief ���_�̒��ōő��y���W��Ԃ��֐�
		//! @return float ���_�̒��ōő��y���W
		constexpr float GetMaxY() const
		{
			float max_y = vertex[0].y;

			for (int i = 1; i < GetVertexNum(); ++i)
			{
				max_y = (std::max)(max_y, vertex[i].y);
			}

			return max_y;
		}

		//! @brief ���_�̒��ōŏ���y���W��Ԃ��֐�
		//! @return float ���_�̒��ōŏ���y���W
		constexpr float GetMinY() const
		{
			float min_y = vertex[0].y;

			for (int i = 1; i < GetVertexNum(); ++i)
			{
				min_y = (std::min)(min_y, vertex[i].y);
			}

			return min_y;
		}

		//! @brief ���p�`���ʂ��ǂ������ׂ�֐�
		//! @return bool �ʂȂ�true�C���Ȃ�false
		bool IsConvex() const;

		//! @brief �_�����p�`�̓����ɂ��邩�ǂ������ׂ�֐��D���p�`���ʂłȂ��ꍇ�͐���������ł��Ȃ��D
		//! @param [in] point ���ׂ�_
		//! @return bool �����ɂ���Ȃ�true�C�O���ɂ���Ȃ�false
		//! @note �_�����v���C�����v���̂����ꂩ�̏��ԂŒ��_������ł���K�v������D
		//! @note �_�����p�`�̕ӏ�ɂ���ꍇ�͓����ɂ���Ɣ��肷��D
		//! @note ���p�`���ʂłȂ��ꍇ�͐���������ł��Ȃ��D
		bool IsInside(const Vector2& point) const;


		//! @brief ���p�`�̃f�[�^�𕶎���ŏo�͂���
		//! @return std::string ���p�`�̃f�[�^�𕶎���ŏo�͂�������
		std::string ToString() const;

	private:

		std::array<Vector2, kMaxVertexNum> vertex;	//!< ���_���W

		int vertex_num;					//!< ���_��
	};


	// �o�̓X�g���[��
	template <class Char>
	inline std::basic_ostream<Char>& operator <<(std::basic_ostream<Char>& os, const Polygon2& poly)
	{
		os << poly.ToString();

		return os;
	}

}	// namespace designlab


#endif // DESIGNLAB_POLYGON2_H