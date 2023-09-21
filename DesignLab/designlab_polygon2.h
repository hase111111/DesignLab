//! @file designlab_polygon2.h
//! @brief ���p�`��\���\����


#ifndef DESIGNLAB_POLYGON2_H
#define DESIGNLAB_POLYGON2_H


#include <string>
#include <vector>

#include "designlab_vector2.h"


namespace designlab
{

	static constexpr int MAX_VERTEX_NUM = 6;	//!< ���x�𑁂����邽�߂ɂ��炩���ߍő�T�C�Y�����肵�Ă����D


	//! @struct designlab::Polygon2
	//! @brief 2�����̑��p�`��\���\����
	//! @details 2�����̑��p�`��\���\���́D
	//! @n ���_�͔����v���C�����͎��v���ɕ���ł���K�v������D
	//! @n �����łȂ��ꍇ�͂������̊֐�������ɓ��삵�Ȃ��D
	//! @n
	//! @n �܂��C���_�̐��͍ő��MAX_VERTEX_NUM�܂ł����o�^�ł��Ȃ��D
	//! @n ����͓���̍������̂��߂Ɋm�ۂ����z��̃T�C�Y���Œ肵�Ă��邽�߂ł���D
	//! @n ���̒l��ύX����ꍇ�́C�R�[�h����MAX_VERTEX_NUM�̒l��ύX����K�v������D
	//! @n
	//! @n �Ȃ��C�R�[�h����max�֐��Cmin�֐��ɂ��Ă͈ȉ����Q�ƁD
	//! @n �Q�� : https://cpprefjp.github.io/reference/algorithm/max.html
	struct Polygon2 final
	{

		Polygon2() { vertex.resize(MAX_VERTEX_NUM); };


		//! @brief ���_��ǉ�����֐�
		//! @param [in] v �ǉ����钸�_
		//! @note ���̒��_�Əd�Ȃ��Ă���ꍇ�ł��ǉ�����D
		inline void AddVertex(const SVector2& v)
		{
			vertex[vertex_num] = v;
			++vertex_num;
		}

		//! @brief ���_��ǉ�����֐��D���̒��_�Əd�Ȃ��Ă���ꍇ�͒ǉ����Ȃ�
		//! @param[in] v �ǉ����钸�_
		//! @note ���̒��_�Əd�Ȃ��Ă���ꍇ�͒ǉ����Ȃ��D���̏����̕������x���Ȃ�̂ŁC�d�Ȃ邱�Ƃ��Ȃ��ꍇ��addVertex���g������
		//! @return bool �ǉ��ł������ǂ����C�ǉ��ł����ꍇ��true�C�ǉ��ł��Ȃ������ꍇ��false
		bool AddVertexCheckForDuplicates(const SVector2& v);


		//! @brief ���_���폜����֐��D�x���̂ő��p����ׂ��ł͂Ȃ�
		//! @param [in] i �폜���钸�_�̃C���f�b�N�X
		//! @note ���݂��Ȃ����_���w�肵���ꍇ�͉������Ȃ��D
		//! @note �폜�������_�̃C���f�b�N�X�͕ς��̂Œ��ӁD
		void RemoveVertex(const int i);

		//! @brief 1�ԍŌ�̒��_���폜����֐�
		//! @note ���_��1���Ȃ��ꍇ�͉������Ȃ��D
		inline void RemoveLastVertex()
		{
			if (GetVertexNum() == 0)
			{
				return;
			}

			--vertex_num;
		}

		//! @brief ���p�`�����Z�b�g����֐�
		inline void Reset()
		{
			vertex_num = 0;
		}


		//! @brief ���p�`�̒��_����Ԃ��֐�
		//! @return int ���p�`�̒��_��
		constexpr int GetVertexNum() const { return vertex_num; }

		//! @brief ���_�̍��W��Ԃ��֐�
		//! @param [in] i ���_�̃C���f�b�N�X
		//! @return SVector2 ���_�̍��W
		//! @note ���݂��Ȃ����_���w�肵���ꍇ��(0,0)��Ԃ��D
		inline SVector2 GetVertex(int i) const
		{
			if (i < 0 || i >= GetVertexNum())
			{
				return SVector2{ 0, 0 };
			}

			return vertex[i];
		}

		//! @brief ���_�̒��ōő��x���W��Ԃ��֐�
		//! @return float ���_�̒��ōő��x���W
		float GetMaxX() const;

		//! @brief ���_�̒��ōŏ���x���W��Ԃ��֐�
		//! @return float ���_�̒��ōŏ���x���W
		float GetMinX() const;

		//! @brief ���_�̒��ōő��y���W��Ԃ��֐�
		//! @return float ���_�̒��ōő��y���W
		float GetMaxY() const;

		//! @brief ���_�̒��ōŏ���y���W��Ԃ��֐�
		//! @return float ���_�̒��ōŏ���y���W
		float GetMinY() const;

		//! @brief ���p�`���ʂ��ǂ������ׂ�֐�
		//! @return bool �ʂȂ�true�C���Ȃ�false
		bool IsConvex() const;

		//! @brief �_�����p�`�̓����ɂ��邩�ǂ������ׂ�֐��D���p�`���ʂłȂ��ꍇ�͐���������ł��Ȃ��D
		//! @param [in] point ���ׂ�_
		//! @return bool �����ɂ���Ȃ�true�C�O���ɂ���Ȃ�false
		//! @note �_�����v���C�����v���̂����ꂩ�̏��ԂŒ��_������ł���K�v������D
		//! @note �_�����p�`�̕ӏ�ɂ���ꍇ�͓����ɂ���Ɣ��肷��D
		//! @note ���p�`���ʂłȂ��ꍇ�͐���������ł��Ȃ��D
		bool IsInside(const SVector2& point) const;


		//! @brief ���p�`�̃f�[�^�𕶎���ŏo�͂���
		//! @return std::string ���p�`�̃f�[�^�𕶎���ŏo�͂�������
		std::string ToString() const;

	private:

		std::vector<SVector2> vertex;	//!< ���_���W
		int vertex_num = 0;				//!< ���_��

	};


	// �o�̓X�g���[��
	template <class Char>
	inline std::basic_ostream<Char>& operator <<(std::basic_ostream<Char>& os, const Polygon2& v)
	{
		os << "Vertex Num : " << v.GetVertexNum() << "\n";

		for (int i = 0; i < v.GetVertexNum(); ++i)
		{
			os << "Vertex " << i << " : " << v.GetVertex(i) << "\n";
		}

		os << "Max X : " << v.GetMaxX() << "\n";
		os << "Min X : " << v.GetMinX() << "\n";
		os << "Max Y : " << v.GetMaxY() << "\n";
		os << "Min Y : " << v.GetMinY() << "\n";

		os << "Convex :" << (v.IsConvex() ? "True" : "False") << "\n";

		return os;
	}

}	// namespace designlab


#endif // !DESIGNLAB_POLYGON2_H