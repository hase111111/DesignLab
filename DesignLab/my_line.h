#pragma once

#include "my_vector2.h"


namespace my_vec
{
	//! @struct SLine2
	//! @date 2023/08/06
	//! @auther ���J��
	//! @brief 2�����̐�����\���\����
	struct SLine2 final
	{
		SLine2() = default;
		constexpr SLine2(const SVector2& start, const SVector2& end) : start(start), end(end) {}


		bool operator==(const SLine2& other) const
		{
			return start == other.start && end == other.end;
		}


		//! @brief �����̒��������߂�֐�
		//! @return float �����̒���
		inline float getLength() const
		{
			return (end - start).length();
		}

		//! @brief ���������s���ǂ������ׂ�֐�
		//! @param[in] other ���̐���
		//! @return bool ���s�Ȃ�true�C�����łȂ����false
		constexpr bool isParallel(const SLine2& other) const
		{
			//�O�ς�p���Čv�Z����
			return my_math::isEqual((end - start).cross(other.end - other.start), 0.0f);
		}

		//! @brief ���̐����Ƃ̌�_�����߂�D
		//! @param [in] other ���̐���
		//! @return my_vec::SVector2 ��_�D��_���Ȃ�or���s�ȏꍇ��(0, 0)��Ԃ��D
		//! @note �Q�l�Fhttp://marupeke296.com/COL_main.html
		SVector2 getIntersection(const SLine2& other) const;

		//! @brief ��_�����݂��Ă��邩�ǂ������ׂ�֐�
		//! @param [in] other ���̐���
		//!	@return ��_������Ȃ�true�D�Ȃ�or���s�Ȃ�false
		bool hasIntersection(const SLine2& other) const;


		SVector2 start;	//!< �����̎n�_
		SVector2 end;	//!< �����̏I�_
	};

} //namespace my_vec


//! @file my_line.h
//! @date 2023/08/06
//! @auther ���J��
//! @brief 2�����̐�����\���\����
//! @n �s�� : @lineinfo
