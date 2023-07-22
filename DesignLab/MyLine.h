#pragma once
#include "MyVector2.h"

namespace my_vec
{
	//! @struct SLine2
	//! @brief 2�����̐�����\���\����
	struct SLine2 final
	{
		SLine2() = default;
		constexpr SLine2(const SVector2& start, const SVector2& end) : start(start), end(end) {}

		SVector2 start;
		SVector2 end;

		//! @brief ���̐����Ƃ̌�_�����߂�D
		//! @param[in] other ���̐���
		//! @return ��_�D��_���Ȃ�or���s�ȏꍇ��(0, 0)��Ԃ��D
		//! @note �Q�l�Fhttp://marupeke296.com/COL_main.html
		SVector2 getIntersection(const SLine2& other) const;

		//! @brief ��_�����݂��Ă��邩�ǂ������ׂ�֐�
		//! @param[in] other ���̐���
		//!	@return ��_������Ȃ�true�D�Ȃ�or���s�Ȃ�false
		bool hasIntersection(const SLine2& other) const;

		//! @brief �����̒��������߂�֐�
		//! @return �����̒���
		inline float getLength() const
		{
			return (end - start).length();
		}

		//! @brief ���������s���ǂ������ׂ�֐�
		//! @param[in] other ���̐���
		constexpr bool isParallel(const SLine2& other) const
		{
			//�O�ς�p���Čv�Z����
			return my_math::isEqual((end - start).cross(other.end - other.start), 0.0f);
		}

		bool operator==(const SLine2& other) const
		{
			return start == other.start && end == other.end;
		}
	};
}