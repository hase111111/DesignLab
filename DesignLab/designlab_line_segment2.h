//! @file designlab_line_segment2.h
//! @brief 2�����̐�����\���\����

#ifndef DESIGNLAB_LINE_SEGMENT2_H_
#define DESIGNLAB_LINE_SEGMENT2_H_


#include "designlab_vector2.h"
#include "designlab_math_util.h"


namespace designlab
{
	//! @struct LineSegment2
	//! @brief 2�����̐�����\���\����
	struct LineSegment2 final
	{
		LineSegment2() = default;
		constexpr LineSegment2(const Vector2& start, const Vector2& end) : start(start), end(end) {}
		constexpr LineSegment2(float startx, float starty, float endx, float endy) : start(startx, starty), end(endx, endy) {}
		constexpr LineSegment2(const LineSegment2& other) = default;
		constexpr LineSegment2(LineSegment2&& other) noexcept = default;
		constexpr LineSegment2& operator=(const LineSegment2& other) = default;


		constexpr bool operator==(const LineSegment2& other) const
		{
			return start == other.start && end == other.end;
		}

		constexpr bool operator!=(const LineSegment2& other) const { return !(*this == other); }


		//! @brief �����̒��������߂�֐�
		//! @return float �����̒���
		inline float GetLength() const
		{
			return (end - start).GetLength();
		}

		//! @brief ���������s���ǂ������ׂ�֐��D�S��constexpr�֐��ŏ����ł��邽�ߔ��ɍ����D
		//! @param[in] other ���̐���
		//! @return bool ���s�Ȃ�true�C�����łȂ����false
		constexpr bool IsParallel(const LineSegment2& other) const
		{
			//�O�ς�0�Ȃ畽�s
			return ::designlab::math_util::IsEqual(
				(end - start).Cross(other.end - other.start), 
				0.0f
			);
		}

		//! @brief ���̐����Ƃ̌�_�����߂�D
		//! @param [in] other ���̐���
		//! @return designlab::Vector2 ��_�D��_���Ȃ�or���s�ȏꍇ��(0, 0)��Ԃ��D
		//! @n �[�_��v�C�����s�̏ꍇ���l�����Ă��Ȃ��̂Œ���
		//! @note �Q�l�Fhttp://marupeke296.com/COL_main.html 
		Vector2 GetIntersection(const LineSegment2& other) const;

		//! @brief ���̐����ƌ�_�����݂��Ă��邩�ǂ������ׂ�֐�
		//! @param [in] other ���̐���
		//!	@return bool ��_������Ȃ�true�D�Ȃ�or���s�Ȃ�false
		bool HasIntersection(const LineSegment2& other) const;

		//! @brief ���̐����ƌ�_�����݂��Ă��邩�ǂ������ׁC��_��Ԃ��֐�
		//! @param [in] other ���̐���
		//! @param [out] intersection ��_
		//! @return bool ��_��1��������Ȃ�true�D�Ȃ�or�������d�Ȃ��Ă��Č�_�������ɂ���Ȃ�false
		bool CheckAndGetIntersection(const LineSegment2& other, Vector2* intersection) const;


		Vector2 start;	//!< �����̎n�_
		Vector2 end;	//!< �����̏I�_
	};

} //namespace designlab


#endif // !DESIGNLAB_LINE_SEGMENT2_H_