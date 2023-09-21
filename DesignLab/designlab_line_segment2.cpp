#include "designlab_line_segment2.h"

#include "cassert_define.h"
#include "designlab_math_util.h"


namespace dlm = ::designlab::math_util;

namespace designlab 
{

	SVector2 LineSegment2::GetIntersection(const LineSegment2& other) const
	{
		if (IsParallel(other))
		{
			return SVector2{ 0, 0 };	//���s�Ȃ�Ό�_�͑��݂��Ȃ��D
		}

		const SVector2 v1 = end - start;
		const SVector2 v2 = other.end - other.start;
		const SVector2 v3 = other.start - start;
		const float d = v1.Cross(v2);

		const float t1 = v3.Cross(v2) / d;
		const float t2 = v3.Cross(v1) / d;

		// t1, t2��0~1�͈͓̔��ɂ��邩�Ȃ�΁C��_�͐�����ɑ��݂���

		if (t1 < 0.0f - dlm::kAllowableError || t1 > 1.0f + dlm::kAllowableError || t2 < 0.0f - dlm::kAllowableError || t2 > 1.0f + dlm::kAllowableError)
		{
			return SVector2{ 0, 0 };	//��_�͐����̊O
		}

		return start + v1 * t1;
	}

	bool LineSegment2::HasIntersection(const LineSegment2& other) const
	{
		if (IsParallel(other))
		{
			return false;	//���s�Ȃ�Ό�_�͑��݂��Ȃ��D
		}

		const SVector2 v1 = end - start;
		const SVector2 v2 = other.end - other.start;
		const SVector2 v3 = other.start - start;
		const float d = v1.Cross(v2);

		const float t1 = v3.Cross(v2) / d;
		const float t2 = v3.Cross(v1) / d;

		// t1, t2��0~1�͈͓̔��ɂ��邩�Ȃ�΁C��_�͐�����ɑ��݂���

		if (t1 < 0.0f - dlm::kAllowableError || t1 > 1.0f + dlm::kAllowableError || t2 < 0.0f - dlm::kAllowableError || t2 > 1.0f + dlm::kAllowableError)
		{
			return false;	//��_�͐����̊O
		}

		return true;
	}

	bool LineSegment2::CheckAndGetIntersection(const LineSegment2& other, SVector2* intersection) const
	{
		// intersection��nullptr�ł����Ă͂Ȃ�Ȃ�
		assert(intersection != nullptr);

		if (IsParallel(other))
		{
			return false;	//���s�Ȃ�Ό�_�͑��݂��Ȃ��D
		}

		const SVector2 v1 = end - start;
		const SVector2 v2 = other.end - other.start;
		const SVector2 v3 = other.start - start;
		const float d = v1.Cross(v2);

		const float t1 = v3.Cross(v2) / d;
		const float t2 = v3.Cross(v1) / d;

		// t1, t2��0~1�͈͓̔��ɂ��邩�Ȃ�΁C��_�͐�����ɑ��݂���

		if (t1 < 0.0f - dlm::kAllowableError || t1 > 1.0f + dlm::kAllowableError || t2 < 0.0f - dlm::kAllowableError || t2 > 1.0f + dlm::kAllowableError)
		{
			return false;	//��_�͐����̊O
		}

		*intersection = start + v1 * t1;
		return true;
	}

}
