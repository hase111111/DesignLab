#include "designlab_line_segment2.h"

#include "cassert_define.h"
#include "designlab_math_util.h"


namespace dlm = ::designlab::math_util;

namespace designlab 
{

	Vector2 LineSegment2::GetIntersection(const LineSegment2& other) const
	{
		if (IsParallel(other))
		{
			return Vector2{ 0, 0 };	//平行ならば交点は存在しない．
		}

		const Vector2 v1 = end - start;
		const Vector2 v2 = other.end - other.start;
		const Vector2 v3 = other.start - start;
		const float d = v1.Cross(v2);

		const float t1 = v3.Cross(v2) / d;
		const float t2 = v3.Cross(v1) / d;

		// t1, t2が0~1の範囲内にあるかならば，交点は線分上に存在する

		if (t1 < 0.0f - dlm::kAllowableError || t1 > 1.0f + dlm::kAllowableError || t2 < 0.0f - dlm::kAllowableError || t2 > 1.0f + dlm::kAllowableError)
		{
			return Vector2{ 0, 0 };	//交点は線分の外
		}

		return start + v1 * t1;
	}

	bool LineSegment2::HasIntersection(const LineSegment2& other) const
	{
		if (IsParallel(other))
		{
			return false;	//平行ならば交点は存在しない．
		}

		const Vector2 v1 = end - start;
		const Vector2 v2 = other.end - other.start;
		const Vector2 v3 = other.start - start;
		const float d = v1.Cross(v2);

		const float t1 = v3.Cross(v2) / d;
		const float t2 = v3.Cross(v1) / d;

		// t1, t2が0~1の範囲内にあるかならば，交点は線分上に存在する

		if (t1 < 0.0f - dlm::kAllowableError || t1 > 1.0f + dlm::kAllowableError || t2 < 0.0f - dlm::kAllowableError || t2 > 1.0f + dlm::kAllowableError)
		{
			return false;	//交点は線分の外
		}

		return true;
	}

	bool LineSegment2::CheckAndGetIntersection(const LineSegment2& other, Vector2* intersection) const
	{
		// intersectionはnullptrであってはならない
		assert(intersection != nullptr);

		if (IsParallel(other))
		{
			return false;	//平行ならば交点は存在しない．
		}

		const Vector2 v1 = end - start;
		const Vector2 v2 = other.end - other.start;
		const Vector2 v3 = other.start - start;
		const float d = v1.Cross(v2);

		const float t1 = v3.Cross(v2) / d;
		const float t2 = v3.Cross(v1) / d;

		// t1, t2が0~1の範囲内にあるかならば，交点は線分上に存在する

		if (t1 < 0.0f - dlm::kAllowableError || t1 > 1.0f + dlm::kAllowableError || t2 < 0.0f - dlm::kAllowableError || t2 > 1.0f + dlm::kAllowableError)
		{
			return false;	//交点は線分の外
		}

		*intersection = start + v1 * t1;
		return true;
	}

}
