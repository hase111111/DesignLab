#include "pch.h"

#include "../DesignLab/leg_state.h"


namespace dllf = ::designlab::leg_func;


namespace
{
	//! @brief std::arrayを同じ値で埋める
	template<typename T, size_t S>
	std::array<T, S> MakeArraySameValue(const T& value)
	{
		std::array<T, S> res;
		res.fill(value);
		return res;
	}
}


namespace designlab::test::node::leg_state
{
	TEST(LegFuncTest, ChangeLegStateTest)
	{
	}

}	// namespace designlab::test::node::leg_state