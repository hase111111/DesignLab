﻿#ifndef DESIGNLAB_MATH_CONST_H_
#define DESIGNLAB_MATH_CONST_H_


namespace designlab
{

//float と double以外は作成しない．
template<typename T>
struct MathConst {};

template<>
struct MathConst<float>
{
	static constexpr float kPi = 3.14159265358979323846f;
	static constexpr float kAllowableError = 0.001f;
};

template<>
struct MathConst<double>
{
	static constexpr double kPi = 3.14159265358979323846;
	static constexpr double kAllowableError = 0.001;
};


} // namespace designlab

#endif //DESIGNLAB_MATH_CONST_H_