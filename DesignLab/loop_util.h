﻿
/// @file      loop_util.h
/// @author    hasegawa
/// @copyright (C) 2023 Design Engineering Laboratory, Saitama University All right reserved.

#ifndef DESIGNLAB_LOOP_UTIL_H_
#define DESIGNLAB_LOOP_UTIL_H_

#include <tuple>

namespace designlab
{

class DoubleIntRange final
{
public:
    class Iterator final
    {
    private:
        int i;
        int j;
        int j_max;

    public:
        Iterator(int i, int j, int j_max) : i(i), j(j), j_max(j_max) {}

        std::tuple<int, int> operator*() const
        {
            return std::make_tuple(i, j);
        }

        Iterator& operator++()
        {
            ++j;

            if (j == j_max)
            {
                ++i;
                j = 0;
            }

            return *this;
        }

        bool operator!=(const Iterator& other) const
        {
            return i != other.i || j != other.j;
        }
    };

    DoubleIntRange(const int i_max, const int j_max) :
        i_max_(i_max),
        j_max_(j_max)
    {
    }


    Iterator begin() const
    {
        return Iterator(0, 0, j_max_);
    }

    Iterator end() const
    {
        return Iterator(i_max_, 0, j_max_);
    }

private:
    int i_max_;
    int j_max_;
};

}  // namespace designlab

#endif  // DESIGNLAB_LOOP_UTIL_H_