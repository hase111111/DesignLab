﻿//! @file display_quality.h
//! @brief 描画の品質設定を示す列挙体．

#ifndef DESIGNLAB_DISPLAY_QUALITY_H_
#define DESIGNLAB_DISPLAY_QUALITY_H_


namespace designlab::enums
{

//! enum DisplayQuality
//! @brief 描画の品質設定を示す列挙体．
enum class DisplayQuality
{
	kLow,		//!< 低品質．
	kMedium,	//!< 中品質．
	kHigh,		//!< 高品質．
};

}	// namespace designlab::enums


#endif	// DESIGNLAB_DISPLAY_QUALITY_H_
