﻿
//! @author    Hasegawa
//! @copyright © 埼玉大学 設計工学研究室 2023. All right reserved.

#include "graphic_const.h"

#include <Dxlib.h>


namespace designlab
{

const char GraphicConst::kWindowName[] = "HexapodGraphic";

const int GraphicConst::kColorBit = 32;

const int GraphicConst::kBackColorRed = 170;
const int GraphicConst::kBackColorGreen = 255;
const int GraphicConst::kBackColorBlue = 255;

const float GraphicConst::kCameraToTargetMax = 10000.0f;

const float GraphicConst::kCameraToTargetMin = 10.0f;

}  // namespace designlab
