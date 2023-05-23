#include "GraphicConst.h"
#include "DxLib.h"

const std::string GraphicConst::WIN_NAME = TEXT("PhantomXGraphic");

const int GraphicConst::WIN_X = 1280;

const int GraphicConst::WIN_Y = 720;

const int GraphicConst::COLOR_BIT = 32;

const float GraphicConst::CAMERA_FAR = 3000.0f;

const float GraphicConst::CAMERA_NEAR = 0.1f;

const float GraphicConst::CAMERA_TO_TARGET_MAX = GraphicConst::CAMERA_FAR * 2.0f / 3.0f;
