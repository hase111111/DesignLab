#pragma once
#include "MapState.h"
#include "GraphicDataBroker.h"
#include "GraphicSystem.h"
#include "NodeValidityChecker.h"
#include "Target.h"

//���̃v���O�����̏������܂Ƃ߂����́D�����̓��e��傫���ς������ꍇ��int main()����C�S���ʂ̃N���X���Ăׂ΂悢�D
class SystemMain final
{
public:
	SystemMain();
	~SystemMain() = default;

	void main();

private:
	MapState m_Map;
	STarget m_target;
	GraphicDataBroker m_Broker;
	GraphicSystem m_Graphic;
	NodeValidityChecker m_Checker;
};
