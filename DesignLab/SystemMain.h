#pragma once
#include "MapState.h"
#include "GraphicDataBroker.h"
#include "GraphicSystem.h"
#include "NodeValidityChecker.h"
#include "Target.h"


class SystemMain final
{
public:
	SystemMain();
	~SystemMain() = default;

	//! @brief ���܂܂�int main�ōs��ꂽ�������܂Ƃ߂����́D�ڕW�n�_�֒������C���e�v��Ɏ��s�����ꍇ�ɁC�V�~�����[�V�������I����D�K��̉񐔃V�~�����[�V����������I������D
	void main();

private:
	MapState m_Map;
	STarget m_target;
	GraphicDataBroker m_Broker;
	GraphicSystem m_Graphic;
	NodeValidityChecker m_Checker;
};


//! @file SystemMain.h
//! @brief ���̃v���O�����̏������܂Ƃ߂����́D�����̓��e��傫���ς������ꍇ��int main()����C�S���ʂ̃N���X���Ăׂ΂悢�D
//! @author ���J��

//! @class SystemMain
//! @brief ���`��K�͂Ȑ݌v�ɂ����āCint main�ɂȂ�ł��l�ߍ��ނ킯�ɂ͂����Ȃ����߁C���̃N���X�ɂ܂Ƃ߂�D
//! @details �����̓��e������������Ƃ��ɂ́Cint main����ĂԃN���X��ς��邾���ł����D
//! @author ���J��

