#pragma once
#include <memory>
#include "map_state.h"
#include "Target.h"
#include "graphic_data_broker.h"
#include "graphic_system.h"
#include "InterfacePassFinder.h"
#include "interface_graphic_main.h"


//! @class SystemMain
//! @date 2023/08/06
//! @author ���J��
//! @brief ���`��K�͂Ȑ݌v�ɂ����āCint main�ɂȂ�ł��l�ߍ��ނ킯�ɂ͂����Ȃ����߁C���̃N���X�ɂ܂Ƃ߂�D
//! @details �����̓��e������������Ƃ��ɂ́Cint main����ĂԃN���X��ς��邾���ł����D
class SystemMain final
{
public:
	SystemMain(std::unique_ptr<IPassFinder>&& graph_search);
	~SystemMain() = default;

	//! @brief ���܂܂�int main�ōs��ꂽ�������܂Ƃ߂����́D�ڕW�n�_�֒������C���e�v��Ɏ��s�����ꍇ�ɁC�V�~�����[�V�������I����D�K��̉񐔃V�~�����[�V����������I������D
	void main();

private:
	MapState m_map_state;
	STarget m_target;
	GraphicDataBroker m_broker;
	GraphicSystem m_graphic_system;
	std::unique_ptr<IPassFinder> mp_pass_finder;
};


//! @file system_main.h
//! @date 2023/08/06
//! @author ���J��
//! @brief ���̃v���O�����̏������܂Ƃ߂����́D�����̓��e��傫���ς������ꍇ��int main()����C�S���ʂ̃N���X���Ăׂ΂悢�D
//! @n �s�� : @lineinfo