//! @file interface_node_creator_builder
//! @brief INodeCreator�𐶐����邽�߂̃r���_�[�N���X

#ifndef DESIGNLAB_INTERFACE_NODE_CREATOR_BUILDER_H_
#define DESIGNLAB_INTERFACE_NODE_CREATOR_BUILDER_H_

#include <map>
#include <memory>

#include "devide_map_state.h"
#include "hexapod_next_move.h"
#include "interface_node_creator.h"
#include "interface_hexapod_coordinate_converter.h"
#include "interface_hexapod_state_presenter.h"
#include "interface_hexapod_vaild_checker.h"


//! @class INodeCreatorBuilder
//! @brief INodeCreator�𐶐����邽�߂̃r���_�[�N���X
class INodeCreatorBuilder
{
public:

	INodeCreatorBuilder() = default;
	virtual ~INodeCreatorBuilder() = default;

	//! @brief INodeCreator�𐶐�����
	//! @param [in] map �������ꂽ�}�b�v
	//! @param [in] calculator_ptr ���{�b�g�̏�Ԃ��v�Z����N���X
	//! @param [out] node_creator INodeCreator���i�[����map
	//! @n key:HexapodMove, value:INodeCreator
	//! @n �܂�C���{�b�g�̓���ɑΉ�����INodeCreator���i�[����K�v������
	virtual void Build(
		const DevideMapState& map, 
		const std::shared_ptr<const IHexapodCoordinateConverter>& converter_ptr,
		const std::shared_ptr<const IHexapodStatePresenter>& presenter_ptr,
		const std::shared_ptr<const IHexapodVaildChecker>& checker_ptr,
		std::map<HexapodMove, std::unique_ptr<INodeCreator> > *node_creator) const = 0;
};


#endif	// DESIGNLAB_INTERFACE_NODE_CREATOR_BUILDER_H_