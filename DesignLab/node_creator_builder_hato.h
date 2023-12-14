﻿//! @file node_creator_builder_hato.h
//! @brief 波東さんが行った処理と同様の方法になるようにノード生成クラスを生成するクラス．

#ifndef DESIGNLAB_NODE_CREATOR_BUILDER_HATO_H_
#define DESIGNLAB_NODE_CREATOR_BUILDER_HATO_H_

#include "interface_node_creator_builder.h"


//! @class NodeCreatorBuilderHato
//! @brief 波東さんが行った処理と同様の方法になるようにノード生成クラスを生成するクラス．
class NodeCreatorBuilderHato final : public INodeCreatorBuilder
{
public:

	NodeCreatorBuilderHato(
		const std::shared_ptr<const IHexapodCoordinateConverter>& converter_ptr,
		const std::shared_ptr<const IHexapodStatePresenter>& presenter_ptr,
		const std::shared_ptr<const IHexapodVaildChecker>& checker_ptr
	);

	void Build(
		const DevideMapState& map, 
		std::map<::designlab::enums::HexapodMove, std::unique_ptr<INodeCreator> >* node_creator) const override;

private:
	const std::shared_ptr<const IHexapodCoordinateConverter> converter_ptr_;
	const std::shared_ptr<const IHexapodStatePresenter> presenter_ptr_;
	const std::shared_ptr<const IHexapodVaildChecker> checker_ptr_;
};


#endif // DESIGNLAB_NODE_CREATOR_BUILDER_HATO_H_