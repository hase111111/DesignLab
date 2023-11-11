//! @file graph_viewer_system_main.h
//! @brief �O���t��\������V�X�e���̃��C���N���X

#ifndef DESIGNLAB_GRAPH_VIEWER_SYSTEM_MAIN_H_
#define DESIGNLAB_GRAPH_VIEWER_SYSTEM_MAIN_H_

#include <memory>

#include "application_setting_recorder.h"
#include "graphic_data_broker.h"
#include "interface_pass_finder.h"
#include "interface_system_main.h"
#include "map_state.h"
#include "simulation_map_creator.h"
#include "stopwatch.h"


//! @class GraphViewerSystemMain
//! @brief �O���t��\������V�X�e���̃��C���N���X
//! @details ���̌����̎�@�ł͖؍\���̃O���t���쐬����D
//! �ǂ̂悤�ȃO���t���쐬����邩���m�F���邽�߂ɁC���̃O���t��\������V�X�e�����쐬�����D
class GraphViewerSystemMain final : public ISystemMain
{
public:

	GraphViewerSystemMain(
		std::unique_ptr<IPassFinder>&& pass_finder_ptr,
		const std::shared_ptr<GraphicDataBroker>& broker_ptr,
		const std::shared_ptr<const ApplicationSettingRecorder>& setting_ptr
	);

	//! @brief ���C���֐�
	void Main() override;

private:

	//! @brief �O���t���쐬����D
	//! @param [in] parent �e�m�[�h
	//! @param [out] graph �쐬�����O���t
	void CreateGraph(const RobotStateNode parent, std::vector<RobotStateNode>* graph);


	//! @brief �O���t�̃X�e�[�^�X��\������D
	//! @n �S�m�[�h���C�؂̐[���C�e�[�����Ƃ̃m�[�h����\������D
	//! @param [in] graph �O���t
	void OutputGraphStatus(const std::vector<RobotStateNode>& graph) const;

	//! @brief �}�b�v�����̃��[�h����͂���
	//! @return MapCreateMode �}�b�v�����̃��[�h
	MapCreateMode InputMapCreateMode() const;

	//! @brief �}�b�v�����̃I�v�V��������͂���
	//! @return unsigned int MapCreateOption �}�b�v�����̃I�v�V����
	unsigned int InputMapCreateOption() const;

	//! @brief �O���t�̒�����1�̃m�[�h��I������D�O���t����̏ꍇ�́C������Ԃ̃m�[�h��Ԃ��D
	//! @param [in] graph �O���t
	//! @return RobotStateNode �I�����ꂽ�m�[�h
	RobotStateNode SelectNode(const std::vector<RobotStateNode>& graph) const;


	std::unique_ptr<IPassFinder> pass_finder_ptr_;

	const std::shared_ptr<GraphicDataBroker> broker_ptr_;

	const std::shared_ptr<const ApplicationSettingRecorder> setting_ptr_;

	MapState map_state_;

	Stopwatch stopwatch_;
};


#endif // DESIGNLAB_GRAPH_VIEWER_SYSTEM_MAIN_H_