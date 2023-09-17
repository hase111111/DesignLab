//#pragma once
//
//#include <memory>
//#include <vector>
//
//#include "button_controller.h"
//#include "graphic_const.h"
//#include "application_setting_recorder.h"
//#include "node.h"
//
//
////! @enum ENodeDisplayNode
////! @date 2023/08/09
////! @author ���J��
////! @brief �m�[�h�̕\�����@��\���񋓌^�D
//enum class ENodeDisplayNode : int
//{
//	AUTO_UPDATE,
//	ALWAYS_NEW,
//	SELECTABLE
//};
//
//
////! @class GUIController
////! @date 2023/08/09
////! @author ���J�� 
////! @brief UI��\������֐��D�\��������̂�ύX�������ꍇ�͕ҏW���Ă��������D
//class GUIController
//{
//public:
//
//	GUIController() = delete;
//	GUIController(const SApplicationSettingRecorder* const setting);
//
//	void Update(const int max_node, int& display_node, const int counter);
//
//	void Draw(const SNode& node) const;
//
//private:
//
//	//�����Ƀm�[�h�̏�Ԃ�`�悷��֐��D��ω����֐��ł��D�\����Ȃ�
//	void drawNodeByStr(const SNode node) const;
//
//	//�E���Ƀ{�^���̎g������`�悷��֐��D����
//	void drawExplanationByStr() const;
//
//	const int BOX_X;
//	const int BOX_Y;
//	const int CENTER_X;
//	const int CENTER_Y;
//
//	const int CHANGE_NEXT_NODE;	//���̃m�[�h���Đ�����܂ł̎��ԁD
//
//	const SApplicationSettingRecorder* const mp_setting;
//
//
//	bool m_is_displayed = true;		//UI��\�����邩�ǂ����D
//
//	ENodeDisplayNode m_camera_view_mode = ENodeDisplayNode::SELECTABLE;		//�ǂ̂悤�Ƀm�[�h��\�����邩
//
//	std::vector<std::unique_ptr<ButtomController>> m_buttom;		//�{�^���̊Ǘ��N���X�D
//};
//
//
////! @file gui_controller.h
////! @date 2023/08/09
////! @author ���J��
////! @brief GUI��\������N���X�D
////! @n �s�� : @lineinfo
