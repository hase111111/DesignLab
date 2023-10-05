//! @file fps_controller.h
//! @brief FPS�����ɃL�[�v���邽�߂̃N���X�D


#ifndef DESIGNLAB_FPS_CONTROLLER_H_
#define DESIGNLAB_FPS_CONTROLLER_H_


#include <list>


//! @class FpsController
//! @brief FPS�����ɃL�[�v���邽�߂̃N���X�D
//! @details ��{�I�ɂ͉������Ȃ��Ƃ�FPS�͈��ɂȂ�̂����ǁC144Fps�Ƃ�240Fps���炢�̃Q�[�~���O���j�^�[���g���Ă���ꍇ�C��ʂ����������Ȃ邱�Ƃ�����D
//! @n ����Ă��̃N���X���g����FPS�𐧌䂷��DFPS�� Frames per Second�F1�b������̉�ʍX�V�񐔂̂��ƁDFirst Person Shooting�̂��Ƃł͂Ȃ��D
//! @n ������PC�̃��j�^�[�̖��Ȃ̂ŁC�K�v�Ȃ��Ȃ�Ώ����Ă��܂��Ă����܂�Ȃ����C�������Ƃ���Ŗ��͂Ȃ��̂ŕ��u���Ă����Ă�����č\��Ȃ��D
//! @n
//! @n [�Q�l] 
//! @n �EFPS�̐�����s�� https://dixq.net/rp2/07.html �����̃T�C�g�̃v���O�������Q�l�ɂ��Ă��邪�C�@�\��F�X�ǉ����Ă���D
//! @n �EDxlib��FPS�ƃ��t���b�V�����[�g�ɂ��� https://dixq.net/forum/viewtopic.php?t=20224
class FpsController final
{
public:
	FpsController() = delete;
	FpsController(int target_fps);

	//! @brief ��������������ꍇ�CFPS�����ɂ��邽�߂ɑ҂D
	void Wait();

	//! @brief 60Hz�ȏ�̃��j�^�[�g�p���ɏ������l�܂��ĉ�ʂ�������Ȃ��悤�ɁC�`�揈�����X�L�b�v���邩�ǂ����𔻒肷��D
	//! @return bool �������l�܂��ĕ`����΂������Ƃ���true��Ԃ��D���̌�t���O��false�ɂ���D
	bool SkipDrawScene();

private:

	//! @brief ���݂̎������L�^����֐�
	//! @param [in] now_time ���݂̎���(�~���b) 
	void RegistTime(int now_time);

	//! @brief �ǂꂾ���҂Ă΂悢���Ԃ��֐��D
	//! @param [out] wait_time �҂ׂ�����
	//! @return bool �R�}�������Ă���ꍇ��false
	bool CheckNeedSkipDrawScreen(int* wait_time) const;

	// @brief �ڕW��FPS�����������ǂ����𔻒肷��֐�
	// @return bool ���̒l�C�܂���60���傫���l�ł����false
	bool TargetFpsIsVaild() const;


	const int kTargetFpsValue;	//!< �ڕW��FPS�C�����l�� 60 or 30�DDxlib�̎g�p�� 60 ������l�𐄏����Ȃ��D

	const int kOneFrameTime;	//!< 1�t���[��������ɂ����鎞��(�~���b)

	const int kListMax;			//!< ���X�g��2�b���̃t���[�����Ƃɂ����������Ԃ��L�^���邽�߁C���X�g�̍ő�T�C�Y�����߂�D


	std::list<int> time_list_;		//!< 1�t���[�����Ƃɂ����������Ԃ��L�^���郊�X�g�D

	bool need_skip_draw_screen_;	//!< �R�}�������������邽�߂̃t���O�Dtrue�ł���� 1�t���[���`����΂��C�t���O��܂�
};


#endif // DESIGNLAB_FPS_CONTROLLER_H_