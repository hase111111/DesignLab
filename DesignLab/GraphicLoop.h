#pragma once
#include "Fps.h"
#include "InterfaceGraphicMain.h"
#include <memory>

class GraphicLoop final
{
public:
	GraphicLoop() = delete;

	GraphicLoop(std::unique_ptr<IGraphicMain>&& mp_graphic_main);
	~GraphicLoop() = default;

	
	//! GraphicSystem�N���X��while���[�v�̒��Ŗ��t���[���Ă΂�鏈�� <br> <br> [�`��̏����ɂ���]<br>
	//! ScreenFlip�֐���ClearDrawScreen�֐��̏ڍׁF�E�B���h�E�̉摜�\���̓p���p������̗l�ɉ�ʂ�f�����؂�ւ��邱�ƂŃA�j���[�V�������Č����Ă���D<br>
	//! �������C�P�ɉ�ʂ�؂�ւ����ꍇ�C�{���̃p���p������̗l�ɃE�B���h�E�ɂ�������łĂ��܂��D<br>
	//! ������GraphicSystem�N���X��dxlibInit�֐��̒��ŌĂ΂�Ă��� SetDrawScreen(DX_SCREEN_BACK) �ɂ���Ă������񗠉�ʂɊG��`�悵�Ă���C
	//! ScreenFlip�֐��ŃE�B���h�E�ɊG��߂����Ƃŉ�ʂ̂�������Ȃ����Ă���D<br>
	//! ClearDrawScreen �� ScreenFlip �� ProcessMessage�ƕԂ��l�������Ȃ̂ŁCloop�֐��̗l�ȏ������ƂȂ�D<br>
	//! @return bool mp_GraphicMain��update�֐���false��Ԃ����ꍇ�C������mp_GraphicMain�N���X��null�̏ꍇfalse��Ԃ����[�v�𔲂���. 
	bool loop();

private:

	Fps m_Fps;	//FPS�����ɐ��䂷��N���X�D�ڂ�����Fps.h��

	std::unique_ptr<IGraphicMain> mp_GraphicMain;	//�`�揈�����s�����C���̃N���X�D
};

//! @class GraphicLoop
//! @brief GraphicSystem�N���X��while���[�v�̒��Ŗ��t���[���Ă΂�鏈�����������Ă���D������Q�[�����[�v�ł���D
//! @details �ǂ�ȕ`�揈�����s�������R���X�g���N�^�Ŏ󂯎��D������󂯎��Ȃ������ꍇ�C����������Ȃ��D<br>
//! �ʏ�̃R���X�g���N�^�͌ĂׂȂ��悤�ɍ폜����Ă���C�K��std::unique_ptr<AbstractGraphicMain>�������ɂƂ�K�v������D<br> <br>
//! unique_ptr(���j�[�N�|�C���^)�ɂ��ẮC�Q�Ɓ� https://qiita.com/seriru13/items/06d044cbe5bcc44cca10
//! @author ���J��

//! @file GraphicLoop.h
//! @brief GraphicLoop�N���X�̎������s���Ă���D
//! @author ���J��
