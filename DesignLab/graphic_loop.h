#pragma once

#include <memory>

#include "fps.h"
#include "interface_graphic_main.h"


//! @class GraphicLoop
//! @date 2023/08/08
//! @author ���J��
//! @brief GraphicSystem�N���X��while���[�v�̒��Ŗ��t���[���Ă΂�鏈�����������Ă���D������Q�[�����[�v�ł���D
//! @details �ǂ�ȕ`�揈�����s�������R���X�g���N�^�Ŏ󂯎��D������󂯎��Ȃ������ꍇ�C����������Ȃ��D
//! @n �ʏ�̃R���X�g���N�^�͌ĂׂȂ��悤�ɍ폜����Ă���C�K��std::unique_ptr<AbstractGraphicMain>�������ɂƂ�K�v������D@n
//! @n unique_ptr(���j�[�N�|�C���^)�ɂ��ẮC�Q�Ɓ� https://qiita.com/seriru13/items/06d044cbe5bcc44cca10
class GraphicLoop final
{
public:
	GraphicLoop() = delete;

	GraphicLoop(std::unique_ptr<IGraphicMain>&& graphic_main);
	~GraphicLoop() = default;


	//! @brief GraphicSystem�N���X��while���[�v�̒��Ŗ��t���[���Ă΂�鏈��
	//! @return bool mp_graphic_main��update�֐���false��Ԃ����ꍇ��Cmp_graphic_main��null�̏ꍇ�ȂǁC�`�揈�����I������ꍇ�Ƀ��[�v�𔲂��Cfalse��Ԃ�. 
	bool loop();

private:

	Fps m_fps;	//FPS�����ɐ��䂷��N���X�D�ڂ�����Fps.h��

	std::unique_ptr<IGraphicMain> mp_graphic_main;	//�`�揈�����s�����C���̃N���X�D
};


//! @file graphic_loop.h
//! @date 2023/08/08
//! @author ���J��
//! @brief �摜�`�揈���̃��C�����[�v�ŌĂ΂�鏈���ł���GraphicLoop�N���X
//! @n �s�� : @lineinfo
