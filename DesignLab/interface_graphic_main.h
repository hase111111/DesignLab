//! @file interface_graphic_main.h
//! @brief GraphicMain�̃C���^�[�t�F�[�X

#ifndef DESIGNLAB_INTERFACE_GRAPHIC_MAIN_H_
#define DESIGNLAB_INTERFACE_GRAPHIC_MAIN_H_


//! @class IGraphicMain
//! @brief GraphicMain�̃C���^�[�t�F�[�X
//! @details �`��̏������s���N���X�͕K�����̃N���X���p�����邱�ƁD�t�Ɍ����Όp������Ύ��R�ɏ����������\�ɂȂ�D
//! @n �Ƃ������C�����\�ȃO���t�B�b�N�̃p�[�c����邽�߂̋��^�����̃N���X���Ƃ������ƁD
class IGraphicMain
{
public:

	IGraphicMain() = default;
	virtual ~IGraphicMain() = default;


	//! @brief �`���ʂ̍X�V���s���D�������z�֐��̂��߁C�p����ł͕K��override����K�v������D
	//! @return bool ���[�v�𔲂��C�O���t�B�b�N�̕\�����I������Ȃ��false���������D
	virtual bool Update() = 0;

	//! @brief �`����s���D�����ł͕`��n�̏����݂̂��s�������̃f�[�^���X�V���Ȃ�����const��t���Ă���D�������z�֐��D
	virtual void Draw() const = 0;

};


#endif // !DESIGNLAB_INTERFACE_GRAPHIC_MAIN_H_