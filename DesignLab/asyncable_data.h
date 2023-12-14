//! @file asyncable_data.h
//! @brief �񓯊��������s���ۂɁC�f�[�^�̍X�V�񐔂ƃf�[�^���܂Ƃ߂Ĉ������߂̃N���X�D


#ifndef DESIGNLAB_ASYNCABLE_DATA_H_
#define DESIGNLAB_ASYNCABLE_DATA_H_

#include <vector>

#include <boost/thread.hpp>


// �e���v���[�g�̎����̓w�b�_�[�ɏ����D https://qiita.com/i153/items/38f9688a9c80b2cb7da7


//! @class AsyncableData
//! @brief �񓯊��������s���ۂɁC�f�[�^�̍X�V�񐔂ƃf�[�^���܂Ƃ߂Ĉ������߂̃N���X�D(�R�s�[�E���[�u�͋֎~)
//! @details ���̍\���̂́C�f�[�^�̍X�V�񐔂ƃf�[�^���܂Ƃ߂Ĉ������߂̃N���X�D
//! @n �l�̕ύX���s���ۂɁC�f�[�^�̍X�V�񐔂��C���N�������g(++�̂���)���邱�ƂŁC�f�[�^�̍X�V�񐔂��J�E���g����D
//! @n �܂��C�l�̎Q�ƂƕύX���s���ۂɃ~���[�e�b�N�X��p���āC�����ɕύX����邱�Ƃ�h���D�~���[�e�b�N�X�ɂ��Ă͈ȉ����Q�ƁD
//! @n
//! @n [�񓯊������ɂ���] 
//! @n �񓯊����� (����E�����ɏ������s������) ���s���ۂɁC��̂Ƀf�[�^�ɓ����^�C�~���O�ő��삷��Ɗ댯(����`�����ɂȂ�C���������s�����s��ɂȂ�)�D
//! @n ���̃N���X�͂����h�����߂�boost::shared_mutex���g�p���Ă���D
//! @n �ڂ����� https://www.mathkuro.com/c-cpp/boost/how-to-use-boost-thread/#toc10 ��5�͂��Q�Ƃ��Ăق����D
//! @n ���̃N���X���ł�read lock, write lock���g���Ă���D 
//! @n �Q�l https://iorate.hatenablog.com/entry/20130222/1361538198 
//! @n 
//! @n �����o��m_mtx�ɂ��Ă���mutable �� const�ȃ����o�֐�(�����o�̒l��ύX�ł��Ȃ������o�֐�)�ɂ����Ă��ύX�ł���悤�ɂȂ郁���o�ϐ���\���D
//! @n �ʏ��Ύg���ׂ��ł͂Ȃ����C����̂悤�ȏꍇ(boost::shared_mutex���g���ꍇ)�͗L���I�D
//! @tparam T �񓯊��������s���f�[�^�D������s�����Ƃ��ł���^���w�肷�邱�ƁD
//! @n C++20�ł�concept���g�����ƂŁC������s�����Ƃ��ł���^���w��ł���D�A�b�v�f�[�g�����炻����g�p����p�ɕύX����D
template <typename T>
class AsyncableData
{
	static_assert(std::is_copy_assignable<T>::value, "������s�����Ƃ��ł���^���w�肵�Ă��������D");

public:

	AsyncableData() : update_count_(0) {};
	explicit AsyncableData(const T& data) : data_(data), update_count_(0) {};

	//!< �R�s�[�E���[�u�͋֎~
	AsyncableData(const AsyncableData&) = delete;
	AsyncableData& operator=(const AsyncableData&) = delete;
	AsyncableData(AsyncableData&&) = delete;

	//! @brief �l���R�s�[���ĕԂ��D
	//! @n ���̎��Cread lock��������D
	//! @n ���R�C�f�[�^�̍X�V�񐔂̓C���N�������g����Ȃ��D
	//! @return T �l�̃R�s�[
	T GetData() const
	{
		//�ǂݎ��p�̃��b�N��������D���̃X�R�[�v { } �𔲂���܂Ń��b�N��������D(�܂肱�̊֐����I���܂�)
		boost::shared_lock<boost::shared_mutex> read_lock(mtx_);
		return data_;
	};

	//! @brief �l��ύX����D
	//! @n ���̎��Cwrite lock��������D
	//! @n �f�[�^�̍X�V�񐔂��C���N�������g����D
	//! @param [in] data �Z�b�g����l�Dconst�Q�Ɠn�������D
	void SetData(const T& data)
	{
		//�������ݗp�̃��b�N��������D�܂��́Cupgrade_lock��p�ӂ��āC�����unique_lock�ɕύX����D
		boost::upgrade_lock<boost::shared_mutex> upgrade_lock(mtx_);

		{
			boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(upgrade_lock);

			data_ = data;
			++update_count_;
		}
	};

	//! @brief �f�[�^�̍X�V�񐔂�Ԃ��D
	//! @n ���̎��Cread lock��������D
	//! @n ���̒l�𒲂ׂāC�f�[�^�̍X�V�񐔂��ς���Ă��邩���m�F���邱�ƂŁC�f�[�^�̍X�V���K�v�����m�F����D
	//! @return int �f�[�^�̍X�V��
	int GetUpdateCount() const
	{
		//�ǂݎ��p�̃��b�N��������D���̃X�R�[�v { } �𔲂���܂Ń��b�N��������D(�܂肱�̊֐����I���܂�)
		boost::shared_lock<boost::shared_mutex> read_lock(mtx_);
		return update_count_;
	};

private:

	mutable boost::shared_mutex mtx_;	//!< ���b�N�p��mutex

	T data_;

	int update_count_;
};



//! @brief �񓯊��������s���ۂɁC�f�[�^�̍X�V�񐔂ƃf�[�^���܂Ƃ߂Ĉ������߂̍\���� (vector��)
//! @n �R�s�[�E���[�u�͋֎~
//! @details vector�ł�AsyncableData�Dvector������AsyncableData���쐬����ƁC�����炪�Ă΂��D
template <typename T>
class AsyncableData <std::vector<T> >
{
public:

	AsyncableData() : data_({}), update_count_(0) {};
	explicit AsyncableData(const std::vector<T>& data) : data_(data), update_count_(0) {}

	//!< �R�s�[�E���[�u�͋֎~
	AsyncableData(const AsyncableData&) = delete;
	AsyncableData& operator=(const AsyncableData&) = delete;
	AsyncableData(AsyncableData&&) = delete;

	//! @brief �l���R�s�[���ĕԂ��D
	//! @n ���̎��Cread lock��������D
	//! @n ���R�C�f�[�^�̍X�V�񐔂̓C���N�������g����Ȃ��D
	//!	@return T �l�̃R�s�[�D
	std::vector<T> GetData() const
	{
		//�ǂݎ��p�̃��b�N��������D���̃X�R�[�v { } �𔲂���܂Ń��b�N��������D(�܂肱�̊֐����I���܂�)
		boost::shared_lock<boost::shared_mutex> read_lock(mtx_);
		return data_;
	};

	//! @brief �l��ύX����D
	//! @n ���̎��Cwrite lock��������D
	//! @n �f�[�^�̍X�V�񐔂��C���N�������g����D
	//! @param [in] data �Z�b�g����l�Dconst�Q�Ɠn�������D
	void SetData(const std::vector<T>& data)
	{
		//�������ݗp�̃��b�N��������D�܂��́Cupgrade_lock��p�ӂ��āC�����unique_lock�ɕύX����D
		boost::upgrade_lock<boost::shared_mutex> upgrade_lock(mtx_);

		{
			boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(upgrade_lock);

			data_ = data;
			++update_count_;
		}
	};

	//! @brief �Ō���ɒl��ǉ�����D
	//! @n ���̎��Cwrite lock��������D
	//! @n �f�[�^�̍X�V�񐔂��C���N�������g����D
	//! @param [in] data ���ɒǉ�����l�Dconst�Q�Ɠn�������D
	void PushBack(const T& data)
	{
		//�������ݗp�̃��b�N��������D�܂��́Cupgrade_lock��p�ӂ��āC�����unique_lock�ɕύX����D
		boost::upgrade_lock<boost::shared_mutex> upgrade_lock(mtx_);

		{
			boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(upgrade_lock);

			data_.push_back(data);
			++update_count_;
		}
	};

	//! @brief �l�����ׂč폜����.
	//! @n ���̎��Cwrite lock��������D
	//! @n �f�[�^�̍X�V�񐔂��C���N�������g����D
	void Clean()
	{
		//�������ݗp�̃��b�N��������D�܂��́Cupgrade_lock��p�ӂ��āC�����unique_lock�ɕύX����D
		boost::upgrade_lock<boost::shared_mutex> upgrade_lock(mtx_);

		{
			boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(upgrade_lock);

			data_.clear();
			++update_count_;
		}
	};

	//! @brief size��Ԃ��D�v�f�̐���size_t�ŕԂ��D
	//! @n ���̎��Cread lock��������D
	//! @return size_t �v�f�̐��D
	size_t GetSize() const
	{
		//�ǂݎ��p�̃��b�N��������D���̃X�R�[�v { } �𔲂���܂Ń��b�N��������D(�܂肱�̊֐����I���܂�)
		boost::shared_lock<boost::shared_mutex> read_lock(mtx_);
		return data_.size();
	};

	//! @brief �f�[�^�̍X�V�񐔂�Ԃ��D
	//! @n ���̎��Cread lock��������D
	//! @n ���̒l�𒲂ׂāC�f�[�^�̍X�V�񐔂��ς���Ă��邩���m�F���邱�ƂŁC�f�[�^�̍X�V���K�v�����m�F����D
	//! @return int �f�[�^�̍X�V�񐔁D
	int GetUpdateCount() const
	{
		//�ǂݎ��p�̃��b�N��������D���̃X�R�[�v { } �𔲂���܂Ń��b�N��������D(�܂肱�̊֐����I���܂�)
		boost::shared_lock<boost::shared_mutex> read_lock(mtx_);
		return update_count_;
	};

private:

	mutable boost::shared_mutex mtx_;	//!< ���b�N�p��mutex�D

	std::vector<T> data_;

	int update_count_;
};


#endif	// DESIGNLAB_ASYNCABLE_DATA_H_