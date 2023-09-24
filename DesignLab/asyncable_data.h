//! @file asyncable_data.h
//! @brief �񓯊��������s���ۂɁC�f�[�^�̍X�V�񐔂ƃf�[�^���܂Ƃ߂Ĉ������߂̃N���X

#ifndef DESIGNLAB_ASYNCABLE_DATA_H_
#define DESIGNLAB_ASYNCABLE_DATA_H_


#include <vector>

#include <boost/thread.hpp>


// �e���v���[�g�̎����� .h�ɏ������ق��������炵�� https://qiita.com/i153/items/38f9688a9c80b2cb7da7


//! @class AsyncableData
//! @brief �񓯊��������s���ۂɁC�f�[�^�̍X�V�񐔂ƃf�[�^���܂Ƃ߂Ĉ������߂̍\����
//! @details ���̍\���̂́C�f�[�^�̍X�V�񐔂ƃf�[�^���܂Ƃ߂Ĉ������߂̍\���́D
//! @n �l�̕ύX���s���ۂɁC�f�[�^�̍X�V�񐔂��C���N�������g(++�̂���)���邱�ƂŁC�f�[�^�̍X�V�񐔂��J�E���g����
//! @n �܂��C�l�̎Q�ƂƕύX���s���ۂɃ~���[�e�b�N�X��p���āC�����ɕύX����邱�Ƃ�h���D�~���[�e�b�N�X�ɂ��Ă͈ȉ����Q��
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
template <typename T>
class AsyncableData
{
public:

	AsyncableData() : update_count_(0) {};
	explicit AsyncableData(const T& data) : data_(data), update_count_(0) {};

	//!< �R�s�[�E���[�u�͋֎~
	AsyncableData(const AsyncableData&) = delete;
	AsyncableData& operator=(const AsyncableData&) = delete;
	AsyncableData(AsyncableData&&) = delete;

	//! @breif �l���R�s�[���ĕԂ��D
	//! @n ���̎��Cread lock��������D
	//! @n ���R�C�f�[�^�̍X�V�񐔂̓C���N�������g����Ȃ��D
	//! @return T �l�̃R�s�[
	T data() const
	{
		//�ǂݎ��p�̃��b�N��������D���̃X�R�[�v { } �𔲂���܂Ń��b�N��������D(�܂肱�̊֐����I���܂�)
		boost::shared_lock<boost::shared_mutex> read_lock(mtx_);
		return data_;
	};


	//! @brief �l��ύX����D
	//! @n ���̎��Cwrite lock��������D
	//! @n �f�[�^�̍X�V�񐔂��C���N�������g����D
	//! @param [in] data �l
	void set_data(const T& data)
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
	int update_count() const
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


template <typename T>
class AsyncableData <std::vector<T>>
{
public:

	AsyncableData() : data_({}), update_count_(0) {};
	explicit AsyncableData(const std::vector<T>& data) : data_(data), update_count_(0) {}

	//!< �R�s�[�E���[�u�͋֎~
	AsyncableData(const AsyncableData&) = delete;
	AsyncableData& operator=(const AsyncableData&) = delete;
	AsyncableData(AsyncableData&&) = delete;

	//! @breif �l���R�s�[���ĕԂ��D
	//! @n ���̎��Cread lock��������D
	//! @n ���R�C�f�[�^�̍X�V�񐔂̓C���N�������g����Ȃ��D
	//!	@return T �l�̃R�s�[
	std::vector<T> data() const
	{
		//�ǂݎ��p�̃��b�N��������D���̃X�R�[�v { } �𔲂���܂Ń��b�N��������D(�܂肱�̊֐����I���܂�)
		boost::shared_lock<boost::shared_mutex> read_lock(mtx_);
		return data_;
	};

	//! @brief �l��ύX����D
	//! @n ���̎��Cwrite lock��������D
	//! @n �f�[�^�̍X�V�񐔂��C���N�������g����D
	//! @param [in] data �l
	void set_data(const std::vector<T>& data)
	{
		//�������ݗp�̃��b�N��������D�܂��́Cupgrade_lock��p�ӂ��āC�����unique_lock�ɕύX����D
		boost::upgrade_lock<boost::shared_mutex> upgrade_lock(mtx_);

		{
			boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(upgrade_lock);

			data_ = data;
			++update_count_;
		}
	};

	//! @brief push_back���s���D
	//! @n ���̎��Cwrite lock��������D
	//! @n �f�[�^�̍X�V�񐔂��C���N�������g����D
	//! @param [in] data �l
	void push_back(const T& data)
	{
		//�������ݗp�̃��b�N��������D�܂��́Cupgrade_lock��p�ӂ��āC�����unique_lock�ɕύX����D
		boost::upgrade_lock<boost::shared_mutex> upgrade_lock(mtx_);

		{
			boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(upgrade_lock);

			data_.push_back(data);
			++update_count_;
		}
	};

	//! @brief clean���s���D
	//! @n ���̎��Cwrite lock��������D
	//! @n �f�[�^�̍X�V�񐔂��C���N�������g����D
	//! @param [in] data �l
	void clean()
	{
		//�������ݗp�̃��b�N��������D�܂��́Cupgrade_lock��p�ӂ��āC�����unique_lock�ɕύX����D
		boost::upgrade_lock<boost::shared_mutex> upgrade_lock(mtx_);

		{
			boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(upgrade_lock);

			data_.clear();
			++update_count_;
		}
	};

	//! @brief size��Ԃ��D
	//! @n ���̎��Cread lock��������D
	//! @return size_t
	size_t size() const
	{
		//�ǂݎ��p�̃��b�N��������D���̃X�R�[�v { } �𔲂���܂Ń��b�N��������D(�܂肱�̊֐����I���܂�)
		boost::shared_lock<boost::shared_mutex> read_lock(mtx_);
		return data_.size();
	};

	int update_count() const
	{
		//�ǂݎ��p�̃��b�N��������D���̃X�R�[�v { } �𔲂���܂Ń��b�N��������D(�܂肱�̊֐����I���܂�)
		boost::shared_lock<boost::shared_mutex> read_lock(mtx_);
		return update_count_;
	};

private:

	mutable boost::shared_mutex mtx_;	//!< ���b�N�p��mutex

	std::vector<T> data_;

	int update_count_;
};


#endif