#ifndef MW_RINGBUFFER_HPP
#define MW_RINGBUFFER_HPP


#include "MW_Common.hpp"
#include <array>

/**
 * @brief 一个通用的、固定大小、无动态内存分配的环形队列缓冲区 (Ring Buffer)
 * @tparam T 存储的元素类型
 * @tparam Size 缓冲区的最大容量
 * @note 这个实现不是线程安全的。如果在中断和主循环中同时访问，需要用户在外部处理临界区（例如关中断）。
 */
template <typename T, uint32_t Size>
class RingBuffer {
public:
    /**
     * @brief 构造函数，初始化环形缓冲区
     * @details 初始化头指针、尾指针和元素计数器为 0
     */
    RingBuffer() : head(0), tail(0), count(0) {};

    /**
     * @brief 向队列尾部压入一个元素
     * @param item 要压入的元素
     * @return 如果队列未满，成功压入则返回 MW_Status::SUCCESS；否则返回 MW_Status::ERROR 
     */
    MW_Status push(const T& item) {
        if (is_full()) {
            return MW_Status::RESOURCE_BUSY;
        }
        buffer[tail] = item;
        tail = (tail + 1) % Size;
        count++;
        return MW_Status::SUCCESS;
    }

    /**
     * @brief 从队列头部弹出一个元素
     * @param item 用于接收弹出元素的引用
     * @return 如果队列不为空，成功弹出则返回 MW_Status::SUCCESS；否则返回 MW_Status::ERROR 
     */
    MW_Status pop(T& item) {
        if (is_empty()) {
            return MW_Status::ERROR;
        }
        item = buffer[head];
        head = (head + 1) % Size;
        count--;
        return MW_Status::SUCCESS;
    }

    /**
     * @brief 查看队列头部的元素，但不弹出
     * @param item 用于接收元素的引用
     * @return 如果队列不为空，则返回 MW_Status::SUCCESS；否则返回 MW_Status::ERROR 
     */
    MW_Status peek(T& item) const {
        if (is_empty()) {
            return MW_Status::ERROR;
        }
        item = buffer[head];
        return MW_Status::SUCCESS;
    }

    /**
     * @brief 检查队列是否已满
     * @return 如果队列已满，则返回 true；否则返回 false
     */
    bool is_full() const {
        return count == Size;
    }

    /**
     * @brief 检查队列是否为空
     * @return 如果队列为空，则返回 true；否则返回 false
     */
    bool is_empty() const {
        return count == 0;
    }

    /**
     * @brief 获取当前队列中的元素数量
     * @return 当前队列中的元素数量
     */
    uint32_t size() const {
        return count;
    }

    /**
     * @brief 获取队列的最大容量    
     * @return 队列的最大容量
     */
    uint32_t capacity() const {
        return Size;
    }

private:
    std::array<T, Size> buffer;
    uint32_t head;
    uint32_t tail;
    uint32_t count;
};

#endif /* MW_RINGBUFFER_HPP */