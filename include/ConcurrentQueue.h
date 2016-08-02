#pragma once

#include <queue>
#include <boost/thread/mutex.hpp>


/// @class Thread safe implementation of a queue
template<typename Data>
class ConcurrentQueue
{
private:
    std::queue<Data> the_queue;
    mutable boost::mutex the_mutex;
    boost::condition_variable the_condition_variable;
    boost::atomic<int> len;
public:
    ConcurrentQueue()
    {
        len = 0;
    }

    void push(Data const& data)
    {
        boost::mutex::scoped_lock lock(the_mutex);
        the_queue.push(data);
        lock.unlock();
        the_condition_variable.notify_one();
        len++;
    }

    bool empty() const
    {
        boost::mutex::scoped_lock lock(the_mutex);
        return the_queue.empty();
    }

    bool try_pop(Data& popped_value)
    {
        boost::mutex::scoped_lock lock(the_mutex);
        if(the_queue.empty())
        {
            return false;
        }

        popped_value=the_queue.front();
        the_queue.pop();
        len--;
        return true;
    }

    void wait_and_pop(Data& popped_value)
    {
        boost::mutex::scoped_lock lock(the_mutex);
        while(the_queue.empty())
        {
            the_condition_variable.wait(lock);
        }

        popped_value=the_queue.front();
        the_queue.pop();
        len--;
    }

    int getSize()
    {
//        boost::mutex::scoped_lock lock(the_mutex);
//        int len = the_queue.size();
////        lock.unlock();
//        the_condition_variable.notify_one();
        return len;
    }

};
