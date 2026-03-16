#ifndef THREADPOOL_H
#define THREADPOOL_H

#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <atomic>


class ThreadPool {
public:
    //线程启动，招募numthreads个服务员
    ThreadPool(size_t numThreads) : stop(false) {
        for (size_t i = 0; i <numThreads; ++i) {
            //每个服务员都是独立的线程（worker）
            workers.emplace_back([this] {
                while(true) {
                    std::function<void()> task;//准备接任务
                    {
                        //加锁：防止多个线程抢一个任务执行
                        std::unique_lock<std::mutex> lock(this->queue_mutex);

                        //服务员（worker）在这里等待，直到有新任务或者stop
                        this->condition.wait(lock,[this] {
                            return this->stop || !this->tasks.empty();
                        });

                        //如果餐厅倒闭且活干完了，退出线程（return）
                        if (this->stop && this->tasks.empty()) return;

                        //从队列中拿走最前面的任务
                        task = std::move(this->tasks.front());
                        this->tasks.pop();
                    }//离开作用域，解锁：拿到任务出锁，让其他服务员拿任务
                    task();//执行任务（执行具体工作代码）
                }
            });
        }
    }

    //迎宾员用来分配任务的函数
    template<class F>
    void enqueue(F&& f) {
        {
            //加锁：把任务防止桌子上，防止别人刚好在拿任务造成混乱
            std::unique_lock<std::mutex> lock(queue_mutex);
            tasks.emplace(std::forward<F>(f));
        }
        //唤醒一个正在等待的服务员
        condition.notify_one();
    }

    //析构函数：餐厅关门时的清理工作
    ~ThreadPool() {
        {
        std::unique_lock<std::mutex> lock(queue_mutex);
        stop = true;//宣布餐厅倒闭
        }
        condition.notify_all();//叫醒所有服务员，要下班了
        for(std::thread &worker : workers) {
            worker.join();//老板等待所有员工干完活
        }
    }

private:
    std::vector<std::thread> workers;//保存所有线程
    std::queue<std::function<void()>> tasks;//存放要执行的任务

    std::mutex queue_mutex;//保护任务的锁
    std::condition_variable condition;//唤醒线程的铃铛
    std::atomic<bool> stop;//餐厅关闭的标志

};
#endif