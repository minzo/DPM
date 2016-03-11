//==============================================================================
//
// Thread Pool
//
//==============================================================================
#ifndef _THREAD_POOL_H_
#define _THREAD_POOL_H_

#include <thread>
#include <mutex>
#include <condition_variable>

#include <map>
#include <queue>
#include <vector>
#include <functional>

class ThreadPool
{
public:
    //--------------------------------------------------------------------------
    // @brief コンストラクタ
    // @param nThreads スレッド数
    //--------------------------------------------------------------------------
    ThreadPool(int nThreads_ = std::thread::hardware_concurrency()) : nThreads(nThreads_)
    {
        // スレッド内の処理
        auto worker = [](ThreadPool& threadPool)
        {
            std::function<void(int)> task;
            
            while(true)
            {
                {
                    std::unique_lock<std::mutex> lock(threadPool.mutexTaskQueue);
                    
                    // タスクが入るまで待機状態
                    while(threadPool.taskQueue.empty() && !threadPool.isDestruct)
                    {
                        // アイドル状態のスレッド数を1つ増やす
                        threadPool.nIdleThreads++;
                        
                        // 全スレッドアイドル状態なら Join によるブロッキング解除
                        if(threadPool.nIdleThreads == threadPool.nThreads)
                        {
                            threadPool.conditionThreadPoolJoin.notify_all();
                        }

                        // スレッドを待機状態にする
                        threadPool.condition.wait(lock);
                        
                        // アイドル状態のスレッド数を1つ減らす
                        threadPool.nIdleThreads--;
                    }
                    
                    // 終了処理
                    if(threadPool.taskQueue.empty() || threadPool.isDestruct)
                    {
                        return;
                    }
                    
                    // タスク取り出し
                    task = threadPool.taskQueue.front();
                    threadPool.taskQueue.pop();
                }
                
                // タスクの実行
                task(threadPool.thread_map[std::this_thread::get_id()]);
            }
        };
        
        // スレッド生成
        for(int i=0; i<nThreads; i++)
        {
            std::thread thread = std::thread(worker, std::ref(*this));
            thread_map[thread.get_id()] = i;
            threads.push_back(std::move(thread));
        }
    }
    
    //--------------------------------------------------------------------------
    // @brief デストラクタ
    //--------------------------------------------------------------------------
    ~ThreadPool()
    {
        // 終了フラグ
        isDestruct = true;
        
        // 全スレッドに通知
        condition.notify_all();
        
        // Join に通知
        conditionThreadPoolJoin.notify_all();
        
        // スレッド後始末
        for(int i = 0;i<threads.size(); i++)
        {
            threads[i].join();
        }
    }


    //--------------------------------------------------------------------------
    // @brief タスクを追加する
    //--------------------------------------------------------------------------
    void Request(std::function<void(int)> task)
    {
        {
            std::unique_lock<std::mutex> lock(mutexTaskQueue);

            taskQueue.push(task);
        }
        
        // 待機スレッドのどれか 1 つに通知して待機解除
        condition.notify_one();
    }

    //--------------------------------------------------------------------------
    // @brief 全てのスレッドがアイドル状態
    //--------------------------------------------------------------------------
    bool IsAllThreadIdle()
    {
        return nIdleThreads == nThreads;
    }
    
    //--------------------------------------------------------------------------
    // @brief 全スレッド数を返す
    //--------------------------------------------------------------------------
    int GetNumThread()
    {
        return nThreads;
    }

    //--------------------------------------------------------------------------
    // @brief アイドル状態のスレッド数を返す
    //--------------------------------------------------------------------------
    int GetNumThreadIdle()
    {
        return nIdleThreads;
    }
    
    //--------------------------------------------------------------------------
    // @brief スレッドプール内の処理が全て終了するのを待機する
    //--------------------------------------------------------------------------
    void Join()
    {
        std::mutex mutex;
        std::unique_lock<std::mutex> lock(mutex);
        
        // タスクが空かつ全スレッドがアイドル状態になるまで待つ
        while(!IsAllThreadIdle() || !taskQueue.empty())
        {
            if(isDestruct) break;
            
            conditionThreadPoolJoin.wait(lock);
        }
    }
    
private:
    
    // task queue
    std::queue<std::function<void(int)> > taskQueue;
    
    // mutex for task queue
    std::mutex mutexTaskQueue;
    
    // task queue 用の状態変数
    std::condition_variable condition;
    
    // スレッド
    std::vector<std::thread> threads;
    
    // スレッド番号
    std::map<std::thread::id, int> thread_map;
    
    // スレッド数
    int nThreads;
    
    // アイドル状態のスレッド数
    int nIdleThreads = 0;

    // スレッドプールの処理が終わるまで待機させるための状態変数
    std::condition_variable conditionThreadPoolJoin;
    
    
    // スレッドプールが破棄されるフラグ
    bool isDestruct = false;
};

#endif