//
// Created by conny on 31/03/17.
//

#ifndef _3DHEADTRACKER_LOCKQUEUE_HPP
#define _3DHEADTRACKER_LOCKQUEUE_HPP

#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>

class LockQueue {
	int queueSize;
	std::mutex queueLock;
	std::queue<int> waitQueue;
	std::mutex lockMutex[2];
public:

	LockQueue(int size);
//	~LockQueue();
	void lock(int idThread);
	void unlock();


};


#endif //_3DHEADTRACKER_LOCKQUEUE_HPP
