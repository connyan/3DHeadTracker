//
// Created by conny on 31/03/17.
//

#include <iostream>
#include "LockQueue.hpp"

void LockQueue::lock(int idThread) {
	using Clock = std::chrono::steady_clock;

	if (idThread >= queueSize) {
		throw std::exception();
	}

	queueLock.lock();
	if (waitQueue.empty()) {
		for (std::mutex& m : lockMutex) {
			m.try_lock();
		}
		waitQueue.push(idThread);
		queueLock.unlock();
	} else {
		waitQueue.push(idThread);
		queueLock.unlock();
		lockMutex[idThread].lock();
	}
}

LockQueue::LockQueue(int size) : queueSize{size} {
}

void LockQueue::unlock() {
	using Clock = std::chrono::steady_clock;
	queueLock.lock();

	waitQueue.pop();

	if (!waitQueue.empty()) {
		lockMutex[waitQueue.front()].unlock();
	} else {
		for (std::mutex& m : lockMutex) {
			m.unlock();
		}
	}

	queueLock.unlock();
}


