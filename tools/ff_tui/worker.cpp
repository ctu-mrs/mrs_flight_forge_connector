#include "worker.h"

#include <exception>

Worker::Worker() : thread_([this] { Loop(); }) {
}

Worker::~Worker() {
  Stop();
}

void Worker::Stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stop_) {
      return;
    }
    stop_ = true;
    queue_.clear();
  }
  cv_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
}

void Worker::Post(std::function<void()> job, const std::string& key) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stop_) {
      return;
    }
    if (!key.empty()) {
      for (auto& queued : queue_) {
        if (queued.key == key) {
          queued.fn = std::move(job);
          return;
        }
      }
    }
    queue_.push_back(Job{key, std::move(job)});
  }
  cv_.notify_one();
}

bool Worker::HasKey(const std::string& key) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (running_ && running_key_ == key) {
    return true;
  }
  for (const auto& queued : queue_) {
    if (queued.key == key) {
      return true;
    }
  }
  return false;
}

bool Worker::Idle() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return !running_ && queue_.empty();
}

void Worker::Loop() {
  while (true) {
    Job job;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this] { return stop_ || !queue_.empty(); });
      if (stop_) {
        return;
      }
      job = std::move(queue_.front());
      queue_.pop_front();
      running_     = true;
      running_key_ = job.key;
    }
    try {
      job.fn();
    }
    catch (const std::exception&) {
      // jobs report their own errors; a stray throw must not kill the thread
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      running_ = false;
      running_key_.clear();
    }
  }
}
