#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

// One background thread with a FIFO job queue. Each connection gets its own
// worker, so blocking socket calls never stall the UI and calls on one socket
// never overlap.
class Worker {
public:
  Worker();
  ~Worker();

  Worker(const Worker&)            = delete;
  Worker& operator=(const Worker&) = delete;

  // A non-empty key coalesces: a job still waiting with the same key is
  // replaced instead of queued twice (held-down keys, live preview, polling).
  void Post(std::function<void()> job, const std::string& key = "");

  // true when a job with this key is queued or running
  bool HasKey(const std::string& key) const;

  bool Idle() const;

  void Stop();

private:
  struct Job
  {
    std::string           key;
    std::function<void()> fn;
  };

  void Loop();

  mutable std::mutex      mutex_;
  std::condition_variable cv_;
  std::deque<Job>         queue_;
  std::string             running_key_;
  bool                    running_ = false;
  bool                    stop_    = false;
  std::thread             thread_;
};
