#ifndef LEO_SLAM_PRODUCERCONSUMERMASTER_H
#define LEO_SLAM_PRODUCERCONSUMERMASTER_H

#include <future>
#include <queue>
#include <thread>
#include <mutex>
#include <memory>
#include <functional>
#include <iostream>
#include <chrono>
#include <algorithm>
#include <folly/ProducerConsumerQueue.h>

namespace mapora
{
namespace slam
{
namespace leo_loam
{
class ProConMaterialBase
{
public:
  using Ptr = std::shared_ptr<ProConMaterialBase>;
  unsigned long count{0};

  virtual void Log() {std::cout << "count: " << count << std::endl;}

  virtual ~ProConMaterialBase() = default;
};

class ProducerConsumerMaster
{
public:
  using SharedPtr = std::unique_ptr<ProducerConsumerMaster>;
  using FuncType = std::function<ProConMaterialBase::Ptr(ProConMaterialBase::Ptr)>;
  using TypePushTop = std::function<void (const ProConMaterialBase::Ptr &)>;

  ProducerConsumerMaster(const size_t queue_limit);

  void AddWorker(FuncType & function, const std::string & name, const bool & is_end);

  void PushTop(const ProConMaterialBase::Ptr & material);

  void KillAll();

  void WaitUntilAllDone();

  int get_queue_limit() const;

  void set_debug_queue_size(bool debug_queue_size);

  void set_debug_results(bool debug_results);

  void set_debug_detail(bool debug_detail);

  void set_debug_timing(bool debug_timing);

  bool get_is_running();

  void Start();

  TypePushTop get_push_top();

private:
  std::shared_ptr<std::atomic<bool>> all_should_die;

  struct WorkerMessage
  {
    explicit WorkerMessage(int queue_limit);
    using Ptr = std::shared_ptr<WorkerMessage>;
    using TypeQueueContains = ProConMaterialBase::Ptr;
    using TypeQueue = folly::ProducerConsumerQueue<TypeQueueContains>;
    std::shared_ptr<TypeQueue> queue;
    int queue_limit;
  };

  std::vector<WorkerMessage::Ptr> messages_;
  std::vector<std::future<void>> futures_;

  std::mutex mutex_io_;

  void Worker(
    int indice_worker,
    const std::string & name_worker,
    WorkerMessage::Ptr msg_before,
    WorkerMessage::Ptr msg_after,
    const std::shared_ptr<std::atomic<bool>> & should_die,
    FuncType function,
    const bool & is_end);

  int queue_limit_{3};
  unsigned long tally_message{0};
  bool debug_queue_size_{false};
  bool debug_results_{false};
  bool debug_detail_{false};
  bool debug_timing_{false};
  std::atomic<bool> is_running_{false};
};

}  // namespace leo_loam
}  // namespace slam
}  // namespace mapora

#endif  // LEO_SLAM_PRODUCERCONSUMERMASTER_H
