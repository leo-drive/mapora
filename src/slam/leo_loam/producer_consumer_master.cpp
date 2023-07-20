#include "mapora/slam/leo_loam/producer_consumer_master.hpp"

namespace mapora
{
namespace slam
{
namespace leo_loam
{
using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::high_resolution_clock::time_point;

ProducerConsumerMaster::WorkerMessage::WorkerMessage(int queue_limit)
: queue_limit(queue_limit)
{
  queue = std::make_shared<TypeQueue>(queue_limit);
}

ProducerConsumerMaster::ProducerConsumerMaster(const size_t queue_limit)
: queue_limit_(queue_limit)
{
  all_should_die = std::make_shared<std::atomic<bool>>(false);
  WorkerMessage::Ptr message_first = std::make_shared<WorkerMessage>(queue_limit_);
  messages_.push_back(message_first);
}

void ProducerConsumerMaster::AddWorker(
  FuncType & function, const std::string & name, const bool & is_end)
{
  WorkerMessage::Ptr message_sub = std::make_shared<WorkerMessage>(queue_limit_);
  messages_.push_back(message_sub);
  size_t indice_last = messages_.size() - 1;

  std::future<void> future_of_worker = std::async(
    std::launch::async,
    &ProducerConsumerMaster::Worker,
    this,
    indice_last,
    name,
    messages_[indice_last - 1],
    messages_[indice_last],
    std::ref(all_should_die),
    function,
    is_end);
  futures_.emplace_back(std::move(future_of_worker));
}

void ProducerConsumerMaster::Worker(
  int indice_worker,
  const std::string & name_worker,
  WorkerMessage::Ptr msg_before,
  WorkerMessage::Ptr msg_after,
  const std::shared_ptr<std::atomic<bool>> & should_die,
  FuncType function,
  const bool & is_end)
{
  auto ending_procedure = [this, &indice_worker, &name_worker]() {
      std::lock_guard<std::mutex> lock(mutex_io_);
      std::cout << "Worker " << std::to_string(indice_worker) << " " << name_worker << " died." <<
        std::endl;
    };

  while (!*should_die) {
    WorkerMessage::TypeQueueContains queue_data;

    if (debug_detail_) {
      std::lock_guard<std::mutex> lock(mutex_io_);
      std::cout << "Worker " << std::to_string(indice_worker) << " " << name_worker <<
        " waiting." <<
        std::endl;
    }

    while (!msg_before->queue->read(queue_data)) {
      // spin until we get a value
      if (*should_die) {
        ending_procedure();
        return;
      }
    }
    if (debug_queue_size_) {
      std::lock_guard<std::mutex> lock(mutex_io_);
      std::cout << "Took an item from Queue " << name_worker <<
        " size: " << msg_before->queue->sizeGuess() << std::endl;
    }

    ProConMaterialBase::Ptr result;
    try {
      TimePoint time_point_start;
      if (debug_timing_) {time_point_start = Clock::now();}

      result = function(std::move(queue_data));

      std::chrono::duration<double, std::milli> duration{};
      if (debug_timing_) {
        TimePoint time_point_end = Clock::now();
        duration = time_point_end - time_point_start;
      }

      if (debug_results_ || debug_timing_) {
        std::lock_guard<std::mutex> lock(mutex_io_);
        std::cout << "Worker " << indice_worker << " " << name_worker;
        if (debug_timing_) {std::cout << " took: " << duration.count() << " ms." << std::endl;}
        if (debug_results_) {
          std::cout << "Result: " << std::endl;
          result->Log();
          std::cout << std::endl;
        }
      }
    } catch (std::exception & ex) {
      std::lock_guard<std::mutex> lock(mutex_io_);
      std::cerr << "Worker" << name_worker << " Exception: " << ex.what() << std::endl;
      continue;
    }


    // now pass the result to the next queue!
    if (!is_end) {
      while (!msg_after->queue->write(result)) {
        // spin until the queue has room
        if (*should_die) {
          ending_procedure();
          return;
        }
      }
    }
  }

  ending_procedure();
}

void ProducerConsumerMaster::PushTop(const ProConMaterialBase::Ptr & material)
{
  if (!is_running_) {return;}
  material->count = tally_message;
  struct CountIsSmallerThan
  {
    const size_t limit;

    explicit CountIsSmallerThan(size_t n)
    : limit(n) {}

    bool operator()(const WorkerMessage::Ptr & msg) const
    {
      return msg->queue->sizeGuess() < limit;
    }
  };
  if (std::all_of(messages_.begin(), messages_.end(), CountIsSmallerThan(queue_limit_))) {
    WorkerMessage::Ptr message_first = messages_.front();

    while (!message_first->queue->write(material)) {
      // spin until the queue has room
      if (all_should_die) {return;}
    }

  } else if (debug_detail_) {
    std::lock_guard<std::mutex> lock(mutex_io_);
    std::cout << "Skipped msg " << material->count << std::endl;
  }
  tally_message++;
}

void ProducerConsumerMaster::WaitUntilAllDone()
{
  for (auto & future : futures_) {
    future.get();
  }
  std::lock_guard<std::mutex> lock(mutex_io_);
  std::cout << "All workers died." << std::endl;
  is_running_ = false;
}

void ProducerConsumerMaster::KillAll() {*all_should_die = true;}

int ProducerConsumerMaster::get_queue_limit() const {return queue_limit_;}

void ProducerConsumerMaster::set_debug_queue_size(bool debug_queue_size)
{
  debug_queue_size_ = debug_queue_size;
}

void ProducerConsumerMaster::set_debug_results(bool debug_results)
{
  debug_results_ = debug_results;
}

void ProducerConsumerMaster::set_debug_detail(bool debug_detail) {debug_detail_ = debug_detail;}

void ProducerConsumerMaster::set_debug_timing(bool debug_timing) {debug_timing_ = debug_timing;}

void ProducerConsumerMaster::Start()
{
  is_running_ = true;
  std::lock_guard<std::mutex> lock(mutex_io_);
  std::cout << "Producer Consumer has started!" << std::endl;
}

bool ProducerConsumerMaster::get_is_running() {return is_running_;}
ProducerConsumerMaster::TypePushTop ProducerConsumerMaster::get_push_top()
{
  using std::placeholders::_1;
  return std::bind(&ProducerConsumerMaster::PushTop, this, _1);
}

}  // namespace leo_loam
}  // namespace slam
}  // namespace mapora
