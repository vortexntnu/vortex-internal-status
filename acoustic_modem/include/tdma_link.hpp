#include "am_driver.hpp"
#include "tdma_manager.hpp"

struct LinkTxMessage {
  MsgType type;
  std::vector<float> payload;
};

class TDMALink {
    
  public:
  TDMALink(AcousticModemDriver& driver, TDMAManager tdma);

  ~TDMALink();

  void start();

  void stop();

  // this is the queue of the transmission if it's not right slot
  void enqueue(MsgType type, std::vector<float> payload);

  private:
  void tx_worker();

  AcousticModemDriver& driver_;
  TDMAManager tdma_;

  std::atomic<bool> running_{false};

  mutable std::mutex tx_mtx_;
  std::condition_variable tx_cv_;
  std::queue<LinkTxMessage> tx_q_;

  std::thread tx_thread_;
}