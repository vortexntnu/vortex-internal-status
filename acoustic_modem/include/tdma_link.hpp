#include "am_driver.hpp"
#include "tdma_manager.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <queue>
#include <thread>
#include <vector>

/**
 * TODO: hybrid part, should allow to send only short packet, so maybe only max 2 floats
 *       and ACKs. this is done in order to avoid exceeding the slot time
 */

struct LinkTxMessage {
  MsgType type;
  std::vector<float> payload;
};

class TDMALink {
    
  public:
  TDMALink(AcousticModemDriver& driver, TDMAManager& tdma);

  ~TDMALink();

  void start();

  void stop();

  // this is the queue of the transmission if it's not right slot
  void enqueue(MsgType type, std::vector<float> payload);

  void on_data_received(MsgType type, uint16_t msg_id);
  void on_ack_received(MsgType type, uint16_t msg_id);

  private:
  void tx_worker();
  void send_new_message(const LinkTxMessage& msg);
  void resend_last_message();
  void send_pending_ack();

  AcousticModemDriver& driver_;
  TDMAManager tdma_;

  std::atomic<bool> running_{false};

  std::thread tx_thread_;
  mutable std::mutex tx_mtx_;
  //std::condition_variable tx_cv_;
  std::queue<LinkTxMessage> tx_queue;

  //WAITING FOR ACK
  bool waiting_ack_=false;
  uint16_t waiting_msg_id_{0};
  MsgType waiting_type_{};
  LinkTxMessage last_sent_;
  std::chrono::steady_clock::time_point last_tx_time_;
  int retry_count_{0};

  // ACK to send
  /**
   * Done: we could need a queue and a struct for all the acks,
   *       if the rx is fast enough(improbable) we could lose some ack becasue of overwriting
   */
  // bool pending_ack_{false};
  // uint16_t pending_ack_msg_id_{0};
  // MsgType pending_ack_type_{};

  struct PendingAck {
  MsgType type;
  uint16_t msg_id;
  };
  std::queue<PendingAck> pending_acks_;

  // in case of ack lost we avoid sending the payload again by checking the last msg_id
  uint16_t last_rx_msg_id_{0};
  bool last_rx_valid_{false};

  std::chrono::seconds ack_timeout_{50};
  int max_retries_{3};
};