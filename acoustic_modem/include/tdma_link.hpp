#include "tdma_manager.hpp"
#include "am_driver_iface.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <queue>
#include <thread>
#include <vector>

/*
 * Persistent commands are handled separately from normal reliable messages.
 *
 * - Normal messages:
 *   handshake + float payload + ACK/retransmission logic.
 *
 * - Persistent commands:
 *   standalone 16-bit control words (e.g. SURFACE, ABORT, STOP)
 *   transmitted periodically during the TDMA slot through the driver.
 *
 * While a persistent command is active, normal queued traffic is paused,
 * but ACKs are still allowed to pass.
 */

struct LinkTxMessage {
  MsgType type;
  std::vector<float> payload;
};

class TDMALink {
    
  public:
  TDMALink(IAcousticModemDriver& driver, TDMAManager& tdma);
  ~TDMALink();
  void start();
  void stop();

  void enqueue(MsgType type, std::vector<float> payload);

  void start_persistent_command(PersistentCmd cmd); 
  void stop_persistent_command();

  void on_data_received(MsgType type, uint16_t msg_id);
  void on_ack_received(MsgType type, uint16_t msg_id);

  private:
  void tx_worker();
  void send_new_message(const LinkTxMessage& msg);
  void resend_last_message();
  void send_pending_ack();

  IAcousticModemDriver& driver_;
  TDMAManager& tdma_;

  std::atomic<bool> running_{false};
  std::thread tx_thread_;
  mutable std::mutex tx_mtx_;
  std::queue<LinkTxMessage> tx_queue;

  //WAITING FOR ACK
  bool waiting_ack_=false;
  uint16_t waiting_msg_id_{0};
  MsgType waiting_type_{};
  LinkTxMessage last_sent_;
  std::chrono::steady_clock::time_point last_tx_time_;
  int retry_count_{0};
  struct PendingAck {
  MsgType type;
  uint16_t msg_id;
  };
  std::queue<PendingAck> pending_acks_;

  uint16_t last_rx_msg_id_{0};
  bool last_rx_valid_{false};

  std::optional<PersistentCmd> current_persistent_cmd_;
  std::chrono::steady_clock::time_point last_persistent_tx_;

  std::chrono::seconds ack_timeout_{50};
  int max_retries_{3};
};