#include "tdma/tdma_link.hpp"
#include <rclcpp/rclcpp.hpp>

TDMALink::TDMALink(IAcousticModemDriver& driver, TDMAManager& tdma): driver_(driver), tdma_(tdma){}

TDMALink::~TDMALink(){
    stop();
}

void TDMALink::start(){
    running_=true;
    tx_thread_ = std::thread(&TDMALink::tx_worker, this);
}

void TDMALink::stop() {
    running_ = false;

    if (tx_thread_.joinable()) {
        tx_thread_.join();
    }
}

void TDMALink::enqueue(MsgType type, std::vector<float> payload){
    std::lock_guard<std::mutex>lock(tx_mtx_);
    tx_queue.push(LinkTxMessage{type, payload});
}

void TDMALink::start_persistent_command(PersistentCmd cmd){
    std::lock_guard<std::mutex> lock(tx_mtx_);
    current_persistent_cmd_=cmd;
    last_persistent_tx_=std::chrono::steady_clock::now()-std::chrono::milliseconds(2000);
}

void TDMALink::stop_persistent_command(){
    std::lock_guard<std::mutex> lock(tx_mtx_);
    current_persistent_cmd_.reset();
}

void TDMALink::on_data_received(MsgType type, uint16_t msg_id){
    std::lock_guard<std::mutex>lock(tx_mtx_);
    pending_acks_.push(PendingAck{type,msg_id});

    if (msg_id == last_rx_msg_id_ && last_rx_valid_) {
        return;
    }
    last_rx_valid_=true;
    last_rx_msg_id_ = msg_id;
}

void TDMALink::on_ack_received(MsgType type, uint16_t msg_id){
    std::lock_guard<std::mutex>lock(tx_mtx_);
    if(waiting_ack_ && waiting_msg_id_==msg_id){
        waiting_ack_=false;
        retry_count_=0;
        waiting_msg_id_=0;
        waiting_type_=MsgType{};

        std::cout << "[TDMA LINK] ACK received for msg_id=" << msg_id << "\n";
    }
}

void TDMALink::on_tdma_sync_received(){
    auto rx_time=std::chrono::steady_clock::now();

    tdma_.sync_rx(rx_time);

    auto logger=rclcpp::get_logger("acoustic_modem_driver");
    RCLCPP_INFO(logger, "TDMA SYNC received, local TDMA armed");
}

void TDMALink::send_tdma_sync(){
    auto tx_time = std::chrono::steady_clock::now();

    driver_.send_two_bytes(driver_.make_tdma_sync());
    std::this_thread::sleep_for(std::chrono::milliseconds(1700));

    tdma_.sync_tx(tx_time);

    auto logger=rclcpp::get_logger("acoustic_modem_driver");
    RCLCPP_INFO(logger, "TDMA SYNC sent");
}

void TDMALink::send_pending_ack(){
    // PRECONDITION: tx_mtx_ must already be locked
    PendingAck ack=pending_acks_.front();
    pending_acks_.pop();

    driver_.send_two_bytes(driver_.make_ack(ack.type,ack.msg_id));
    std::this_thread::sleep_for(std::chrono::milliseconds(1600));
    auto logger = rclcpp::get_logger("acoustic_modem_driver");
    RCLCPP_INFO(logger, "SENT ACK");
}

void TDMALink::resend_last_message(){
    driver_.send_message(waiting_type_,waiting_msg_id_,last_sent_.payload.data());
    std::cout<<"resend message"<<"\n";
    last_tx_time_=std::chrono::steady_clock::now();
    retry_count_++;
}

void TDMALink::send_new_message(const LinkTxMessage& msg){
    uint16_t id= driver_.reserve_msg_id();
    driver_.send_message(msg.type,id,msg.payload.data());

    waiting_ack_=true;
    waiting_msg_id_=id;
    waiting_type_=msg.type;
    last_sent_=msg;
    last_tx_time_=std::chrono::steady_clock::now();
    retry_count_=0;
}

void TDMALink::tx_worker(){
    while(running_){
        auto now=std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex>lock(tx_mtx_);
            if(tdma_.tx_allowed(now)){
                /**
                 * in this case we can transmit more than one ack for slot, 
                 * we can change it by putting a flag saying we already sent one, 
                 * to be checked with more testing in real time, now it works
                 */
                if(!pending_acks_.empty()){
                    send_pending_ack();
                }
                if(current_persistent_cmd_.has_value()){
                    if(now-last_persistent_tx_>=std::chrono::milliseconds(2200)){
                        driver_.send_two_bytes(driver_.make_persistent_cmd(*current_persistent_cmd_));
                        last_persistent_tx_=now;
                    }
                }else{
                    if(waiting_ack_){
                        if(now-last_tx_time_>=ack_timeout_){
                            if(retry_count_<max_retries_){
                                resend_last_message();
                            }else{
                                waiting_ack_=false;
                                retry_count_=0;
                            }
                        }
                    }
                    if(!waiting_ack_ && !tx_queue.empty()){
                        auto logger = rclcpp::get_logger("acoustic_modem_driver");
                        LinkTxMessage msg=tx_queue.front();
                        for (size_t i = 0; i < msg.payload.size(); ++i) {
                            RCLCPP_INFO(logger, "  -> float[%zu] = %f", i, msg.payload[i]);
                        }
                        tx_queue.pop();
                        send_new_message(msg);
                    }
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}