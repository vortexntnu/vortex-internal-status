#include "tdma_link.hpp"

TDMALink::TDMALink(AcousticModemDriver& driver, TDMAManager& tdma): driver_(driver), tdma_(tdma){}

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

void TDMALink::on_data_received(MsgType type, uint16_t msg_id){
    // we use the mutex to avoid multiple acces to shared variables between rx_thread and worker thread
    std::lock_guard<std::mutex>lock(tx_mtx_);
    pending_ack_=true;
    pending_ack_msg_id_=msg_id;
    pending_ack_type_=type;

    if (msg_id == last_rx_msg_id_ && last_rx_valid_) {
        return; // duplicate, ignore payload
    }
    last_rx_valid_=true;
    last_rx_msg_id_ = msg_id;
}

void TDMALink::on_ack_received(MsgType type, uint16_t msg_id){
    std::lock_guard<std::mutex>lock(tx_mtx_);
    if(waiting_ack_ && waiting_msg_id_==msg_id){
        waiting_ack_=false;
        retry_count_=0;

        // not necessary beacuse if waiting_ack is false these are not used but just to be precise:
        waiting_msg_id_=0;
        waiting_type_=MsgType{};

        //debug
        std::cout << "[TDMA LINK] ACK received for msg_id=" << msg_id << "\n";
    }
}

void TDMALink::send_pending_ack(){
    driver_.send_two_bytes(driver_.make_ack(pending_ack_type_,pending_ack_msg_id_));

     std::cout << "[TDMA LINK] Sent ACK for msg_id=" << pending_ack_msg_id_ << "\n";

     pending_ack_=false;
}

void TDMALink::resend_last_message(){
    driver_.send_message(waiting_type_,waiting_msg_id_,last_sent_.payload.data());
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
                if(pending_ack_){
                    send_pending_ack();
                }
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
                if(!tx_queue.empty()){
                    LinkTxMessage msg=tx_queue.front();
                    tx_queue.pop();
                    send_new_message(msg);
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}