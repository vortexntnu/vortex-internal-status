#include "am_driver.hpp"

/**
 * Idea: use hybrid TDMA, to try to avoid collisions
 * 
 */

enum class TDMAState{
    MY_TURN;
    WAIT_TURN;
    LISTEN;
}


class TDMAManager{
    public:
    TDMAManager(double max_slot_time, double guard_time, double silence_timeout, bool is_master);

    // modem is allowed to send
    bool can_send();

    //change the offset
    void offset();

    // change who is allowed to send
    void update();

    private:
    double max_slot_time_;
    double guard_time_;
    double silence_timeout_;
    bool is_master_;
    TDMAState state;
    

}