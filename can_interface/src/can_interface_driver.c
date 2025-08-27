#include "can_interface_driver.h"


int canfd_init(int* sock, const char* interface) {
    struct sockaddr_can addr;
    struct ifreq ifr;

    *sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (*sock < 0) {
        perror("Error creating CAN socket");
        return -1;
    }

    memset(&ifr, 0,sizeof(ifr));
    memset(&addr, 0,sizeof(addr));

    strncpy(ifr.ifr_name, interface, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(*sock, SIOCGIFINDEX, &ifr) < 0) {
        perror("Error getting CAN interface index");
        close(*sock);
        return -1;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(*sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("Error binding CAN socket");
        close(*sock);
        return -1;
    }

    int enable_fd = 1;
    if (setsockopt(*sock, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd, sizeof(enable_fd)) < 0) {
        perror("setsockopt(CAN_RAW_FD_FRAMES)");
    }

    return 0;
}

void set_can_filter(int sock, uint16_t start_id, uint16_t id_mask) {
    struct can_filter filter[1];
    filter[0].can_id = start_id;
    filter[0].can_mask = id_mask;

    if (setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) <
        0) {
        perror("Error setting CAN filter");
    }
}

int canfd_send(int sock, const struct canfd_frame* frame) {
    if (write(sock, frame, sizeof(*frame)) != sizeof(*frame)) {
        perror("Error sending CAN FD message");
        return -1;
    }

    return 0;
}

int canfd_recieve(int sock, struct canfd_frame* msg, int timeout_ms) {
    struct timeval timeout;
    fd_set read_fds;

    FD_ZERO(&read_fds);
    FD_SET(sock, &read_fds);

    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = select(sock + 1, &read_fds, NULL, NULL, &timeout);
    if (ret < 0) {
        perror("Error in select()");
        return -1;
    } else if (ret == 0) {
        return -1;  // Timeout
    }

    if (read(sock, msg, sizeof(*msg)) != sizeof(*msg)) {
        perror("Error receiving CAN FD message");
        return -1;
    }

    return 0;
}

void canfd_close(int* sock) {
    if (*sock >= 0) {
        close(*sock);
        *sock = -1;
    }
}
