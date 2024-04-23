#ifndef CAN_INTERFACE_H
#define CAN_INTERFACE_H

#include <cstring>
#include <iostream>
#include <linux/can.h>
#include <net/if.h>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <linux/can.h>

// Basic file descriptor wrapper. Not really intended for use outside
// SocketCAN.
#include <string>
class fd {
  int id;

public:
  fd() : id(-1) {}
  fd(int n) : id(n) {}
  fd(fd &&other) {
    id = other.id;
    other.id = -1;
  }
  ~fd() {
    if (id != -1)
      close(id);
  }

  void operator=(fd &&other) {
    id = other.id;
    other.id = -1;
  }
  int operator*() { return id; }

  int into_raw() {
    int id_ = id;
    id = -1;
    return id_;
  }
};

// CAN socket.
class SocketCAN {
  fd sock;

public:
  SocketCAN() {}
  SocketCAN(const std::string &interface) {
    sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (*sock == -1)
      throw std::string("Error creating socket");

    // Select CAN interface.
    ifreq req;
    strncpy(req.ifr_ifrn.ifrn_name, interface.data(),
            sizeof(req.ifr_ifrn.ifrn_name));
    if (ioctl(*sock, SIOCGIFINDEX, &req) == -1)
      throw std::string("Unable to select CAN interface");

    // Bind the socket to the network interface.
    sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = req.ifr_ifru.ifru_ivalue;
    if (bind(*sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) == -1)
      throw std::string("Failed to bind socket to network interface");

    std::cout << "Successfully bound socket to interface.\n";
  }

  void transmit(const struct can_frame &cf) {
    std::cout << "Writing CAN frame: "   //
              << "[ " << (int)cf.data[0] //
              << ", " << (int)cf.data[1] //
              << ", " << (int)cf.data[2] //
              << ", " << (int)cf.data[3] //
              << ", " << (int)cf.data[4] //
              << ", " << (int)cf.data[5] //
              << ", " << (int)cf.data[6] //
              << ", " << (int)cf.data[7] //
              << " ]\n";
    if (write(*sock, &cf, sizeof(cf)) != sizeof(cf)) {
      std::ostringstream error_msg;
      error_msg << "Error transmitting CAN frame: " << strerror(errno);
      throw error_msg.str();
    }
  }

  void transmit(int can_id, const uint8_t data[8]) {
    struct can_frame frame;
    frame.can_id = can_id;
    frame.can_dlc = 8 * sizeof(data[0]);
    memcpy(frame.data, data, frame.can_dlc);
    transmit(frame);
  }
};

#endif /* CAN_INTERFACE_H */
