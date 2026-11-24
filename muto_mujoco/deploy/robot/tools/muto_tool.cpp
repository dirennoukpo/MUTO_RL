// Measurement tool for the MUTO RS (talks to the board directly; the ROS driver must be stopped).
//   battery                      battery voltage (register 0x01, Yahboom: byte/100*8.4 V)
//   timing                       time per read: 0x50 (all 18 servos) and 0x60
//   readall                      all 18 servo angles [servo degrees]
//   raw60 ID                     raw reply of register 0x60 (found: same 7 bytes for any id, not a servo read)
//   step ID FROM TO [SECONDS]    torque on ONE servo, hold FROM, jump to TO, log its angle as fast as possible,
//                                go back to FROM, torque off. Angles in servo degrees. Prints CSV t_ms,angle.
// Uses Diren's muto_link driver (muto_bringup). Compile inside the robot container:
//   g++ -O2 -std=c++17 /tools/muto_tool.cpp -I/muto_ws/src/muto_bringup/include -L/muto_ws/install/muto_bringup/lib \
//       -lmuto_link -Wl,-rpath,/muto_ws/install/muto_bringup/lib -o /tools/muto_tool
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include "muto_link/driver.hpp"
#include "muto_link/transport.hpp"
using namespace muto_link;
using clk = std::chrono::steady_clock;
static double ms(clk::time_point a, clk::time_point b) { return std::chrono::duration<double, std::milli>(b - a).count(); }
static double byte_to_deg(uint8_t b) { int p = b > 127 ? int(b) - 256 : int(b); return p < 0 ? p / 128.0 * 90.0 : p / 127.0 * 90.0; }

int main(int argc, char** argv) {
  if (argc < 2) { std::fprintf(stderr, "usage: muto_tool battery|timing|readall|raw60|step ...\n"); return 2; }
  std::string cmd = argv[1];
  DriverOptions opt; opt.read_timeout_sec = 0.1; opt.retry_count = 2;
  Driver d(std::make_unique<UsbSerial>("/dev/ttyUSB0", 115200, 0.1), opt);
  d.open();
  if (cmd == "battery") {
    auto r = d.read(0x01, {1});
    if (r.empty()) { std::printf("no data\n"); return 1; }
    std::printf("battery raw byte %u -> %.2f V\n", r[0], r[0] / 100.0 * 8.4);
  } else if (cmd == "timing") {
    for (uint8_t reg : {uint8_t(0x50), uint8_t(0x60)}) {
      int ok = 0, n = 30; auto t0 = clk::now(); size_t len = 0;
      for (int i = 0; i < n; ++i) { try { auto r = d.read(reg, {1}); len = r.size(); ++ok; } catch (const std::exception& e) {} }
      std::printf("register 0x%02X: %d/%d replies, %zu data bytes, %.1f ms per read\n", reg, ok, n, len, ms(t0, clk::now()) / n);
    }
  } else if (cmd == "raw60" && argc >= 3) {
    int id = std::atoi(argv[2]);
    auto r = d.read(0x60, {uint8_t(id)});
    std::printf("0x60 id %d ->", id); for (auto b : r) std::printf(" %u", b); std::printf("\n");
    auto a = d.read(0x50, {1});
    std::printf("0x50 angle of id %d = %+.1f deg (byte %u)\n", id, byte_to_deg(a[id - 1]), a[id - 1]);
  } else if (cmd == "readall") {
    auto r = d.read(0x50, {1});
    for (size_t i = 0; i < r.size() && i < 18; ++i) std::printf("id %2zu: %+6.1f\n", i + 1, byte_to_deg(r[i]));
  } else if (cmd == "step" && argc >= 5) {
    int id = std::atoi(argv[2]), from = std::atoi(argv[3]), to = std::atoi(argv[4]);
    double secs = argc > 5 ? std::atof(argv[5]) : 1.0;
    d.torqueOnServo(id);
    d.servoMove(id, from, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    std::printf("t_ms,angle_deg\n");
    auto t0 = clk::now();
    d.servoMove(id, to, 0);
    while (ms(t0, clk::now()) < secs * 1000) {
      try { auto r = d.read(0x50, {uint8_t(id)}); if (r.size() >= size_t(id)) std::printf("%.1f,%.1f\n", ms(t0, clk::now()), byte_to_deg(r[id - 1])); }
      catch (const std::exception& e) {}
    }
    d.servoMove(id, from, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    d.torqueOffServo(id);
  } else { std::fprintf(stderr, "bad command\n"); return 2; }
  d.close();
  return 0;
}
