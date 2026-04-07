#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "muto_link/export.hpp"
#include "muto_link/protocol.hpp"
#include "muto_link/transport.hpp"

namespace muto_link {

struct MUTO_LINK_API DriverOptions {
    double read_timeout_sec = 0.05;
    int retry_count = 1;
    bool validate_checksum = true;
};

struct MUTO_LINK_API ServoState {
    uint8_t id;
    int16_t angle_deg;
    uint16_t speed;
    bool torque_enabled;
};

/**
 * @brief High-level driver for Muto baseboard servo control.
 * 
 * This class provides a convenient C++ interface for controlling servo motors
 * connected to the Muto baseboard via serial communication protocols. It handles
 * protocol framing, communication, and provides both low-level register access
 * and high-level servo control methods.
 * 
 * The driver uses a pluggable transport layer for communication, allowing it to
 * work with different serial interfaces (USB, UART, etc.).
 * 
 * Key features:
 * - Servo torque control (enable/disable)
 * - Precise servo positioning with speed control
 * - Servo angle reading
 * - Low-level register read/write operations
 * - Automatic resource management with RAII
 * 
 * Example usage:
 * @code
 * auto transport = std::make_unique<UsbSerial>("/dev/ttyUSB0");
 * Driver driver(std::move(transport));
 * 
 * driver.open();
 * driver.torqueOn();
 * driver.servoMove(5, 90, 1000);  // Move servo 5 to 90° at speed 1000
 * auto angle = driver.readServoAngle(5);
 * driver.close();
 * @endcode
 */
class MUTO_LINK_API Driver {
public:
    /**
     * @brief Construct a new Driver object.
     * 
     * @param transport Unique pointer to a Transport implementation that handles
     *                  the low-level communication with the Muto baseboard.
     *                  The driver takes ownership of the transport.
     */
    explicit Driver(std::unique_ptr<Transport> transport);
    explicit Driver(std::unique_ptr<Transport> transport, DriverOptions options);
    
    /**
     * @brief Destroy the Driver object.
     * 
     * Automatically closes the transport connection if still open.
     */
    virtual ~Driver() = default;

    /**
     * @brief Open the transport connection.
     * 
     * Establishes communication with the Muto baseboard. Must be called before
     * any servo control operations.
     * 
     * @throws std::runtime_error if the connection cannot be established.
     */
    void open();
    
    /**
     * @brief Close the transport connection.
     * 
     * Terminates communication with the Muto baseboard. Should be called when
     * finished with servo operations to free resources.
     */
    void close();

    /**
     * @brief Enable servo torque control.
     * 
     * Enables torque on all servos, allowing them to be controlled by commands.
     * This must be called before moving servos. When torque is enabled, servos
     * will actively hold their positions and respond to movement commands.
     * 
     * @throws std::runtime_error if the command fails to send.
     */
    void torqueOn();
    
    /**
     * @brief Disable servo torque control.
     * 
     * Disables torque on all servos, allowing manual positioning but preventing
     * command control. Servos can be moved manually when torque is off, but
     * they will not respond to movement commands until torque is re-enabled.
     * 
     * @throws std::runtime_error if the command fails to send.
     */
    void torqueOff();
    
    /**
     * @brief Move a servo to a specific angle at given speed.
     * 
     * Commands the specified servo to move to the target angle using the
     * specified movement speed. The servo must have torque enabled for
     * this command to take effect.
     * 
     * @param servo_id The ID of the servo to move (1-18).
     * @param angle_deg Target angle in degrees (-90 to +90). Values outside
     *                  this range will be clamped.
     * @param speed Movement speed (0-65535). Higher values mean faster movement.
     *              0 = maximum speed, 65535 = minimum speed.
     * 
     * @throws std::invalid_argument if servo_id is out of valid range.
     * @throws std::runtime_error if the command fails to send.
     */
    void servoMove(uint8_t servo_id, int16_t angle_deg, uint16_t speed);
    
    /**
     * @brief Read the current angle of a servo.
     * 
      * Queries the baseboard for servo positions. The protocol returns one byte
      * per servo (18 bytes total), even when a single servo_id is requested.
      * Use readServoAngleDeg() to get a parsed angle for one servo.
     * 
     * @param servo_id The ID of the servo to read (1-18).
      * @return std::vector<uint8_t> Raw servo-angle bytes (index 0 => servo 1).
     * 
     * @throws std::invalid_argument if servo_id is out of valid range.
     * @throws std::runtime_error if the read operation fails.
     */
    std::vector<uint8_t> readServoAngle(uint8_t servo_id);
     /**
      * @brief Read and parse one servo angle in degrees.
      *
      * Extracts the requested servo byte from the 18-byte servo response and
      * converts it from protocol domain to degrees.
      */
    int16_t readServoAngleDeg(uint8_t servo_id);
    ServoState readServoState(uint8_t servo_id);

    /**
     * @brief Send a low-level write command to the baseboard.
     * 
     * Writes data directly to a register address on the Muto baseboard.
     * This is a low-level operation for advanced users who need direct
     * register access.
     * 
     * @param address Register address to write to (0-255).
     * @param data Data bytes to write to the register.
     * 
     * @throws std::runtime_error if the write operation fails.
     */
    void write(uint8_t address, const std::vector<uint8_t>& data);
    
    /**
     * @brief Send a low-level read command and return response.
     * 
     * Reads data from a register address on the Muto baseboard.
     * This is a low-level operation for advanced users who need direct
     * register access.
     * 
     * @param address Register address to read from (0-255).
     * @param data Command data specifying what to read.
     * @return std::vector<uint8_t> Response data bytes from the baseboard.
     * 
     * @throws std::runtime_error if the read operation fails.
     */
    std::vector<uint8_t> read(uint8_t address, const std::vector<uint8_t>& data);

protected:
    /** @brief Transport layer for communication with the baseboard. */
    std::unique_ptr<Transport> transport_;

    DriverOptions options_;

private:
    /** @brief Register address for torque control commands. */
    static constexpr uint8_t kRegTorque = 0x26;
    
    /** @brief Register address for servo position commands. */
    static constexpr uint8_t kRegServoPosition = 0x40;
    
    /** @brief Register address for torque off command. */
    static constexpr uint8_t kRegTorqueOff = 0x27;

    /** @brief Register address for servo angle read commands. */
    static constexpr uint8_t kRegServoAngleRead = 0x50;
};

} // namespace muto_link
