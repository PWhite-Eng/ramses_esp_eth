// Original files from evofw3: cmd.h, cmd.c, cc1101_tune.h, cc1101_tune.c, device.h, device.c, tty.h, version.h
#pragma once

#include <Arduino.h>
#include "CC1101_ESP32.hpp"
#include "EvofwProtocol.hpp"

// From tty.h, for command buffer size
#define TXBUF 32
#define CMD_CHAR '!'

class EvofwHandler {
public:
    /**
     * @brief Constructor for the evofw Application Handler.
     * @param protocol Reference to the EvofwProtocol library.
     * @param cc1101   Reference to the CC1101_ESP32 library.
     * @param host_serial Reference to the Serial port for host commands (e.g., Serial).
     */
    EvofwHandler(EvofwProtocol &protocol, CC1101_ESP32 &cc1101);

    /**
     * @brief Initializes the device ID.
     * @param device_class The class ID for this device.
     */
    void begin(uint8_t device_class = 1); // Default class 1

    /**
     * @brief Main loop function. Call this repeatedly.
     * It runs the tuning state machine.
     * @return 1 if the tuner generated a command, 0 otherwise.
     */
    uint8_t loop(char *cmd_str_buffer, uint8_t buffer_len);

    /**
     * @brief Get the device ID.
     */
    void device_get_id(uint8_t *class_id, uint32_t *id);

    /**
     * @brief Process a single character from the host serial port.
     * @param byte The byte received.
     * @param buffer Pointer to be filled with response buffer.
     * @param n Pointer to be filled with response length.
     * @return 1 if a command is in progress, 0 otherwise.
     */
    uint8_t process_cmd_char(uint8_t byte, char **buffer, uint8_t *n, char* cmdStrBuf);

    /**
     * @brief Process a full command string.
     * @param str The command string (e.g., "V\r").
     * @param buffer Pointer to be filled with response buffer.
     * @param n Pointer to be filled with response length.
     * @return 1 if the command was valid, 0 otherwise.
     */
    uint8_t process_cmd_str(const char *str, char **buffer, uint8_t *n);

    /**
     * @brief Check if tuning is enabled.
     */
    uint8_t cc_tuneEnabled(void);

    /**
     * @brief Run one cycle of the tuning logic.
     * @param msg A received message (or NULL).
     * @param cmdBuff Buffer to write a tuning command into.
     * @return Length of the command written to cmdBuff.
     */
    uint8_t cc_tune_work(struct message *msg, char *cmdBuff);

private:
    // --- External Object References ---
    EvofwProtocol& _protocol;
    CC1101_ESP32& _cc1101;

    // --- device.c state ---
    uint8_t  _devClass;
    uint32_t _devId;

    // --- cmd.c state ---
    struct cmd_state {
        char buffer[TXBUF];
        uint8_t n;
        uint8_t inCmd;
    } _command;

    // --- cc1101_tune.c state ---
    enum cc_tune_state { TUNE_IDLE, TUNE_BEGIN, TUNE_WAIT, TUNE_CHOP, TUNE_ABORT, TUNE_STOP } _tuneState;
    uint8_t _tuneEnabled;
    uint32_t _f0; // Start frequency
    int16_t _tune_x, _tune_y;
    int16_t _tune_step;
    int16_t _tune_low, _tune_high;
    uint32_t _f_current;
    uint32_t _lastF;
    unsigned long _lastValidTuneTime;

    // --- Private Methods ---

    // --- device.c methods ---
    void device_init(uint8_t class_id);

    // --- cc1101_tune.c methods ---
    void cc_tune_enable(uint8_t enable);
    uint32_t cc_tune(uint8_t validMsg, uint8_t timeout);
    
    // --- cmd.c methods ---
    void reset_command(void);
    uint8_t get_hex(uint8_t len, char *param);
    uint8_t cmd_trace(struct cmd_state *cmd);
    uint8_t cmd_version(struct cmd_state *cmd);
    uint8_t cmd_id(struct cmd_state *cmd);
    uint8_t cmd_boot(struct cmd_state *cmd);
    uint8_t cmd_cc1101(struct cmd_state *cmd);
    uint8_t cmd_cc_tune(struct cmd_state *cmd);
    uint8_t cmd_eeprom(struct cmd_state *cmd);
    uint8_t check_command(struct cmd_state *cmd);
};