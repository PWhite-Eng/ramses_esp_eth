// Original files from evofw3: frame.h, frame.c, uart.h, uart.c, message.h, message.c

#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include "CC1101_ESP32.hpp"
#include "trace.h"

// Constants from message.h
#define _MSG_ERR_LIST \
  _MSG_ERR( MSG_SIG_ERR,      "Bad Signature" ) \
  _MSG_ERR( MSG_SYNC_ERR,     "Lost Sync" ) \
  _MSG_ERR( MSG_CLSN_ERR,     "Collision" ) \
  _MSG_ERR( MSG_MANC_ERR,     "Invalid Manchester Code" ) \
  _MSG_ERR( MSG_OVERRUN_ERR,  "Too long" ) \
  _MSG_ERR( MSG_CSUM_ERR,     "Checksum error" ) \
  _MSG_ERR( MSG_TRUNC_ERR,    "Truncated" ) \
  _MSG_ERR( MSG_WARNING,      "Warning" ) \
  _MSG_ERR( MSG_SUSPECT_WARN, "Suspect payload" ) \
  _MSG_ERR( MSG_BAD_TX,       "Bad TX message" )

#define _MSG_ERR(_e,_t) _e,
enum msg_err_code { MSG_OK=0, _MSG_ERR_LIST MSG_ERR_MAX };
#undef _MSG_ERR

// Constants from frame.h
#define FRM_START     0xF0
#define FRM_LOST_SYNC 0xF1
#define FRM_END       0xFF

// Constants from message.c
#define MAX_RAW 162
#define MAX_PAYLOAD 64

// Aliases from original C files
#define DEBUG_ISR(_v)      DEBUG1(_v)
#define DEBUG_EDGE(_v)     DEBUG2(_v)
#define DEBUG_FRAME(_v)    DEBUG3(_v)
#define DEBUG_MSG(_v)      DEBUG4(_v)

// Message structure definition
struct message {
  uint8_t state;
  uint8_t count;

  uint8_t fields;
  uint8_t rxFields;
  uint8_t error;

  uint8_t addr[3][3];
  uint8_t param[2];

  uint8_t opcode[2];
  uint8_t len;

  uint8_t csum;
  uint8_t rssi;

  uint8_t nPayload;
  uint8_t payload[MAX_PAYLOAD];

  uint8_t nBytes;
  uint8_t raw[MAX_RAW];
};

// Main protocol class
class EvofwProtocol {
public:
    /**
     * @brief Constructor for the evofw protocol stack.
     * @param cc1101 Reference to your initialized CC1101_ESP32 object.
     * @param uart   Reference to the HardwareSerial peripheral to use for RX (e.g., Serial1).
     * @param gdo0   GPIO pin number connected to CC1101 GDO0 (for TX FIFO interrupt).
     * @param gdo2   GPIO pin number connected to CC1101 GDO2 (for RX data output).
     */
    EvofwProtocol(CC1101_ESP32 &cc1101, HardwareSerial &uart, int8_t gdo0, int8_t gdo2);

    /**
     * @brief Initializes the protocol stack and radio mode.
     * @param radio_baudrate The baud rate for the async serial (e.g., 38400).
     */
    void begin(uint32_t radio_baudrate = 38400);

    /**
     * @brief Main loop function. Must be called regularly to process RX/TX state.
     */
    void loop(void);

    // --- Public API (from message.h) ---

    /**
     * @brief Frees a message structure and returns it to the pool.
     */
    void msg_free(struct message **msg);

    /**
     * @brief Allocates a new, clean message structure from the pool.
     * @return Pointer to a message structure, or NULL if pool is empty.
     */
    struct message *msg_alloc(void);

    /**
     * @brief Checks if a message has a valid (MSG_OK) error code.
     */
    uint8_t msg_isValid(struct message *msg);

    /**
     * @brief Checks if a message is a transmitted (TX) message.
     */
    uint8_t msg_isTx(struct message *msg);

    /**
     * @brief Gets a completed, received message from the RX queue.
     * @return Pointer to a message, or NULL if queue is empty.
     */
    struct message *msg_rx_get(void);

    /**
     * @brief Formats a message into a human-readable string, field by field.
     * @note Call this repeatedly until it returns 0.
     * @param msg      The message to print.
     * @param msgBuff  A buffer to write the string chunk into.
     * @return The number of bytes written to msgBuff. 0 when printing is complete.
     */
    uint8_t msg_print(struct message *msg, char *msgBuff);

    /**
     * @brief Queues a message for transmission.
     */
    void msg_tx_ready(struct message **msg);

    /**
     * @brief Scans a single byte of a human-readable string to build a TX message.
     * @param msg  The message structure to build.
     * @param byte The input character.
     * @return 1 when a complete message is parsed, 0 otherwise.
     */
    uint8_t msg_scan(struct message *msg, uint8_t byte);
    
    /**
     * @brief Modifies an address field in a message (e.g., for address substitution).
     */
    void msg_change_addr(struct message *msg, uint8_t addr, uint8_t Class, uint32_t Id, uint8_t myClass, uint32_t myId);

    void msg_rx_ready(struct message **msg);

    /**
     * @brief Forcibly resets the radio and protocol state machine to RX mode.
     * This is used by the gateway task to recover from a stuck state.
     */
    void resetToRx();

    /**
     * @brief Sets the protocol trace level.
     * @param level The trace bitmask (TRC_RAW, TRC_ERROR, etc.)
     */
    void setTraceLevel(uint8_t level) { _trace0 = level; }

    /**
     * @brief Gets the current protocol trace level.
     * @return The trace bitmask.
     */
    uint8_t getTraceLevel() const { return _trace0; }

private:
    // --- Pointers to external objects ---
    CC1101_ESP32& _cc1101;
    HardwareSerial& _uart;
    int8_t _gdo0_pin;
    int8_t _gdo2_pin;
    uint32_t _baudrate;

    // --- State (from frame.c) ---
    enum frame_states { FRM_OFF, FRM_IDLE, FRM_RX, FRM_TX };
    struct frame_state {
        uint8_t state;
    } frame;

    // --- RX Frame state (from frame.c) ---
    enum frame_rx_states { FRM_RX_OFF, FRM_RX_IDLE, FRM_RX_SYNCH, FRM_RX_MESSAGE, FRM_RX_DONE, FRM_RX_ABORT };
    struct frame_rx {
        uint8_t  state;
        uint8_t nBytes;
        uint8_t nRaw;
        uint8_t *raw;
        uint32_t syncBuffer;
        uint8_t count;
        uint8_t msgErr;
        uint8_t msgByte;
    } rxFrm;
    uint32_t syncWord;

    // --- TX Frame state (from frame.c) ---
    enum frame_tx_states { FRM_TX_OFF, FRM_TX_READY, FRM_TX_IDLE, FRM_TX_PREFIX, FRM_TX_MESSAGE, FRM_TX_SUFFIX, FRM_TX_DONE };
    struct frame_tx {
        uint8_t state;
        uint8_t nBytes;
        uint8_t nRaw;
        uint8_t *raw;
        uint8_t count;
        uint8_t msgByte;
    } txFrm;

    // --- TX FIFO state (from uart.c) ---
    union shift_register {
        uint16_t reg;
        struct {
            uint8_t bits;
            uint8_t data;
        };
    } tx;
    uint8_t txBits;
    enum tx_fifo_state { TX_FIFO_FILL, TX_FIFO_WAIT } tx_state;
    
    // --- Message Pool (from message.c) ---
    #define N_MSG 4
    #define N_LIST ( N_MSG+1 )
    struct msg_list {
        struct message *msg[N_LIST];
        uint8_t in;
        uint8_t out;
    };
    struct message MSG[N_MSG];
    struct msg_list msg_pool;
    struct msg_list rx_list;
    struct msg_list tx_list;
    struct message *msgRx;
    struct message *TxMsg;

    // --- Private Methods ---
    
    // --- uart.c methods ---
    void uart_rx_enable(void);
    void uart_tx_enable(void);
    void uart_disable(void);
    // TX bit-stuffing logic
    uint8_t swap4(uint8_t in);
    uint8_t swap8(uint8_t in);
    uint8_t tx_data(void);
    inline void insert_p(void)  { tx.data <<= 1 ; tx.data |= 0x01; }
    inline void insert_s(void)  { tx.data <<= 1 ; }
    inline void insert_ps(void) { insert_p(); insert_s(); }
    inline void send(uint8_t n) { tx.reg <<= n ; }
    uint8_t tx_byte(uint8_t byte);
    void tx_flush(void);
    void tx_stop(void);
    void tx_fifo_wait(void);
    uint8_t tx_fifo_send_block(void);
    void tx_prime(void);
    void tx_fifo_fill(void);
    
    // --- frame.c methods ---
    void frame_reset(void);
    uint8_t manchester_decode(uint8_t byte);
    uint8_t manchester_encode(uint8_t value);
    int manchester_code_valid(uint8_t code);
    void frame_rx_reset(void);
    void frame_rx_byte(uint8_t byte); // Called by loop()
    void frame_rx_done(void);
    void frame_tx_reset(void);
    void frame_tx_start(uint8_t *raw, uint8_t nRaw);
    uint8_t frame_tx_byte(uint8_t *byte); // Called by tx_fifo_...
    void frame_tx_done(void);
    void frame_rx_enable(void);
    void frame_tx_enable(void);
    
    // --- message.c methods ---
    void msg_reset(struct message *msg);
    void msg_put(struct msg_list *list, struct message **ppMsg, uint8_t reset);
    struct message *msg_get(struct msg_list *list);
    void msg_create_pool(void);
    struct message *msg_tx_get(void);
    uint8_t get_hdr_flags(uint8_t header);
    uint8_t get_header(uint8_t flags);
    uint8_t msg_checksum(struct message *msg);
    void msg_set_address(uint8_t *addr, uint8_t class_id, uint32_t id);
    void msg_get_address(uint8_t *addr, uint8_t *class_id, uint32_t *id);
    uint8_t msg_print_rssi(char *str, uint8_t rssi, uint8_t valid);
    uint8_t msg_print_type(char *str, uint8_t type);
    uint8_t msg_print_addr(char *str, uint8_t *addr, uint8_t valid);
    uint8_t msg_print_param(char *str, uint8_t param, uint8_t valid);
    uint8_t msg_print_opcode(char *str, uint8_t *opcode, uint8_t valid);
    uint8_t msg_print_len(char *str, uint8_t len, uint8_t valid);
    uint8_t msg_print_payload(char *str, uint8_t payload);
    uint8_t msg_print_error(char *str, uint8_t error);
    uint8_t msg_print_raw(char *str, uint8_t raw, uint8_t i);
    uint8_t msg_print_bytes(char *str, uint8_t raw, uint8_t i);
    uint8_t msg_print_field(struct message *msg, char *buff);
    uint8_t msg_rx_header(struct message *msg, uint8_t byte);
    uint8_t msg_rx_addr(struct message *msg, uint8_t addr, uint8_t byte);
    uint8_t msg_rx_param(struct message *msg, uint8_t param, uint8_t byte);
    uint8_t msg_rx_opcode(struct message *msg, uint8_t byte);
    uint8_t msg_rx_len(struct message *msg, uint8_t byte);
    uint8_t msg_rx_payload(struct message *msg, uint8_t byte);
    uint8_t msg_rx_checksum(struct message *msg, uint8_t byte);
    void msg_rx_process(uint8_t byte);
    void msg_rx_rssi(uint8_t rssi);
    uint8_t *msg_rx_start(void);
    uint8_t msg_rx_byte(uint8_t byte);
    void msg_rx_end(uint8_t nBytes, uint8_t error);
    uint8_t msg_scan_header(struct message *msg, char *str, uint8_t nChar);
    uint8_t msg_scan_addr(struct message *msg, char *str, uint8_t nChar);
    uint8_t msg_scan_param(struct message *msg, char *str, uint8_t nChar);
    uint8_t msg_scan_opcode(struct message *msg, char *str, uint8_t nChar);
    uint8_t msg_scan_len(struct message *msg, char *str, uint8_t nChar);
    uint8_t msg_scan_payload(struct message *msg, char *str, uint8_t nChar);
    uint8_t msg_tx_header(struct message *msg, uint8_t *done);
    uint8_t msg_tx_addr(struct message *msg, uint8_t *done);
    uint8_t msg_tx_param(struct message *msg, uint8_t *done);
    uint8_t msg_tx_opcode(struct message *msg, uint8_t *done);
    uint8_t msg_tx_len(struct message *msg, uint8_t *done);
    uint8_t msg_tx_payload(struct message *msg, uint8_t *done);
    uint8_t msg_tx_checksum(struct message *msg, uint8_t *done);
    uint8_t msg_tx_process(struct message *msg, uint8_t *done);
    void msg_tx_start(struct message **msg);
    uint8_t msg_tx_byte(uint8_t *done);
    void msg_tx_end(uint8_t nBytes);
    void msg_tx_done(void);

    // --- trace.h methods ---
    uint8_t _trace0;
};