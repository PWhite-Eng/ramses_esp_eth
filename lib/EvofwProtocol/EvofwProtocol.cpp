#include "EvofwProtocol.hpp"
#include <driver/gpio.h>
#include "debug.h"
#include "log_config.h"

static constexpr unsigned long TX_FIFO_EMPTY_TIMEOUT_MS = 50;  // 50 ms timeout for TX FIFO empty wait

// Original AVR PROGMEM data, now using Arduino-compatible PROGMEM
static const uint8_t man_encode[16] PROGMEM = {
  0xAA, 0xA9, 0xA6, 0xA5,  0x9A, 0x99, 0x96, 0x95,
  0x6A, 0x69, 0x66, 0x65,  0x5A, 0x59, 0x56, 0x55
};
#define MAN_ENCODE(_i) pgm_read_byte( man_encode+(_i) )

static const uint8_t man_decode[16] PROGMEM = {
  0xF, 0xF, 0xF, 0xF, 0xF, 0x3, 0x2, 0xF,
  0xF, 0x1, 0x0, 0xF, 0xF, 0xF, 0xF, 0xF
};
#define MAN_DECODE(_i) pgm_read_byte( man_decode+(_i) )

static char const * const MsgType[4] PROGMEM = { "RQ", "I","W","RP" };

static uint8_t evo_hdr[] = { 0x33, 0x55, 0x53 };
static uint8_t evo_tlr[] = { 0x35 };

static uint8_t tx_prefix[] = {
  0x55, 0x55, 0x55, 0x55, 0x55,   // Pre-amble
  0xFF, 0x00,                     // Sync Word
  0x33, 0x55, 0x53                // Header
};

static uint8_t tx_suffix[] = {
  0x35,                           // Trailer
  0x55,                           // Training
};

static const uint8_t address_flags[4] = {
  0x10 + 0x20 + 0x40,  // F_ADDR0 + F_ADDR1 + F_ADDR2
  0x40,                // F_ADDR2
  0x10 + 0x40,         // F_ADDR0 + F_ADDR2
  0x10 + 0x20          // F_ADDR0 + F_ADDR1
};


// --- Constructor ---
EvofwProtocol::EvofwProtocol(CC1101_ESP32 &cc1101, HardwareSerial &uart, int8_t gdo0, int8_t gdo2)
    : _cc1101(cc1101), _uart(uart), _gdo0_pin(gdo0), _gdo2_pin(gdo2) {
    
    _trace0 = 0; // Default trace to off
    msgRx = nullptr;
    TxMsg = nullptr;
}

// --- Public Methods ---

void EvofwProtocol::begin(uint32_t radio_baudrate) {
    _baudrate = radio_baudrate;

    // Calculate syncWord (from frame_init)
    syncWord = 0;
    for(uint8_t i=0 ; i<sizeof(evo_hdr) ; i++ )
        syncWord = ( syncWord<<8 ) | evo_hdr[i];

    frame_reset();
    
    // Init message pools (from msg_init)
    msg_create_pool();

    // Set GDO pins
    pinMode(_gdo0_pin, INPUT); // GDO0 is TX FIFO interrupt
    // GDO2 is RX data, will be set as RX pin by _uart.begin()
    
    frame.state = FRM_IDLE;
}

void EvofwProtocol::loop(void) {
    // Replaces uart_work()
    if (_uart.available()) {
        frame_rx_byte(_uart.read());
    }

    // Replaces frame_work()
    switch( frame.state ) {
    case FRM_IDLE:
        if( rxFrm.state==FRM_RX_OFF ) {
            frame_rx_enable();
        }
        break;

    case FRM_RX:
        if( rxFrm.state>=FRM_RX_DONE ) {
            frame_rx_done();
        }
        if( rxFrm.state<FRM_RX_MESSAGE ) {
            if( txFrm.state==FRM_TX_READY ) {
                frame_tx_enable();
            } else if( rxFrm.state==FRM_RX_OFF ) {
                frame_rx_enable();
            }
        }
        break;

    case FRM_TX:
        if( txFrm.state>=FRM_TX_DONE ) {
            frame_tx_done();
            frame_rx_enable();
        }
        break;
    }

    if( !TxMsg ) {
        struct message *tx1 = msg_tx_get();
        if( tx1 )
            msg_tx_start( &tx1 );
    }
}

// --- uart.c TX implementation ---
// (RX is handled by HardwareSerial)

void EvofwProtocol::uart_rx_enable(void) {
    // Use the GDO2 pin as the RX pin for the specified UART
    _uart.begin(_baudrate, SERIAL_8N1, _gdo2_pin, -1);
}

void EvofwProtocol::uart_tx_enable(void) {
 
    tx_prime();
    tx_state = TX_FIFO_FILL;
}

void EvofwProtocol::uart_disable(void) {
    tx_stop();      // Detach GDO0 interrupt
    _uart.end();    // Stop hardware UART
}

uint8_t EvofwProtocol::swap4(uint8_t in) {
    static uint8_t out[16] = {
        0x0, 0x8, 0x4, 0xC, 0x2, 0xA, 0x6, 0xE,
        0x1, 0x9, 0x5, 0xD, 0x3, 0xB, 0x7, 0xF
    };
    return out[in & 0xF];
}

uint8_t EvofwProtocol::swap8(uint8_t in) {
    uint8_t out;
    out  = swap4(in) << 4;
    out |= swap4(in >> 4);
    return out;
}

uint8_t EvofwProtocol::tx_data(void) {
    return _cc1101.writeFifo(tx.data);
}

uint8_t EvofwProtocol::tx_byte(uint8_t byte) {
    uint8_t space = 15;
    tx.bits = swap8(byte);

    switch (txBits) {
    case 0: insert_ps(); send(6); space = tx_data(); send(2); txBits=2; break;
    case 2: insert_ps(); send(4); space = tx_data(); send(4); txBits=4; break;
    case 4: insert_ps(); send(2); space = tx_data(); send(6); // Fall through
    case 6: insert_ps();          space = tx_data();          txBits=8; break;
    case 8:              send(8); space = tx_data();          txBits=0; break;
    }
    return space;
}

void EvofwProtocol::tx_flush(void) {
    if (txBits) {
        send(8 - txBits);
        tx_data();
    }
    tx.data = 0xFF; // Leave in SPACE
    tx_data();
}

void EvofwProtocol::tx_stop(void) {
    
}

void EvofwProtocol::tx_fifo_wait(void) {
    uint8_t data;
    frame_tx_byte(&data); // This is just to satisfy original logic
}

uint8_t EvofwProtocol::tx_fifo_send_block(void) {
    uint8_t done;
    uint8_t count;
    uint8_t block = 4;
    do {
        uint8_t data;
        done = frame_tx_byte(&data);
        count = tx_byte(data);
        block--;
    } while (block && !done && count > 4);
    return done;
}

void EvofwProtocol::tx_prime(void) {
    _cc1101.writeFifo(0x00);
    _cc1101.writeFifo(0xFF); // BREAK
    _cc1101.writeFifo(0x00);
    _cc1101.writeFifo(0x00);

    txBits = 0;
    // We can't check GDO pin state directly in the same way,
    // just prime with a few blocks.
    tx_fifo_send_block();
    tx_fifo_send_block();
}

void EvofwProtocol::tx_fifo_fill(void) {
    uint8_t done = tx_fifo_send_block();
    if (done) {
        tx_flush();
        _cc1101.fifoEnd(); // Tell CC1101 to monitor for TX FIFO empty
        tx_state = TX_FIFO_WAIT;
    }
}

// --- frame.c implementation ---

void EvofwProtocol::frame_reset(void) {
    memset(&frame, 0, sizeof(frame));
    frame_rx_reset();
    frame_tx_reset();
}

int EvofwProtocol::manchester_code_valid(uint8_t code) {
    return (MAN_DECODE((code >> 4) & 0xF) != 0xF) && (MAN_DECODE((code) & 0xF) != 0xF);
}

uint8_t EvofwProtocol::manchester_decode(uint8_t byte) {
    uint8_t decoded;
    decoded  = MAN_DECODE((byte) & 0xF);
    decoded |= MAN_DECODE((byte >> 4) & 0xF) << 2;
    return decoded;
}

uint8_t EvofwProtocol::manchester_encode(uint8_t value) {
    return MAN_ENCODE(value & 0xF);
}

void EvofwProtocol::frame_rx_reset(void) {
    memset(&rxFrm, 0, sizeof(rxFrm));
}

void EvofwProtocol::frame_rx_byte(uint8_t byte) {
    switch (rxFrm.state) {
    case FRM_RX_IDLE:
        rxFrm.syncBuffer = byte;
        if (byte == evo_hdr[0])
            rxFrm.state = FRM_RX_SYNCH;
        break;

    case FRM_RX_SYNCH:
        rxFrm.syncBuffer <<= 8;
        if ((byte == 0x00) || (byte == 0xFF) || (rxFrm.syncBuffer & 0xFF000000)) {
            rxFrm.state = FRM_RX_IDLE;
            break;
        }

        rxFrm.syncBuffer |= byte;
        if (rxFrm.syncBuffer == syncWord) {
            rxFrm.raw = msg_rx_start();
            if (rxFrm.raw) {
                rxFrm.nRaw = rxFrm.raw[0];
                rxFrm.state = FRM_RX_MESSAGE;
                DEBUG_FRAME(1);
            }
        }
        break;

    case FRM_RX_MESSAGE:
        if (byte == 0x00) {
            rxFrm.state = FRM_RX_ABORT;
            rxFrm.msgErr = MSG_CLSN_ERR;
        } else if (byte == FRM_LOST_SYNC) {
            rxFrm.state = FRM_RX_ABORT;
            rxFrm.msgErr = MSG_SYNC_ERR;
        } else if (byte == evo_tlr[0]) {
            rxFrm.state = FRM_RX_DONE;
        } else {
            rxFrm.raw[rxFrm.nBytes++] = byte;

            if (!manchester_code_valid(byte)) {
                rxFrm.state = FRM_RX_ABORT;
                rxFrm.msgErr = MSG_MANC_ERR;
            } else {
                rxFrm.msgByte <<= 4;
                rxFrm.msgByte |= manchester_decode(byte);
                rxFrm.count = 1 - rxFrm.count;

                if (!rxFrm.count) {
                    rxFrm.msgErr = msg_rx_byte(rxFrm.msgByte);
                    if (rxFrm.msgErr != MSG_OK)
                        rxFrm.state = FRM_RX_ABORT;
                }
            }
        }
        break;
    }

    if (rxFrm.state > FRM_RX_SYNCH && rxFrm.state < FRM_RX_DONE) {
        if (rxFrm.nBytes >= rxFrm.nRaw) {
            rxFrm.state = FRM_RX_ABORT;
            rxFrm.msgErr = MSG_OVERRUN_ERR;
        }
    }

    if (rxFrm.state >= FRM_RX_DONE) {
        DEBUG_FRAME(0);
    }
}

void EvofwProtocol::frame_rx_done(void) {
    DEBUG_FRAME(1);
    uint8_t nBytes = rxFrm.nBytes;
    uint8_t msgErr = rxFrm.msgErr;
    uint8_t rssi;

    frame_rx_reset();

    rssi = _cc1101.readRSSI();
    msg_rx_rssi(rssi);
    msg_rx_end(nBytes, msgErr);

    DEBUG_FRAME(0);
}

void EvofwProtocol::frame_tx_reset(void) {
    memset(&txFrm, 0, sizeof(txFrm));
}

void EvofwProtocol::frame_tx_start(uint8_t *raw, uint8_t nRaw) {
    uint8_t i, done, byte;
    for (i = 0; i < nRaw + 1; i += 2) {
        byte = msg_tx_byte(&done);
        if (done) break;
        raw[i] = manchester_encode(byte >> 4);
        raw[i + 1] = manchester_encode(byte);
    }
    txFrm.nBytes = i;
    txFrm.raw = raw;
    txFrm.nRaw = nRaw;
    txFrm.state = FRM_TX_READY;
}

uint8_t EvofwProtocol::frame_tx_byte(uint8_t *byte) {
    uint8_t done = 0;
    switch (txFrm.state) {
    case FRM_TX_IDLE:
        txFrm.state = FRM_TX_PREFIX;
    case FRM_TX_PREFIX:
        if (txFrm.count < sizeof(tx_prefix)) {
            (*byte) = tx_prefix[txFrm.count++];
            break;
        }
        txFrm.count = 0;
        txFrm.state = FRM_TX_MESSAGE;
    case FRM_TX_MESSAGE:
        if (txFrm.count < txFrm.nBytes) {
            (*byte) = txFrm.raw[txFrm.count++];
            break;
        }
        msg_tx_end(txFrm.nBytes);
        txFrm.count = 0;
        txFrm.state = FRM_TX_SUFFIX;
    case FRM_TX_SUFFIX:
        if (txFrm.count < sizeof(tx_suffix)) {
            (*byte) = tx_suffix[txFrm.count++];
            if (txFrm.count == sizeof(tx_suffix))
                done = 1;
            break;
        }
        txFrm.count = 0;
        txFrm.state = FRM_TX_DONE;
    case FRM_TX_DONE:
        done = 1;
        break;
    }
    return done;
}

void EvofwProtocol::frame_tx_done(void) {
    msg_tx_done();
    frame_tx_reset();
}

void EvofwProtocol::frame_rx_enable(void) {
    uart_disable();
    _cc1101.enterRxMode();
    frame.state = FRM_RX;
    rxFrm.state = FRM_RX_IDLE;
    uart_rx_enable();
}

void EvofwProtocol::frame_tx_enable(void) {
    uart_disable();
    _cc1101.enterTxMode();
    frame.state = FRM_TX;
    txFrm.state = FRM_TX_IDLE;
    
    // 1. Prime the FIFO
    tx_prime();
    tx_state = TX_FIFO_FILL;

    // 2. Run the fill loop until the packet is fully loaded
    while (tx_state == TX_FIFO_FILL) {
        tx_fifo_fill();
        // We can add a tiny delay to yield to the OS if needed,
        // but for speed, we'll try without it first.
        // vTaskDelay(1 / portTICK_PERIOD_MS); 
    }

    // 3. Packet is now flushed, wait for it to finish sending
    // (GDO0 goes HIGH when TX FIFO is empty)
    unsigned long tx_start = millis();
    while (digitalRead(_gdo0_pin) == LOW) {
        if (millis() - tx_start > TX_FIFO_EMPTY_TIMEOUT_MS) { // 50ms timeout
            // This should not happen, but it's a safe fallback.
            ESP_LOGW(TAG_EVOFW, "TX timeout waiting for GDO0 HIGH");
            break; 
        }
    }
    
    // 4. Finalize
    tx_fifo_wait(); // Call this once to finalize
    txFrm.state = FRM_TX_DONE; // Manually advance state
}

// --- message.c implementation ---

void EvofwProtocol::msg_reset(struct message *msg) {
    if (msg != NULL) {
        memset(msg, 0, sizeof(*msg));
    }
}

void EvofwProtocol::msg_put(struct msg_list *list, struct message **ppMsg, uint8_t reset) {
    if (ppMsg != NULL) {
        struct message *pMsg = (*ppMsg);
        if (pMsg != NULL) {
            if (reset)
                msg_reset(pMsg);
            list->msg[list->in] = pMsg;
            list->in = (list->in + 1) % N_LIST;
            (*ppMsg) = NULL;
        }
    }
}

struct message *EvofwProtocol::msg_get(struct msg_list *list) {
    struct message *msg = list->msg[list->out];
    if (msg != NULL) {
        list->msg[list->out] = NULL;
        list->out = (list->out + 1) % N_LIST;
        msg->state = 0; // S_START
    }
    return msg;
}

void EvofwProtocol::msg_free(struct message **msg) {
    msg_put(&msg_pool, msg, 1);
}

struct message *EvofwProtocol::msg_alloc(void) {
    return msg_get(&msg_pool);
}

void EvofwProtocol::msg_create_pool(void) {
    memset(&msg_pool, 0, sizeof(msg_pool));
    memset(&rx_list, 0, sizeof(rx_list));
    memset(&tx_list, 0, sizeof(tx_list));
    for (uint8_t i = 0; i < N_MSG; i++) {
        struct message *msg = &MSG[i];
        msg_free(&msg);
    }
}

void EvofwProtocol::msg_rx_ready(struct message **msg) {
    msg_put(&rx_list, msg, 0);
}

struct message *EvofwProtocol::msg_rx_get(void) {
    return msg_get(&rx_list);
}

void EvofwProtocol::msg_tx_ready(struct message **msg) {
    msg_put(&tx_list, msg, 0);
}

struct message *EvofwProtocol::msg_tx_get(void) {
    return msg_get(&tx_list);
}

uint8_t EvofwProtocol::get_hdr_flags(uint8_t header) {
    uint8_t flags;
    flags = (header & 0x30) >> 4; // HDR_T_MASK
    flags |= address_flags[(header & 0x0C) >> 2]; // HDR_A_MASK
    if (header & 0x02) flags |= 0x04; // HDR_PARAM0 -> F_PARAM0
    if (header & 0x01) flags |= 0x08; // HDR_PARAM1 -> F_PARAM1
    return flags;
}

uint8_t EvofwProtocol::get_header(uint8_t flags) {
    uint8_t i;
    uint8_t header = 0xFF;
    uint8_t addresses = flags & (0x10 + 0x20 + 0x40); // F_ADDR0+F_ADDR1+F_ADDR2
    for (i = 0; i < sizeof(address_flags); i++) {
        if (addresses == address_flags[i]) {
            header = i << 2; // HDR_A_SHIFT
            header |= (flags & 0x03) << 4; // F_MASK, HDR_T_SHIFT
            if (flags & 0x04) header |= 0x02; // F_PARAM0 -> HDR_PARAM0
            if (flags & 0x08) header |= 0x01; // F_PARAM1 -> HDR_PARAM1
            break;
        }
    }
    return header;
}

uint8_t EvofwProtocol::msg_checksum(struct message *msg) {
    uint8_t csum;
    uint8_t i, j;
    csum = get_header(msg->fields);
    for (i = 0; i < 3; i++) { for (j = 0; j < 3; j++) { csum += msg->addr[i][j]; } }
    for (i = 0; i < 2; i++) { csum += msg->param[i]; }
    for (i = 0; i < 2; i++) { csum += msg->opcode[i]; }
    csum += msg->len;
    for (i = 0; i < msg->nPayload; i++) { csum += msg->payload[i]; }
    return -csum;
}

void EvofwProtocol::msg_set_address(uint8_t *addr, uint8_t class_id, uint32_t id) {
    addr[0] = ((class_id << 2) & 0xFC) | ((id >> 16) & 0x03);
    addr[1] = ((id >> 8) & 0xFF);
    addr[2] = ((id) & 0xFF);
}

void EvofwProtocol::msg_get_address(uint8_t *addr, uint8_t *class_id, uint32_t *id) {
    uint8_t Class = (addr[0] & 0xFC) >> 2;
    uint32_t Id = (uint32_t)(addr[0] & 0x03) << 16
                | (uint32_t)(addr[1]) << 8
                | (uint32_t)(addr[2]);
    if (class_id) (*class_id) = Class;
    if (id) (*id) = Id;
}

// --- msg_print functions ---
uint8_t EvofwProtocol::msg_print_rssi(char *str, uint8_t rssi, uint8_t valid) {
    return (valid) ? sprintf(str, "%03u ", rssi) : sprintf(str, "--- ");
}

uint8_t EvofwProtocol::msg_print_type(char *str, uint8_t type) {
    char type_str[3];
    strncpy_P(type_str, (PGM_P)pgm_read_ptr(&MsgType[type]), 3);
    return sprintf(str, "%2s ", type_str);
}

uint8_t EvofwProtocol::msg_print_addr(char *str, uint8_t *addr, uint8_t valid) {
    if (valid) {
        uint8_t class_id;
        uint32_t id;
        msg_get_address(addr, &class_id, &id);
        return sprintf(str, "%02hhu:%06lu ", class_id, id);
    } else {
        return sprintf(str, "--:------ ");
    }
}
void EvofwProtocol::msg_change_addr( struct message *msg, uint8_t addr, uint8_t Class,uint32_t Id , uint8_t myClass,uint32_t myId ) {
  if( msg && ( msg->fields & ( 0x10 << addr ) ) ) { // F_ADDR0
    uint8_t *Addr = msg->addr[addr];
	uint8_t class_id;
	uint32_t id;
	msg_get_address( Addr, &class_id, &id );
	if( class_id==Class && id==Id )
	  msg_set_address( Addr, myClass, myId ); 
  }
}
uint8_t EvofwProtocol::msg_print_param(char *str, uint8_t param, uint8_t valid) {
    return (valid) ? sprintf(str, "%03u ", param) : sprintf(str, "--- ");
}
uint8_t EvofwProtocol::msg_print_opcode(char *str, uint8_t *opcode, uint8_t valid) {
    return (valid) ? sprintf(str, "%02X%02X ", opcode[0], opcode[1]) : sprintf(str, "???? ");
}
uint8_t EvofwProtocol::msg_print_len(char *str, uint8_t len, uint8_t valid) {
    return (valid) ? sprintf(str, "%03u ", len) : sprintf(str, "??? ");
}
uint8_t EvofwProtocol::msg_print_payload(char *str, uint8_t payload) {
    return sprintf(str, "%02X", payload);
}
uint8_t EvofwProtocol::msg_print_error(char *str, uint8_t error) {
    // Simplified error printing
    if (error) {
        return sprintf(str, " * ERR:%02X\r\n", error);
    } else {
        return sprintf(str, "\r\n");
    }
}
uint8_t EvofwProtocol::msg_print_raw(char *str, uint8_t raw, uint8_t i) {
    return (i) ? sprintf(str, "%02X.", raw) : sprintf(str, "# %02X.", raw);
}
uint8_t EvofwProtocol::msg_print_bytes(char *str, uint8_t raw, uint8_t i) {
    return (i) ? sprintf(str, "%c", raw) : sprintf(str, "# %c", raw);
}

uint8_t EvofwProtocol::msg_print_field(struct message *msg, char *buff) {
    uint8_t nBytes = 0;
    enum { S_START=0, S_HEADER, S_ADDR0, S_ADDR1, S_ADDR2, S_PARAM0, S_PARAM1,
           S_OPCODE, S_LEN, S_PAYLOAD, S_CHECKSUM, S_TRAILER, S_COMPLETE, S_ERROR };
    switch (msg->state) {
    case S_START:
        nBytes = msg_print_rssi(buff, msg->rssi, msg->rxFields & 0x80); // F_RSSI
        msg->state = S_HEADER; if (nBytes) break;
    case S_HEADER:
        nBytes = msg_print_type(buff, msg->fields & 0x03); // F_MASK
        msg->state = S_PARAM0; if (nBytes) break;
    case S_PARAM0:
        nBytes = msg_print_param(buff, msg->param[0], msg->rxFields & 0x04); // F_PARAM0
        msg->state = S_ADDR0; if (nBytes) break;
    case S_ADDR0:
        nBytes = msg_print_addr(buff, msg->addr[0], msg->rxFields & 0x10); // F_ADDR0
        msg->state = S_ADDR1; if (nBytes) break;
    case S_ADDR1:
        nBytes = msg_print_addr(buff, msg->addr[1], msg->rxFields & 0x20); // F_ADDR1
        msg->state = S_ADDR2; if (nBytes) break;
    case S_ADDR2:
        nBytes = msg_print_addr(buff, msg->addr[2], msg->rxFields & 0x40); // F_ADDR2
        msg->state = S_OPCODE; if (nBytes) break;
    case S_OPCODE:
        nBytes = msg_print_opcode(buff, msg->opcode, msg->rxFields & 0x01); // F_OPCODE
        msg->state = S_LEN; if (nBytes) break;
    case S_LEN:
        nBytes = msg_print_len(buff, msg->len, msg->rxFields & 0x02); // F_LEN
        msg->state = S_PAYLOAD; if (nBytes) break;
    case S_PAYLOAD:
        if (msg->count < msg->nPayload) {
            nBytes = msg_print_payload(buff, msg->payload[msg->count++]);
            if (nBytes) break;
        }
        msg->count = 0; msg->state = S_ERROR;
    case S_ERROR:
        nBytes = msg_print_error(buff, msg->error);
        msg->state = S_TRAILER; if (nBytes) break;
    case S_TRAILER:
        if (msg->error || (_trace0 & TRC_RAW)) {
            if (msg->count < msg->nBytes) {
                nBytes = (msg->rxFields & 0x80) ? // F_RSSI
                    msg_print_raw(buff, msg->raw[msg->count], msg->count) :
                    msg_print_bytes(buff, msg->raw[msg->count], msg->count);
                msg->count++;
            } else if (msg->nBytes) {
                nBytes = sprintf(buff, "\r\n");
                msg->state = S_COMPLETE;
            }
            if (nBytes) break;
        }
        msg->count = 0; msg->state = S_COMPLETE;
    case S_COMPLETE:
        break;
    }
    return nBytes;
}

uint8_t EvofwProtocol::msg_print(struct message *msg, char *msg_buff) {
    if (msg->state == 0) { // S_START
        DEBUG_MSG(1);
        msg->count = 0;
    }
    uint8_t n = msg_print_field(msg, msg_buff);
    if (msg->state == 12) // S_COMPLETE
        DEBUG_MSG(0);
    return n;
}

// --- msg_rx functions ---
uint8_t EvofwProtocol::msg_rx_header(struct message *msg, uint8_t byte) {
    msg->fields = get_hdr_flags(byte);
    return 2; // S_ADDR0
}
uint8_t EvofwProtocol::msg_rx_addr(struct message *msg, uint8_t addr, uint8_t byte) {
    uint8_t state = 2 + addr; // S_ADDR0
    msg->addr[addr][msg->count++] = byte;
    if (msg->count == 3) {
        msg->count = 0; state++;
        msg->rxFields |= (0x10 << addr); // F_ADDR0
    }
    return state;
}
uint8_t EvofwProtocol::msg_rx_param(struct message *msg, uint8_t param, uint8_t byte) {
    uint8_t state = 5 + param; // S_PARAM0
    msg->param[param] = byte;
    state++;
    msg->rxFields |= (0x04 << param); // F_PARAM0
    return state;
}
uint8_t EvofwProtocol::msg_rx_opcode(struct message *msg, uint8_t byte) {
    uint8_t state = 7; // S_OPCODE
    msg->opcode[msg->count++] = byte;
    if (msg->count == 2) {
        msg->count = 0; state++;
        msg->rxFields |= 0x01; // F_OPCODE
    }
    return state;
}
uint8_t EvofwProtocol::msg_rx_len(struct message *msg, uint8_t byte) {
    msg->len = byte;
    msg->rxFields |= 0x02; // F_LEN
    return 9; // S_PAYLOAD
}
uint8_t EvofwProtocol::msg_rx_payload(struct message *msg, uint8_t byte) {
    if (msg->nPayload < MAX_PAYLOAD) {
        msg->payload[msg->nPayload++] = byte;
    }
    msg->count++;
    if (msg->count == msg->len) {
        msg->count = 0;
        return 10; // S_CHECKSUM
    }
    return 9; // S_PAYLOAD
}
uint8_t EvofwProtocol::msg_rx_checksum(struct message *msg, uint8_t byte) {
    if (msg->csum != 0 && !msg->error)
        msg->error = MSG_CSUM_ERR;
    return 12; // S_COMPLETE
}
void EvofwProtocol::msg_rx_process(uint8_t byte) {
    msgRx->csum += byte;
    enum { S_START=0, S_HEADER, S_ADDR0, S_ADDR1, S_ADDR2, S_PARAM0, S_PARAM1,
           S_OPCODE, S_LEN, S_PAYLOAD, S_CHECKSUM, S_TRAILER, S_COMPLETE, S_ERROR };
    switch (msgRx->state) {
    case S_START:
    case S_HEADER:  msgRx->state = msg_rx_header(msgRx, byte); break;
    case S_ADDR0:   if (msgRx->fields & 0x10) { msgRx->state = msg_rx_addr(msgRx, 0, byte); break; }
    case S_ADDR1:   if (msgRx->fields & 0x20) { msgRx->state = msg_rx_addr(msgRx, 1, byte); break; }
    case S_ADDR2:   if (msgRx->fields & 0x40) { msgRx->state = msg_rx_addr(msgRx, 2, byte); break; }
    case S_PARAM0:  if (msgRx->fields & 0x04) { msgRx->state = msg_rx_param(msgRx, 0, byte); break; }
    case S_PARAM1:  if (msgRx->fields & 0x08) { msgRx->state = msg_rx_param(msgRx, 1, byte); break; }
    case S_OPCODE:  msgRx->state = msg_rx_opcode(msgRx, byte); break;
    case S_LEN:     msgRx->state = msg_rx_len(msgRx, byte); break;
    case S_PAYLOAD: msgRx->state = msg_rx_payload(msgRx, byte); break;
    case S_CHECKSUM:msgRx->state = msg_rx_checksum(msgRx, byte); break;
    }
}

void EvofwProtocol::msg_rx_rssi(uint8_t rssi) {
    if (msgRx) { // Add safety check
        msgRx->rssi = rssi;
        msgRx->rxFields |= 0x80; // F_RSSI
    }
}
uint8_t *EvofwProtocol::msg_rx_start(void) {
    DEBUG_MSG(1);
    msgRx = msg_alloc();
    if (msgRx) {
        msgRx->raw[0] = MAX_RAW;
        DEBUG_MSG(0);
        return msgRx->raw;
    }
    DEBUG_MSG(0);
    return NULL;
}
uint8_t EvofwProtocol::msg_rx_byte(uint8_t byte) {
    DEBUG_MSG(1);
    msg_rx_process(byte);
    DEBUG_MSG(0);
    return msgRx->error;
}
void EvofwProtocol::msg_rx_end(uint8_t nBytes, uint8_t error) {
    DEBUG_MSG(1);
    if (!msgRx) return; // Safety check
    msgRx->nBytes = nBytes;
    if (error == MSG_OK) {
        if (((msgRx->rxFields & 0x7C) != (msgRx->fields & 0x7C)) // F_OPTION
         || ((msgRx->rxFields & 0x03) != 0x03) // F_MAND
         || (msgRx->len != msgRx->nPayload)) {
            error = MSG_TRUNC_ERR;
        }
    }
    msgRx->error = error;
    msg_rx_ready(&msgRx);
    DEBUG_MSG(0);
}

// --- msg_scan functions ---
uint8_t EvofwProtocol::msg_scan_header(struct message *msg, char *str, uint8_t nChar) {
    uint8_t ok = 0;
    for (uint8_t msgType = 0; msgType <= 3; msgType++) {
        char type_str[3];
        strncpy_P(type_str, (PGM_P)pgm_read_ptr(&MsgType[msgType]), 3);
        if (0 == strcasecmp(str, type_str)) { // Use strcasecmp
            msg->fields = msgType;
            ok = 1;
            break;
        }
    }
    return ok;
}
uint8_t EvofwProtocol::msg_scan_addr(struct message *msg, char *str, uint8_t nChar) {
    uint8_t ok = 0;
    uint8_t addr = msg->state - 2; // S_ADDR0
    if (str[0] != '-') {
        uint8_t class_id;
        uint32_t id;
        if (nChar < 11 && 2 == sscanf(str, "%hhu:%lu", &class_id, &id)) {
            msg_set_address(msg->addr[addr], class_id, id);
            msg->fields |= (0x10 << addr); // F_ADDR0
            ok = 1;
        }
    } else {
        ok = 1;
    }
    return ok;
}
uint8_t EvofwProtocol::msg_scan_param(struct message *msg, char *str, uint8_t nChar) {
    uint8_t ok = 0;
    uint8_t param = msg->state - 5; // S_PARAM0
    if (str[0] != '-') {
        if (nChar < 5 && 1 == sscanf(str, "%hhu", msg->param + param)) {
            msg->fields |= (0x04 << param); // F_PARAM0
            ok = 1;
        }
    } else {
        ok = 1;
    }
    return ok;
}
uint8_t EvofwProtocol::msg_scan_opcode(struct message *msg, char *str, uint8_t nChar) {
    if (nChar == 5 && 2 == sscanf(str, "%02hhx%02hhx", msg->opcode + 0, msg->opcode + 1)) {
        msg->rxFields |= 0x01; // F_OPCODE
        return 1;
    }
    return 0;
}
uint8_t EvofwProtocol::msg_scan_len(struct message *msg, char *str, uint8_t nChar) {
    if (nChar < 5 && 1 == sscanf(str, "%hhu", &msg->len)) {
        if (msg->len <= MAX_PAYLOAD) { // Allow 0 length
            msg->rxFields |= 0x02; // F_LEN
            return 1;
        }
    }
    return 0;
}
uint8_t EvofwProtocol::msg_scan_payload(struct message *msg, char *str, uint8_t nChar) {
    if (nChar == 3 && 1 == sscanf(str, "%02hhx", msg->payload + msg->nPayload)) {
        msg->nPayload++;
        return 1;
    }
    return 0;
}
uint8_t EvofwProtocol::msg_scan(struct message *msg, uint8_t byte) {
    static char field[17];
    static uint8_t nChar = 0;
    uint8_t ok = 1;
    enum { S_START=0, S_HEADER, S_ADDR0, S_ADDR1, S_ADDR2, S_PARAM0, S_PARAM1,
           S_OPCODE, S_LEN, S_PAYLOAD, S_CHECKSUM, S_TRAILER, S_COMPLETE, S_ERROR };

    if (byte == '\n') return 0;
    if (byte == '\r') {
        if (msg->state == S_START && nChar == 0) return 0;
        if (msg->state != S_CHECKSUM) {
            nChar = 0; msg->rxFields |= msg->fields;
            msg->error = MSG_BAD_TX; return 1;
        } else {
            byte = '\0';
        }
    }
    if (msg->nBytes < MAX_RAW)
        msg->raw[msg->nBytes++] = byte;
    if (msg->state == S_ERROR) return 0;
    if (byte == ' ') {
        if (nChar == 0) return 0;
        byte = '\0';
    }
    field[nChar++] = (char)byte;

    if (byte && msg->state == S_PAYLOAD) {
        if (nChar == 2) {
            field[nChar++] = '\0';
            ok = msg_scan_payload(msg, field, nChar);
            if (ok) {
                nChar = 0;
                if (msg->nPayload == msg->len)
                    msg->state = S_CHECKSUM;
            }
        } else {
            return 0;
        }
    }

    if (!byte) {
        switch (msg->state) {
        case S_START:
        case S_HEADER:   ok = msg_scan_header(msg, field, nChar); msg->state = S_PARAM0; break;
        case S_ADDR0:    ok = msg_scan_addr(msg, field, nChar); msg->state = S_ADDR1; break;
        case S_ADDR1:    ok = msg_scan_addr(msg, field, nChar); msg->state = S_ADDR2; break;
        case S_ADDR2:    ok = msg_scan_addr(msg, field, nChar); msg->state = S_OPCODE; break;
        case S_PARAM0:   ok = msg_scan_param(msg, field, nChar); msg->state = S_ADDR0; break;
        case S_OPCODE:   ok = msg_scan_opcode(msg, field, nChar); msg->state = S_LEN; break;
        case S_LEN:      ok = msg_scan_len(msg, field, nChar);
                         msg->state = (msg->len > 0) ? S_PAYLOAD : S_CHECKSUM; break;
        case S_PAYLOAD:  msg->state = S_ERROR; break;
        case S_CHECKSUM: msg->state = (nChar != 1) ? S_ERROR : S_COMPLETE; break;
        }
        nChar = 0;
    }
    if (!ok) msg->state = S_ERROR;
    if (msg->state == S_PAYLOAD && (msg->rxFields & 0x03) != 0x03) // F_MAND
        msg->state = S_ERROR;
    if (msg->state == S_COMPLETE) {
        msg->rxFields |= msg->fields;
        return 1;
    }
    return 0;
}

// --- msg_tx functions ---
uint8_t EvofwProtocol::msg_tx_header(struct message *msg, uint8_t *done) {
    uint8_t byte = 0;
    if (msg->count < 1) { byte = get_header(msg->fields); msg->count++; }
    else { msg->count = 0; }
    (*done) = (msg->count) ? 0 : 1;
    return byte;
}
uint8_t EvofwProtocol::msg_tx_addr(struct message *msg, uint8_t *done) {
    uint8_t byte = 0; uint8_t addr = msg->state - 2; // S_ADDR0
    if (msg->fields & (0x10 << addr)) {
        if (msg->count < 3) { byte = msg->addr[addr][msg->count]; msg->count++; }
        else { msg->count = 0; }
    }
    (*done) = (msg->count) ? 0 : 1;
    return byte;
}
uint8_t EvofwProtocol::msg_tx_param(struct message *msg, uint8_t *done) {
    uint8_t byte = 0; uint8_t param = msg->state - 5; // S_PARAM0
    if (msg->fields & (0x04 << param)) {
        if (msg->count < 1) { byte = msg->param[param]; msg->count++; }
        else { msg->count = 0; }
    }
    (*done) = (msg->count) ? 0 : 1;
    return byte;
}
uint8_t EvofwProtocol::msg_tx_opcode(struct message *msg, uint8_t *done) {
    uint8_t byte = 0;
    if (msg->count < 2) { byte = msg->opcode[msg->count]; msg->count++; }
    else { msg->count = 0; }
    (*done) = (msg->count) ? 0 : 1;
    return byte;
}
uint8_t EvofwProtocol::msg_tx_len(struct message *msg, uint8_t *done) {
    uint8_t byte = 0;
    if (msg->count < 1) { byte = msg->len; msg->count++; }
    else { msg->count = 0; }
    (*done) = (msg->count) ? 0 : 1;
    return byte;
}
uint8_t EvofwProtocol::msg_tx_payload(struct message *msg, uint8_t *done) {
    uint8_t byte = 0;
    if (msg->count < msg->len) { byte = msg->payload[msg->count]; msg->count++; }
    else { msg->count = 0; }
    (*done) = (msg->count) ? 0 : 1;
    return byte;
}
uint8_t EvofwProtocol::msg_tx_checksum(struct message *msg, uint8_t *done) {
    uint8_t byte = 0;
    if (msg->count < 1) { byte = msg->csum; msg->count++; }
    else { msg->count = 0; }
    (*done) = (msg->count) ? 0 : 1;
    return byte;
}
uint8_t EvofwProtocol::msg_tx_process(struct message *msg, uint8_t *done) {
    uint8_t byte = 0, d = 1;
    enum { S_START=0, S_HEADER, S_ADDR0, S_ADDR1, S_ADDR2, S_PARAM0, S_PARAM1,
           S_OPCODE, S_LEN, S_PAYLOAD, S_CHECKSUM, S_TRAILER, S_COMPLETE, S_ERROR };
    switch (msg->state) {
    case S_START:
    case S_HEADER:   byte = msg_tx_header(msg, &d);   if (!d) break; msg->state = S_ADDR0;
    case S_ADDR0:    byte = msg_tx_addr(msg, &d);     if (!d) break; msg->state = S_ADDR1;
    case S_ADDR1:    byte = msg_tx_addr(msg, &d);     if (!d) break; msg->state = S_ADDR2;
    case S_ADDR2:    byte = msg_tx_addr(msg, &d);     if (!d) break; msg->state = S_PARAM0;
    case S_PARAM0:   byte = msg_tx_param(msg, &d);    if (!d) break; msg->state = S_PARAM1;
    case S_PARAM1:   byte = msg_tx_param(msg, &d);    if (!d) break; msg->state = S_OPCODE;
    case S_OPCODE:   byte = msg_tx_opcode(msg, &d);   if (!d) break; msg->state = S_LEN;
    case S_LEN:      byte = msg_tx_len(msg, &d);      if (!d) break; msg->state = S_PAYLOAD;
    case S_PAYLOAD:  byte = msg_tx_payload(msg, &d);  if (!d) break; msg->state = S_CHECKSUM;
    case S_CHECKSUM: byte = msg_tx_checksum(msg, &d); if (!d) break; msg->state = S_COMPLETE;
    case S_TRAILER:
    case S_COMPLETE:
    case S_ERROR:
        break;
    }
    (*done) = d;
    return byte;
}

void EvofwProtocol::msg_tx_start(struct message **msg) {
    if (msg && (*msg)) {
        TxMsg = (*msg);
        TxMsg->csum = msg_checksum(TxMsg);
        frame_tx_start(TxMsg->raw, MAX_RAW);
        (*msg) = nullptr;
    }
}
uint8_t EvofwProtocol::msg_tx_byte(uint8_t *done) {
    uint8_t byte = 0x00;
    if (TxMsg) {
        byte = msg_tx_process(TxMsg, done);
    } else {
        *done = 1;
    }
    return byte;
}
void EvofwProtocol::msg_tx_end(uint8_t nBytes) {
    if (TxMsg) {
        TxMsg->nBytes = nBytes;
    }
}
void EvofwProtocol::msg_tx_done(void) {
    if (TxMsg) {
        TxMsg->rssi = 0;
        TxMsg->error = MSG_OK;
        msg_rx_ready(&TxMsg); // Echo TX to RX queue
    }
}

uint8_t EvofwProtocol::msg_isValid(struct message *msg) {
    return (msg) ? (msg->error == MSG_OK) : 0;
}
uint8_t EvofwProtocol::msg_isTx(struct message *msg) {
    return (msg) ? (msg_isValid(msg) && (msg->rssi == 0)) : 0;
}

void EvofwProtocol::resetToRx(void) {
    // This function forces the radio and state machine back to RX_IDLE.
    // It's a copy of the logic from frame_rx_enable() and frame_rx_reset().
    
    uart_disable(); // Detach interrupts, stop UART
    
    // Force chip to RX mode (which first goes to IDLE)
    _cc1101.enterRxMode(); 

    // If a message was in the middle of transmitting, it is now
    // orphaned. We must free it and clear the TxMsg pointer.
    if (TxMsg) {
        msg_free(&TxMsg); // This also sets TxMsg to NULL
    }
    
    // Purge the entire tx_list queue.
    // These messages are now stale and would fail anyway.
    struct message *stale_msg = msg_tx_get();
    while (stale_msg != NULL) {
        msg_free(&stale_msg);
        stale_msg = msg_tx_get();
    }

    // Reset protocol state machines
    frame_tx_reset(); // Clear any pending TX state
    frame_rx_reset(); // Clear any pending RX state
    frame.state = FRM_RX;
    rxFrm.state = FRM_RX_IDLE;
    
    // Re-enable UART for receiving
    uart_rx_enable(); 
}