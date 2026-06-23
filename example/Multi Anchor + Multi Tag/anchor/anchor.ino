#include "dw3000.h"

// 0xA0~0xA7
#define ANCHOR_ID           0xA0

#define NET_PAN_ID          0xDECA
#define MASTER_ANCHOR_ID    0xA0

#define FUNC_POLL           0xE0
#define FUNC_RESPONSE       0xE1
#define FUNC_REPORT         0xE2
#define FUNC_BEACON         0xE3

#define TDMA_FRAME_MS       1400
#define SLOT_DURATION_MS    150
#define BEACON_ENABLE       1
#define BEACON_START_DELAY_MS 5000
#define BEACON_TX_TIMEOUT_MS 20

#define AIDX                ((ANCHOR_ID) & 0x0F)
#define RESP_DLY            (2000 + AIDX * 5000)

#define MAX_ANCHORS         8

const uint8_t PIN_RST = 27;
const uint8_t PIN_IRQ = 34;
const uint8_t PIN_SS = 4;

static dwt_config_t config = {
    5, DWT_PLEN_128, DWT_PAC8, 9, 9,
    1, DWT_BR_6M8, DWT_PHRMODE_STD, DWT_PHRRATE_STD,
    (129 + 8 - 8), DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

static uint8_t tx_resp_msg[] = {
    0x41, 0x88, 0,
    0, 0, 0, 0, 0, 0,
    FUNC_RESPONSE,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0
};

static uint8_t tx_beacon_msg[] = {
    0x41, 0x88, 0,
    0, 0, 0xFF, 0xFF, 0, 0,
    FUNC_BEACON,
    0,
    0, 0,
    0, 0,
    0, 0
};

#define RESP_POLL_RX_TS_IDX 10
#define RESP_TX_TS_IDX      14
#define BEACON_SEQ_IDX      10
#define BEACON_FRAME_IDX    11
#define BEACON_SLOT_IDX     13
#define REPORT_MASK_IDX     10
#define REPORT_RANGE_IDX    11
#define RX_BUF_LEN          32

static uint8_t rx_buffer[RX_BUF_LEN];
static uint32_t status_reg = 0;
static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;
static uint64_t last_rx_ts;
static uint32_t delayed_tx_fail_count = 0;
static uint32_t beacon_tx_fail_count = 0;
static uint32_t last_beacon_tx_ms = 0;
static uint8_t beacon_sequence = 0;

extern dwt_txconfig_t txconfig_options;

static void send_beacon(void);
static void handle_poll(void);
static void handle_report(uint16_t data_len);

static inline uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static inline void write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

void setup()
{
    UART_init();

    tx_resp_msg[7] = (uint8_t)(ANCHOR_ID);
    tx_resp_msg[8] = (uint8_t)(ANCHOR_ID >> 8);

    tx_beacon_msg[3] = (uint8_t)(NET_PAN_ID);
    tx_beacon_msg[4] = (uint8_t)(NET_PAN_ID >> 8);
    tx_beacon_msg[7] = (uint8_t)(ANCHOR_ID);
    tx_beacon_msg[8] = (uint8_t)(ANCHOR_ID >> 8);
    write_u16_le(&tx_beacon_msg[BEACON_FRAME_IDX], TDMA_FRAME_MS);
    write_u16_le(&tx_beacon_msg[BEACON_SLOT_IDX], SLOT_DURATION_MS);

    spiBegin(PIN_IRQ, PIN_RST);
    spiSelect(PIN_SS);
    delay(2);

    dwt_softreset();
    delay(2);

    while (!dwt_checkidlerc())
    {
        test_run_info((unsigned char *)"IDLE FAILED\r\n");
        while (1);
    }

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        test_run_info((unsigned char *)"INIT FAILED\r\n");
        while (1);
    }

    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    if (dwt_configure(&config))
    {
        test_run_info((unsigned char *)"CONFIG FAILED\r\n");
        while (1);
    }

    dwt_configuretxrf(&txconfig_options);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
}

void loop()
{
    if (BEACON_ENABLE && ANCHOR_ID == MASTER_ANCHOR_ID && millis() > BEACON_START_DELAY_MS && millis() - last_beacon_tx_ms >= TDMA_FRAME_MS)
    {
        send_beacon();
    }

    dwt_setrxtimeout(20 * 1000UL);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
    {
    }

    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
    {
        last_rx_ts = get_rx_timestamp_u64();
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

        uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
        uint16_t data_len = frame_len - FCS_LEN;

        if (data_len <= sizeof(rx_buffer) && data_len > 0)
        {
            dwt_readrxdata(rx_buffer, data_len, 0);
            uint16_t rx_pan_id = read_u16_le(&rx_buffer[3]);

            if (rx_pan_id != NET_PAN_ID)
            {
                return;
            }

            uint8_t function_code = rx_buffer[9];

            if (function_code == FUNC_POLL)
            {
                handle_poll();
            }
            else if (function_code == FUNC_REPORT)
            {
                handle_report(data_len);
            }
        }
    }
    else
    {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    }
}

// A0 广播 Beacon，为所有 Tag 提供 TDMA epoch
static void send_beacon()
{
    tx_beacon_msg[BEACON_SEQ_IDX] = beacon_sequence++;

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    dwt_writetxdata(sizeof(tx_beacon_msg), tx_beacon_msg, 0);
    dwt_writetxfctrl(sizeof(tx_beacon_msg), 0, 0);

    if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS)
    {
        uint32_t beacon_tx_start_time = millis();

        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
        {
            if (millis() - beacon_tx_start_time > BEACON_TX_TIMEOUT_MS)
            {
                beacon_tx_fail_count++;
                break;
            }

            delay(1);
        }

        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    }
    else
    {
        beacon_tx_fail_count++;
    }

    last_beacon_tx_ms = millis();
}

// 处理 Tag Poll 并按 Anchor ID 延迟回复 Response
static void handle_poll()
{
    poll_rx_ts = last_rx_ts;

    uint32_t resp_tx_time = (poll_rx_ts + ((uint64_t)RESP_DLY * UUS_TO_DWT_TIME)) >> 8;
    dwt_setdelayedtrxtime(resp_tx_time);

    resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

    tx_resp_msg[2] = rx_buffer[2];
    tx_resp_msg[3] = rx_buffer[3];
    tx_resp_msg[4] = rx_buffer[4];
    tx_resp_msg[5] = rx_buffer[7];
    tx_resp_msg[6] = rx_buffer[8];

    resp_msg_set_ts(&tx_resp_msg[RESP_POLL_RX_TS_IDX], poll_rx_ts);
    resp_msg_set_ts(&tx_resp_msg[RESP_TX_TS_IDX], resp_tx_ts);

    dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
    dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);

    int ret = dwt_starttx(DWT_START_TX_DELAYED);

    if (ret == DWT_SUCCESS)
    {
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
        {
        }

        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    }
    else
    {
        delayed_tx_fail_count++;
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_TXFRS_BIT_MASK);
    }
}

// 处理 Tag Report 并输出统一结果
static void handle_report(uint16_t data_len)
{
    if (data_len < REPORT_RANGE_IDX + MAX_ANCHORS * 2)
    {
        return;
    }

    uint16_t tag_id = read_u16_le(&rx_buffer[7]);
    uint8_t report_mask = rx_buffer[REPORT_MASK_IDX];

    Serial.print("T");
    Serial.print(tag_id);
    Serial.print(",mask:");
    Serial.print(report_mask, HEX);
    Serial.print(",seq:");
    Serial.print(rx_buffer[2]);
    Serial.print(",fail:");
    Serial.print(delayed_tx_fail_count);
    Serial.print(",range:(");

    for (int i = 0; i < 8; i++)
    {
        if (i > 0)
        {
            Serial.print(",");
        }

        int16_t distance_cm = (int16_t)read_u16_le(&rx_buffer[REPORT_RANGE_IDX + i * 2]);
        Serial.print(distance_cm);
    }

    Serial.print("),ancid:(");

    for (int i = 0; i < 8; i++)
    {
        if (i > 0)
        {
            Serial.print(",");
        }

        Serial.print((report_mask & (1 << i)) ? i : -1);
    }

    Serial.println(")");
}
