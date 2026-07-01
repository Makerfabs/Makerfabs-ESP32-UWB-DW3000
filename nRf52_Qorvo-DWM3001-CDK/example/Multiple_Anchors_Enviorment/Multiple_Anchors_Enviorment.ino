#include "dwm3001.h"
#include "board_identity.h"
#include <string.h>
#include <cmath>


#define APP_NAME "SS TWR MULTI-ANCHOR v1.0"

// connection pins
const uint8_t PIN_RST = DW3000_RST_PIN; // reset pin
const uint8_t PIN_IRQ = DW3000_IRQ_PIN; // irq pin
const uint8_t PIN_SS = DW3000_CS_PIN;   // spi select pin

//sender
//#define SENDER_ID 0xC4C8
#define SENDER_ID 0x5E16

static uint16_t get_node_id()
{
  uint64_t cpu_id = board_get_cpu_id();
  uint8_t id_tab[8];
  memcpy(id_tab, &cpu_id, sizeof(id_tab));
  return (uint16_t)(((id_tab[4] | (id_tab[6] << 7)) << 8) | id_tab[5]);
}

static bool is_initiator()
{
  return get_node_id() == SENDER_ID;
}

//List of anchors  
static const uint16_t ANCHOR_IDS[] = {
  0xCA94,
  0xEB86,
  0x22A1,
  0xE7D4,
  0xAF78,
};

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
        5,               /* Channel number. */
        DWT_PLEN_128,    /* Preamble length. Used in TX only. */
        DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
        9,               /* TX preamble code. Used in TX only. */
        9,               /* RX preamble code. Used in RX only. */
        1,               /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
        DWT_BR_6M8,      /* Data rate. */
        DWT_PHRMODE_STD, /* PHY header mode. */
        DWT_PHRRATE_STD, /* PHY header rate. */
        (129 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
        DWT_STS_MODE_OFF, /* STS disabled */
        DWT_STS_LEN_64,/* STS length see allowed values in Enum dwt_sts_lengths_e */
        DWT_PDOA_M0      /* PDOA mode off */
};

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 100

/* Default antenna delay values for 64 MHz PRF. */
#define TX_ANT_DLY 16380
#define RX_ANT_DLY 16380

/* Frames used in the ranging process. */
static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, 0xE0, 0, 0};
static uint8_t rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, 0xE0, 0, 0};
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4

/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

#define RX_BUF_LEN 20
static uint8_t rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;

/* Delay between frames, in UWB microseconds. */
#if defined(ARDUINO_ARCH_NRF5)
#define HOST_PROC_COMPENSATION_UUS 300
#else
#define HOST_PROC_COMPENSATION_UUS 0
#endif

#define POLL_TX_TO_RESP_RX_DLY_UUS (240 + HOST_PROC_COMPENSATION_UUS)
#define RESP_RX_TIMEOUT_UUS (400 + HOST_PROC_COMPENSATION_UUS)
#define POLL_RX_TO_RESP_TX_DLY_UUS (450 + HOST_PROC_COMPENSATION_UUS)

/* Hold copies of computed time of flight and distance here for reference. */
static double tof;
static double distance;

/* Timestamps of frames transmission/reception. */
static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. */
extern dwt_txconfig_t txconfig_options;

static void set_initiator_addresses(uint16_t target_id)
{
    tx_poll_msg[3] = 0xCA;
    tx_poll_msg[4] = 0xDE;
    tx_poll_msg[5] = (uint8_t)(target_id & 0xFF);
    tx_poll_msg[6] = (uint8_t)(target_id >> 8);
    tx_poll_msg[7] = (uint8_t)(SENDER_ID & 0xFF);
    tx_poll_msg[8] = (uint8_t)(SENDER_ID >> 8);

    rx_resp_msg[3] = 0xCA;
    rx_resp_msg[4] = 0xDE;
    rx_resp_msg[5] = (uint8_t)(SENDER_ID & 0xFF);
    rx_resp_msg[6] = (uint8_t)(SENDER_ID >> 8);
    rx_resp_msg[7] = (uint8_t)(target_id & 0xFF);
    rx_resp_msg[8] = (uint8_t)(target_id >> 8);
}

static void set_responder_addresses(uint16_t my_id)
{
    rx_poll_msg[3] = 0xCA;
    rx_poll_msg[4] = 0xDE;
    rx_poll_msg[5] = (uint8_t)(my_id & 0xFF);
    rx_poll_msg[6] = (uint8_t)(my_id >> 8);
    rx_poll_msg[7] = (uint8_t)(SENDER_ID & 0xFF);
    rx_poll_msg[8] = (uint8_t)(SENDER_ID >> 8);

    tx_resp_msg[3] = 0xCA;
    tx_resp_msg[4] = 0xDE;
    tx_resp_msg[5] = (uint8_t)(SENDER_ID & 0xFF);
    tx_resp_msg[6] = (uint8_t)(SENDER_ID >> 8);
    tx_resp_msg[7] = (uint8_t)(my_id & 0xFF);
    tx_resp_msg[8] = (uint8_t)(my_id >> 8);
}

void setup() {
  UART_init();
  test_run_info((unsigned char *)APP_NAME);

  /* Configure SPI rate, DW3000 supports up to 38 MHz */
  /* Reset DW IC */
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);

  delay(2); // Time needed for DW3000 to start up

  while (!dwt_checkidlerc())
  {
    UART_puts("IDLE FAILED\r\n");
    while (1) ;
  }

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
  {
    UART_puts("INIT FAILED\r\n");
    while (1) ;
  }

  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  if(dwt_configure(&config))
  {
    UART_puts("CONFIG FAILED\r\n");
    while (1) ;
  }

  if (is_initiator())
  {
    set_initiator_addresses(ANCHOR_IDS[0]);
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
  }
  else
  {
    set_responder_addresses(get_node_id());
  }

  /* Configure the TX spectrum parameters (power, PG delay and PG count) */
  dwt_configuretxrf(&txconfig_options);

  /* Apply default antenna delay value. */
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
}

static void loop_initiator(void) {
        static size_t anchor_index = 0;
        static bool cycle_started = false;
        static bool first_in_cycle = true;
        static char cycle_buf[768];
        static size_t cycle_len = 0;
        uint16_t target_id = ANCHOR_IDS[anchor_index];

        if (!cycle_started)
        {
            cycle_len = (size_t)snprintf(cycle_buf,
                                         sizeof(cycle_buf),
                                         "{\"initiator\":\"0x%04X\",\"anchors\":[",
                                         SENDER_ID);
            cycle_started = true;
            first_in_cycle = true;
        }

        set_initiator_addresses(target_id);

        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
        dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);

        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
        { };

        frame_seq_nb++;

        if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
        {
            uint32_t frame_len;

            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
            if (frame_len <= sizeof(rx_buffer))
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);

                rx_buffer[ALL_MSG_SN_IDX] = 0;
                if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
                {
                  uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
                  int32_t rtd_init, rtd_resp;
                  float clockOffsetRatio;

                  /* Retrieve poll transmission and response reception timestamps. See NOTE 9 below. */
                  poll_tx_ts = dwt_readtxtimestamplo32();
                  resp_rx_ts = dwt_readrxtimestamplo32();

                  /* Read carrier integrator value and calculate clock offset ratio. See NOTE 11 below. */
                  clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

                  /* Get timestamps embedded in response message. */
                  resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
                  resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

                  /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
                  rtd_init = resp_rx_ts - poll_tx_ts;
                  rtd_resp = resp_tx_ts - poll_rx_ts;

                  tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
                  distance = tof * SPEED_OF_LIGHT; 
                  //distance = fabs(distance);
                  // if (distance < 0)
                  // {
                  //   distance = 0;
                  // }


                    long meters_part = (long)distance;
                    long centimeters_part = (long)((distance - meters_part) * 100.0 + 0.5);
                    if (centimeters_part >= 100)
                    {
                        meters_part += 1;
                        centimeters_part -= 100;
                    }

                    char json_msg[192];
                    if (!first_in_cycle && cycle_len < sizeof(cycle_buf) - 1)
                    {
                        cycle_buf[cycle_len++] = ',';
                        cycle_buf[cycle_len] = '\0';
                    }

                    snprintf(json_msg,
                             sizeof(json_msg),
                             "{\"target\":\"0x%04X\",\"t1\":%lu,\"t2\":%lu,\"t3\":%lu,\"t4\":%lu,\"Ranging\":%ld.%02ld}",
                             target_id,
                             (unsigned long)poll_tx_ts,
                             (unsigned long)poll_rx_ts,
                             (unsigned long)resp_tx_ts,
                             (unsigned long)resp_rx_ts,
                             meters_part,
                             centimeters_part);
                    if (cycle_len < sizeof(cycle_buf) - 1)
                    {
                        size_t remaining = sizeof(cycle_buf) - cycle_len;
                        int wrote = snprintf(cycle_buf + cycle_len, remaining, "%s", json_msg);
                        if (wrote > 0 && (size_t)wrote < remaining)
                        {
                            cycle_len += (size_t)wrote;
                        }
                    }
                    first_in_cycle = false;
                }
            }
        }
        else
        {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        }

        anchor_index++;
        if (anchor_index >= (sizeof(ANCHOR_IDS) / sizeof(ANCHOR_IDS[0])))
        {
            anchor_index = 0;
            if (cycle_len < sizeof(cycle_buf) - 3)
            {
                cycle_buf[cycle_len++] = ']';
                cycle_buf[cycle_len++] = '}';
                cycle_buf[cycle_len] = '\0';
            }
            test_run_info((unsigned char *)cycle_buf);
            cycle_started = false;
        }

        Sleep(RNG_DELAY_MS);
}

static void loop_responder(void) {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)))
        { };

        if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
        {
            uint32_t frame_len;

            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
            if (frame_len <= sizeof(rx_buffer))
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);

                rx_buffer[ALL_MSG_SN_IDX] = 0;
                if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
                {
                    uint32_t resp_tx_time;
                    int ret;

                    poll_rx_ts = get_rx_timestamp_u64();

                    resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                    dwt_setdelayedtrxtime(resp_tx_time);

                    resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

                    resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
                    resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

                    tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                    dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
                    dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);
                    ret = dwt_starttx(DWT_START_TX_DELAYED);

                    if (ret == DWT_SUCCESS)
                    {
                        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
                        { };

                        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

                        frame_seq_nb++;
                    }
                }
            }
        }
        else
        {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        }
}

void loop() {
  if (is_initiator())
  {
    loop_initiator();
  }
  else
  {
    loop_responder();
  }
}
