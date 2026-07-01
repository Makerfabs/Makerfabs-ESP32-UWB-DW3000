#include "board_identity.h"
#include <string.h>

void print_hex64(uint64_t value)
{
    uint32_t msb = (uint32_t)(value >> 32);
    uint32_t lsb = (uint32_t)(value & 0xFFFFFFFF);
    if (msb == 0)
    {
        Serial.print("0x");
    }
    else
    {
        Serial.print("0x");
        if (msb < 0x1000)
            Serial.print("0");
        if (msb < 0x100)
            Serial.print("0");
        if (msb < 0x10)
            Serial.print("0");
        Serial.print(msb, HEX);
    }
    if (lsb < 0x10000000)
        Serial.print("0");
    if (lsb < 0x1000000)
        Serial.print("0");
    if (lsb < 0x100000)
        Serial.print("0");
    if (lsb < 0x10000)
        Serial.print("0");
    if (lsb < 0x1000)
        Serial.print("0");
    if (lsb < 0x100)
        Serial.print("0");
    if (lsb < 0x10)
        Serial.print("0");
    Serial.print(lsb, HEX);
}

static uint16_t to_iotlab_id(uint64_t cpu_id)
{
    uint8_t id_tab[8];
    memcpy(id_tab, &cpu_id, sizeof(id_tab));
    return (uint16_t)(((id_tab[4] | (id_tab[6] << 7)) << 8) | id_tab[5]);
}

void setup()
{
    Serial.begin(115200);
}

void loop()
{
    uint64_t mac = board_get_dw_mac();
    uint64_t cpuid = board_get_cpu_id();
    uint16_t short_addr = board_get_short_address();
    uint16_t iotlab_id = to_iotlab_id(cpuid);

    Serial.print("DW MAC: ");
    print_hex64(mac);
    Serial.print(" | CPU ID: ");
    print_hex64(cpuid);
    Serial.print(" | short addr: 0x");
    if (short_addr < 0x1000)
        Serial.print("0");
    if (short_addr < 0x100)
        Serial.print("0");
    if (short_addr < 0x10)
        Serial.print("0");
    Serial.println(short_addr, HEX);
    Serial.print("IoT Lab UID: 0x");
    if (iotlab_id < 0x1000)
        Serial.print("0");
    if (iotlab_id < 0x100)
        Serial.print("0");
    if (iotlab_id < 0x10)
        Serial.print("0");
    Serial.println(iotlab_id, HEX);

    delay(5000);
}
