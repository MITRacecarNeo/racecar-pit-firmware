#include <string.h>
#include <unity.h>

#include "config.h"
#include "crc16.h"
#include "info.h"
#include "packets.h"

void setUp(void) {}
void tearDown(void) {}

static char payload[info::payload_max_bytes];

static size_t buildPayload(bool imu_ok = true, bool power_ok = true)
{
    // Registers as the shipped config leaves them: gyro 952 Hz, mag 155 Hz
    // fast mode, 8 gauss, continuous.
    const Lsm9ds1Registers regs = {true, 0xC8, 0x78, 0x62, 0x20, 0x00};
    const InfoStatus status = {imu_ok, power_ok, "watchdog", 0x80, true, regs};
    return writeFirmwareInfo(status, payload, sizeof(payload));
}

// Value of `key` in the payload, or nullptr. Points into a static buffer.
static const char *valueOf(const char *key)
{
    static char value[64];
    const size_t key_len = strlen(key);
    const char *line = payload;
    while (*line)
    {
        const char *end = strchr(line, '\n');
        if (!end)
            return nullptr;
        if ((size_t)(end - line) > key_len && strncmp(line, key, key_len) == 0 && line[key_len] == '=')
        {
            const size_t n = (size_t)(end - line) - key_len - 1;
            memcpy(value, line + key_len + 1, n);
            value[n] = '\0';
            return value;
        }
        line = end + 1;
    }
    return nullptr;
}

// ------------------------------ writer ---------------------------- //

static void test_writer_formats_each_value_kind(void)
{
    char buf[128];
    InfoWriter w(buf, sizeof(buf));
    w.text("a", "x");
    w.integer("b", -42);
    w.fixed("c", 0.72f, 3);
    w.fixed("d", -1.5f, 1);
    w.fixed("e", 3.0f, 0);
    w.version("f", 0, 9, 12);
    TEST_ASSERT_EQUAL_STRING("a=x\nb=-42\nc=0.720\nd=-1.5\ne=3\nf=0.9.12\n", buf);
    TEST_ASSERT_EQUAL_UINT32(strlen(buf), w.length());
}

static void test_writer_rounds_and_keeps_small_fractions(void)
{
    char buf[64];
    InfoWriter w(buf, sizeof(buf));
    w.fixed("a", 0.002f, 4);
    w.fixed("b", 0.0004f, 3);
    w.fixed("c", -0.0004f, 3);
    w.fixed("d", 0.9996f, 3);
    TEST_ASSERT_EQUAL_STRING("a=0.0020\nb=0.000\nc=0.000\nd=1.000\n", buf);
}

static void test_writer_reports_overflow_as_zero_length(void)
{
    char buf[8];
    InfoWriter w(buf, sizeof(buf));
    w.text("key", "toolong");
    TEST_ASSERT_EQUAL_UINT32(0, w.length());
}

// ------------------------------ payload --------------------------- //

static void test_payload_fits_and_is_printable_ascii(void)
{
    const size_t n = buildPayload();
    TEST_ASSERT_GREATER_THAN_UINT32(0, n);
    TEST_ASSERT_LESS_THAN_UINT32(info::payload_max_bytes, n);
    TEST_ASSERT_EQUAL_CHAR('\n', payload[n - 1]);
    for (size_t i = 0; i < n; i++)
    {
        const char c = payload[i];
        TEST_ASSERT_TRUE(c == '\n' || (c >= 0x20 && c < 0x7F));
    }
}

// ASCII cannot contain the 0xDE/0xAD/0xBE/0xEF bytes of the telemetry magic,
// so a driver that does not know info frames never syncs inside one.
static void test_payload_never_contains_the_telemetry_magic(void)
{
    const size_t n = buildPayload();
    const uint32_t magic = wire::telemetry_magic;
    for (size_t i = 0; i + 4 <= n; i++)
        TEST_ASSERT_NOT_EQUAL(0, memcmp(payload + i, &magic, 4));
}

static void test_payload_starts_with_schema_and_version(void)
{
    buildPayload();
    TEST_ASSERT_EQUAL_INT(0, strncmp(payload, "info=1\n", 7));
    TEST_ASSERT_EQUAL_STRING("0.9.2", valueOf("fw.version"));
    TEST_ASSERT_EQUAL_STRING("unknown", valueOf("fw.git"));
    TEST_ASSERT_EQUAL_STRING("dev", valueOf("fw.label"));
    TEST_ASSERT_EQUAL_STRING("0", valueOf("fw.profile"));
    TEST_ASSERT_EQUAL_STRING("1", valueOf("fw.protocol"));
}

static void test_payload_reports_the_compiled_config(void)
{
    buildPayload();
    TEST_ASSERT_EQUAL_STRING("1025", valueOf("steer.pwm_min_us"));
    TEST_ASSERT_EQUAL_STRING("1500", valueOf("steer.pwm_center_us"));
    TEST_ASSERT_EQUAL_STRING("1975", valueOf("steer.pwm_max_us"));
    TEST_ASSERT_EQUAL_STRING("1000", valueOf("esc.pwm_min_us"));
    TEST_ASSERT_EQUAL_STRING("2000", valueOf("esc.pwm_max_us"));
    TEST_ASSERT_EQUAL_STRING("1375", valueOf("esc.rc_pwm_min_us"));
    TEST_ASSERT_EQUAL_STRING("1625", valueOf("esc.rc_pwm_max_us"));
    TEST_ASSERT_EQUAL_STRING("0.500", valueOf("speed.min_mps"));
    TEST_ASSERT_EQUAL_STRING("1.000", valueOf("speed.max_mps"));
    TEST_ASSERT_EQUAL_STRING("1.000", valueOf("speed.slow_mps"));
    TEST_ASSERT_EQUAL_STRING("0.010", valueOf("speed.deadband"));
    TEST_ASSERT_EQUAL_STRING("20.000", valueOf("pid.kp"));
    TEST_ASSERT_EQUAL_STRING("200", valueOf("link.telemetry_hz"));
    TEST_ASSERT_EQUAL_STRING("12", valueOf("encoder.edges_per_motor_rev"));
    TEST_ASSERT_EQUAL_STRING("0.0720", valueOf("drivetrain.wheel_diameter_m"));
    TEST_ASSERT_EQUAL_STRING("16", valueOf("power.avg_samples"));
    TEST_ASSERT_EQUAL_STRING("10", valueOf("power.poll_ms"));
    TEST_ASSERT_EQUAL_STRING("250", valueOf("imu.read_hz"));
    TEST_ASSERT_EQUAL_STRING("400000", valueOf("i2c.clock_hz"));
    TEST_ASSERT_EQUAL_STRING("rad/s", valueOf("imu.gyro_unit"));
}

static void test_payload_reports_sensor_status(void)
{
    buildPayload(true, false);
    TEST_ASSERT_EQUAL_STRING("ok", valueOf("status.imu"));
    TEST_ASSERT_EQUAL_STRING("missing", valueOf("status.power"));
    TEST_ASSERT_EQUAL_STRING("armed", valueOf("status.watchdog"));
    TEST_ASSERT_EQUAL_STRING("watchdog", valueOf("status.last_reset"));
    TEST_ASSERT_EQUAL_STRING("128", valueOf("status.reset_flags"));
    TEST_ASSERT_EQUAL_STRING("100", valueOf("watchdog.timeout_ms"));
    TEST_ASSERT_EQUAL_STRING("952", valueOf("imu.gyro_odr_hz"));
    TEST_ASSERT_EQUAL_STRING("952", valueOf("imu.accel_odr_hz"));
    TEST_ASSERT_EQUAL_STRING("155.000", valueOf("imu.mag_odr_hz"));
    TEST_ASSERT_EQUAL_STRING("8", valueOf("imu.mag_range_read_gauss"));
    TEST_ASSERT_EQUAL_STRING("200", valueOf("imu.reg_ctrl1_g"));
}

static void test_payload_keys_are_unique(void)
{
    const size_t n = buildPayload();
    static char keys[128][48];
    size_t count = 0;
    for (const char *line = payload; line < payload + n;)
    {
        const char *eq = strchr(line, '=');
        const char *end = strchr(line, '\n');
        TEST_ASSERT_NOT_NULL(eq);
        TEST_ASSERT_TRUE(eq < end);
        const size_t len = (size_t)(eq - line);
        TEST_ASSERT_LESS_THAN_UINT32(sizeof(keys[0]), len);
        TEST_ASSERT_LESS_THAN_UINT32(128, count);
        memcpy(keys[count], line, len);
        keys[count][len] = '\0';
        for (size_t j = 0; j < count; j++)
            TEST_ASSERT_NOT_EQUAL(0, strcmp(keys[j], keys[count]));
        count++;
        line = end + 1;
    }
}

// ----------------------------- LSM9DS1 ---------------------------- //

static void test_lsm9ds1_gyro_and_accel_rates(void)
{
    Lsm9ds1Registers r = {true, 0x00, 0x60, 0, 0, 0};
    TEST_ASSERT_EQUAL_UINT16(0, lsm9ds1GyroOdrHz(r));
    TEST_ASSERT_EQUAL_UINT16(119, lsm9ds1AccelOdrHz(r)); // gyro off: ODR_XL counts
    r.ctrl1_g = 0x80;                                     // gyro 238 Hz
    TEST_ASSERT_EQUAL_UINT16(238, lsm9ds1GyroOdrHz(r));
    TEST_ASSERT_EQUAL_UINT16(238, lsm9ds1AccelOdrHz(r)); // gyro on: follows the gyro
}

static void test_lsm9ds1_mag_rate_and_range(void)
{
    Lsm9ds1Registers r = {true, 0, 0, 0x10, 0x00, 0x00}; // DO = 100: 10 Hz, 4 gauss
    TEST_ASSERT_EQUAL_FLOAT(10.0f, lsm9ds1MagOdrHz(r));
    TEST_ASSERT_EQUAL_UINT8(4, lsm9ds1MagRangeGauss(r));
    r.ctrl1_m = 0x02; // FAST_ODR, low-power mode
    TEST_ASSERT_EQUAL_FLOAT(1000.0f, lsm9ds1MagOdrHz(r));
    r.ctrl2_m = 0x60;
    TEST_ASSERT_EQUAL_UINT8(16, lsm9ds1MagRangeGauss(r));
    r.ctrl3_m = 0x03; // power-down
    TEST_ASSERT_EQUAL_FLOAT(0.0f, lsm9ds1MagOdrHz(r));
}

static void test_payload_without_imu_reports_zero_rates(void)
{
    const InfoStatus status = {false, true, "reset", 1, false, Lsm9ds1Registers{}};
    writeFirmwareInfo(status, payload, sizeof(payload));
    TEST_ASSERT_EQUAL_STRING("0", valueOf("imu.gyro_odr_hz"));
    TEST_ASSERT_EQUAL_STRING("0.000", valueOf("imu.mag_odr_hz"));
    TEST_ASSERT_EQUAL_STRING("missing", valueOf("status.imu"));
}

static void test_payload_reports_power_range_and_dips(void)
{
    InfoStatus status = {true, true, "reset", 1, true, Lsm9ds1Registers{}};
    writeFirmwareInfo(status, payload, sizeof(payload));
    TEST_ASSERT_EQUAL_STRING("40.0", valueOf("power.max_current_a"));
    TEST_ASSERT_EQUAL_STRING("7200", valueOf("power.dip_mv"));
    TEST_ASSERT_EQUAL_STRING("100", valueOf("power.dip_min_ms"));
    TEST_ASSERT_EQUAL_STRING("-1", valueOf("power.min_mv"));
    TEST_ASSERT_EQUAL_STRING("0", valueOf("power.dip_count"));
    TEST_ASSERT_EQUAL_STRING("-1", valueOf("power.last_dip_ms"));

    status.dips = {true, 7.1234f, 3, 123456};
    writeFirmwareInfo(status, payload, sizeof(payload));
    TEST_ASSERT_EQUAL_STRING("7123", valueOf("power.min_mv"));
    TEST_ASSERT_EQUAL_STRING("3", valueOf("power.dip_count"));
    TEST_ASSERT_EQUAL_STRING("123456", valueOf("power.last_dip_ms"));
}

static void test_payload_reports_stall_protection(void)
{
    buildPayload();
    TEST_ASSERT_EQUAL_STRING("1", valueOf("fw.stall_protect"));
    TEST_ASSERT_EQUAL_STRING("10.0", valueOf("stall.trip_a"));
    TEST_ASSERT_EQUAL_STRING("0.10", valueOf("stall.moving_mps"));
    TEST_ASSERT_EQUAL_STRING("500", valueOf("stall.trip_ms"));
    TEST_ASSERT_EQUAL_STRING("2000", valueOf("stall.cooldown_ms"));
    TEST_ASSERT_EQUAL_STRING("30000", valueOf("stall.lockout_ms"));
    TEST_ASSERT_EQUAL_STRING("3", valueOf("stall.lockout_trips"));
    TEST_ASSERT_EQUAL_STRING("60000", valueOf("stall.lockout_window_ms"));
}

// ------------------------------ chunks ---------------------------- //

static void test_chunk_count(void)
{
    TEST_ASSERT_EQUAL_UINT16(1, infoChunkCount(0));
    TEST_ASSERT_EQUAL_UINT16(1, infoChunkCount(1));
    TEST_ASSERT_EQUAL_UINT16(1, infoChunkCount(wire::info_chunk_bytes));
    TEST_ASSERT_EQUAL_UINT16(2, infoChunkCount(wire::info_chunk_bytes + 1));
}

// Byte-for-byte reference shared with tests/scripts/test_pit_uart.py.
static void test_packet_matches_the_reference_bytes(void)
{
    const uint8_t expected[] = {0xDE, 0xC0, 0xED, 0xFE, 0x01, 0x03, 0x19, 0x00, 0x07, 0x00, 0x00, 0x00,
                                0x00, 0x00, 0x01, 0x00, 0x69, 0x6E, 0x66, 0x6F, 0x3D, 0x31, 0x0A, 0xD4, 0x60};
    uint8_t out[info_packet_max_bytes];
    const size_t n = buildInfoPacket("info=1\n", 7, 3, 0, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(sizeof(expected), n);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, out, sizeof(expected));
}

static void test_chunks_reassemble_to_the_payload(void)
{
    const size_t n = buildPayload();
    const uint16_t count = infoChunkCount(n);
    TEST_ASSERT_GREATER_THAN_UINT16(1, count);

    static char joined[info::payload_max_bytes];
    size_t joined_len = 0;
    for (uint16_t i = 0; i < count; i++)
    {
        uint8_t out[info_packet_max_bytes];
        const size_t len = buildInfoPacket(payload, n, 7, i, out, sizeof(out));
        TEST_ASSERT_GREATER_THAN_UINT32(0, len);

        HeaderPacket h;
        memcpy(&h, out, sizeof(h));
        TEST_ASSERT_EQUAL_HEX32(wire::info_magic, h.MagicNum);
        TEST_ASSERT_EQUAL_UINT8(7, h.IDNumber);
        TEST_ASSERT_EQUAL_UINT16(len, h.PacketBytes);
        TEST_ASSERT_EQUAL_UINT32(n, h.FrameBytes);
        TEST_ASSERT_EQUAL_UINT16(i, h.PacketIndex);
        TEST_ASSERT_EQUAL_UINT16(count, h.PacketsPerFrame);

        uint16_t crc;
        memcpy(&crc, out + len - 2, sizeof(crc));
        TEST_ASSERT_EQUAL_HEX16(crc16_ccitt(out, len - 2), crc);

        const size_t chunk = len - sizeof(h) - 2;
        memcpy(joined + joined_len, out + sizeof(h), chunk);
        joined_len += chunk;
    }
    TEST_ASSERT_EQUAL_UINT32(n, joined_len);
    TEST_ASSERT_EQUAL_INT(0, memcmp(payload, joined, n));
}

static void test_packet_rejects_bad_index_and_small_buffer(void)
{
    uint8_t out[info_packet_max_bytes];
    TEST_ASSERT_EQUAL_UINT32(0, buildInfoPacket("info=1\n", 7, 0, 1, out, sizeof(out)));
    TEST_ASSERT_EQUAL_UINT32(0, buildInfoPacket("info=1\n", 7, 0, 0, out, 10));
}

// One telemetry frame plus the largest chunk must fit one slot on the wire.
static void test_slot_fits_on_the_wire(void)
{
    const float slot_us = 1e6f / cfg::UART::tx_freq;
    const float bytes = (float)(sizeof(TXPacket) + info_packet_max_bytes);
    const float wire_us = bytes * 10.0f * 1e6f / cfg::UART::baud_rate;
    TEST_ASSERT_TRUE(wire_us < slot_us);
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_writer_formats_each_value_kind);
    RUN_TEST(test_writer_rounds_and_keeps_small_fractions);
    RUN_TEST(test_writer_reports_overflow_as_zero_length);
    RUN_TEST(test_payload_fits_and_is_printable_ascii);
    RUN_TEST(test_payload_never_contains_the_telemetry_magic);
    RUN_TEST(test_payload_starts_with_schema_and_version);
    RUN_TEST(test_payload_reports_the_compiled_config);
    RUN_TEST(test_payload_reports_sensor_status);
    RUN_TEST(test_payload_keys_are_unique);
    RUN_TEST(test_lsm9ds1_gyro_and_accel_rates);
    RUN_TEST(test_lsm9ds1_mag_rate_and_range);
    RUN_TEST(test_payload_without_imu_reports_zero_rates);
    RUN_TEST(test_chunk_count);
    RUN_TEST(test_packet_matches_the_reference_bytes);
    RUN_TEST(test_chunks_reassemble_to_the_payload);
    RUN_TEST(test_packet_rejects_bad_index_and_small_buffer);
    RUN_TEST(test_slot_fits_on_the_wire);
    RUN_TEST(test_payload_reports_power_range_and_dips);
    RUN_TEST(test_payload_reports_stall_protection);
    return UNITY_END();
}
