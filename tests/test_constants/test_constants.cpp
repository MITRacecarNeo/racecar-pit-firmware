#include <string.h>
#include <unity.h>

#include "car_constants.h"
#include "config.h"

namespace
{
  const SpeedConstants D = cfg::PID_CTRL::speed_defaults;
  uint8_t block[car_constants::block_bytes];
}

void setUp() { memset(block, 0, sizeof(block)); }
void tearDown() {}

void test_defaults_match_the_fleet_config()
{
  TEST_ASSERT_EQUAL_FLOAT(69.0f, D.ff_fwd_us);
  TEST_ASSERT_EQUAL_FLOAT(12.5f, D.kv_ff_fwd);
  TEST_ASSERT_EQUAL_FLOAT(94.0f, D.ff_rev_us);
  TEST_ASSERT_EQUAL_FLOAT(55.0f, D.kv_ff_rev);
  TEST_ASSERT_EQUAL_FLOAT(20.0f, D.kp);
  TEST_ASSERT_EQUAL_FLOAT(60.0f, D.kp_boost);
  TEST_ASSERT_EQUAL_FLOAT(0.25f, D.boost_error_mps);
  TEST_ASSERT_EQUAL_FLOAT(60.0f, D.ki);
  TEST_ASSERT_EQUAL_FLOAT(60.0f, D.ki_rev);
  TEST_ASSERT_EQUAL_FLOAT(160.0f, D.start_rev_from_us);
  TEST_ASSERT_EQUAL_FLOAT(320.0f, D.start_rev_max_us);
  TEST_ASSERT_EQUAL_INT(-1, firstOutOfRange(D));
}

void test_block_round_trip()
{
  SpeedConstants c = D;
  c.ff_rev_us = 110.0f;
  encodeBlock(c, 20261106, block);
  SpeedConstants out = {};
  uint32_t ymd = 0;
  uint16_t crc = 0;
  TEST_ASSERT_EQUAL(BlockStatus::ok, decodeBlock(block, out, ymd, crc));
  TEST_ASSERT_TRUE(sameConstants(c, out));
  TEST_ASSERT_EQUAL_UINT32(20261106, ymd);
  TEST_ASSERT_EQUAL_HEX16(block[car_constants::block_bytes - 2] | (block[car_constants::block_bytes - 1] << 8), crc);
}

void test_erased_eeprom_is_empty()
{
  memset(block, 0xFF, sizeof(block));
  SpeedConstants out = D;
  uint32_t ymd = 7;
  uint16_t crc = 0;
  TEST_ASSERT_EQUAL(BlockStatus::empty, decodeBlock(block, out, ymd, crc));
  TEST_ASSERT_TRUE(sameConstants(D, out)); // untouched
  TEST_ASSERT_EQUAL_UINT32(7, ymd);
}

void test_any_flipped_byte_is_rejected()
{
  encodeBlock(D, 20261106, block);
  for (size_t i = 0; i < car_constants::block_bytes; i++)
  {
    uint8_t copy[car_constants::block_bytes];
    memcpy(copy, block, sizeof(copy));
    copy[i] ^= 0x10;
    SpeedConstants out;
    uint32_t ymd;
    uint16_t crc;
    TEST_ASSERT_NOT_EQUAL(BlockStatus::ok, decodeBlock(copy, out, ymd, crc));
  }
}

void test_wrong_format_is_rejected()
{
  encodeBlock(D, 0, block);
  block[4] = 2;
  SpeedConstants out;
  uint32_t ymd;
  uint16_t crc;
  TEST_ASSERT_EQUAL(BlockStatus::bad_format, decodeBlock(block, out, ymd, crc));
}

void test_out_of_range_value_with_good_crc_is_rejected()
{
  SpeedConstants c = D;
  c.kp = 900.0f; // encodeBlock does not check; decode does
  encodeBlock(c, 0, block);
  SpeedConstants out;
  uint32_t ymd;
  uint16_t crc;
  TEST_ASSERT_EQUAL(BlockStatus::out_of_range, decodeBlock(block, out, ymd, crc));
}

void test_set_by_key()
{
  SpeedConstants c = D;
  TEST_ASSERT_TRUE(setByKey(c, "pid.ff_rev_us", 110.0f));
  TEST_ASSERT_EQUAL_FLOAT(110.0f, c.ff_rev_us);
  TEST_ASSERT_FALSE(setByKey(c, "pid.nope", 1.0f));
  TEST_ASSERT_FALSE(setByKey(c, "pid.kp", -1.0f));
  TEST_ASSERT_FALSE(setByKey(c, "pid.kp", 0.0f / 0.0f));
  TEST_ASSERT_FALSE(setByKey(c, "pid.start_rev_max_us", 100.0f)); // below from_us 160
  TEST_ASSERT_EQUAL_FLOAT(320.0f, c.start_rev_max_us);
  TEST_ASSERT_EQUAL_FLOAT(20.0f, c.kp);
}

void test_every_field_has_a_distinct_pid_key()
{
  for (size_t i = 0; i < car_constants::count; i++)
  {
    TEST_ASSERT_EQUAL_INT(0, strncmp(car_constants::fields[i].key, "pid.", 4));
    for (size_t j = i + 1; j < car_constants::count; j++)
      TEST_ASSERT_NOT_EQUAL(0, strcmp(car_constants::fields[i].key, car_constants::fields[j].key));
  }
}

namespace
{
  char out[2048];
  uint32_t ymd_out;

  ConstantsSession fresh()
  {
    uint8_t erased[car_constants::block_bytes];
    memset(erased, 0xFF, sizeof(erased));
    return openSession(erased, D);
  }

  ConstantsAction cmd(ConstantsSession &s, const char *line, bool stopped = true)
  {
    return constantsCommand(s, D, line, stopped, ymd_out, out, sizeof(out));
  }

  bool replyHas(const char *text) { return strstr(out, text) != nullptr; }
}

void test_session_from_empty_eeprom_uses_defaults()
{
  ConstantsSession s = fresh();
  TEST_ASSERT_EQUAL(BlockStatus::empty, s.eeprom);
  TEST_ASSERT_TRUE(sameConstants(D, s.live));
  TEST_ASSERT_EQUAL_STRING("builtin", constantsSource(s, D));
}

void test_session_from_saved_block_uses_it()
{
  SpeedConstants c = D;
  c.kp = 25.0f;
  encodeBlock(c, 20261106, block);
  ConstantsSession s = openSession(block, D);
  TEST_ASSERT_EQUAL(BlockStatus::ok, s.eeprom);
  TEST_ASSERT_EQUAL_FLOAT(25.0f, s.live.kp);
  TEST_ASSERT_EQUAL_STRING("eeprom", constantsSource(s, D));
}

void test_get_lists_cal_and_pid_keys()
{
  ConstantsSession s = fresh();
  TEST_ASSERT_EQUAL(ConstantsAction::none, cmd(s, "cal get"));
  TEST_ASSERT_TRUE(replyHas("cal.source=builtin\n"));
  TEST_ASSERT_TRUE(replyHas("cal.eeprom=empty\n"));
  TEST_ASSERT_TRUE(replyHas("cal.crc=none\n"));
  TEST_ASSERT_TRUE(replyHas("cal.written=none\n"));
  TEST_ASSERT_TRUE(replyHas("pid.ff_rev_us=94.000\n"));
  TEST_ASSERT_TRUE(replyHas("pid.boost_error_mps=0.250\n"));
  TEST_ASSERT_TRUE(replyHas("ok get\n"));
}

void test_set_applies_all_or_nothing()
{
  ConstantsSession s = fresh();
  TEST_ASSERT_EQUAL(ConstantsAction::apply, cmd(s, "cal set pid.ff_rev_us=110 pid.kv_ff_rev=28"));
  TEST_ASSERT_EQUAL_STRING("ok set\n", out);
  TEST_ASSERT_EQUAL_FLOAT(110.0f, s.live.ff_rev_us);
  TEST_ASSERT_EQUAL_FLOAT(28.0f, s.live.kv_ff_rev);
  TEST_ASSERT_EQUAL_STRING("builtin_modified", constantsSource(s, D));

  TEST_ASSERT_EQUAL(ConstantsAction::none, cmd(s, "cal set pid.kp=30 pid.ki=9999"));
  TEST_ASSERT_EQUAL_STRING("err bad value pid.ki\n", out);
  TEST_ASSERT_EQUAL_FLOAT(20.0f, s.live.kp); // first pair not applied either
  TEST_ASSERT_EQUAL(ConstantsAction::none, cmd(s, "cal set pid.kp="));
  TEST_ASSERT_EQUAL(ConstantsAction::none, cmd(s, "cal set pid.kp=3x"));
  TEST_ASSERT_EQUAL(ConstantsAction::none, cmd(s, "cal set"));
}

void test_changes_refused_while_moving()
{
  ConstantsSession s = fresh();
  TEST_ASSERT_EQUAL(ConstantsAction::none, cmd(s, "cal set pid.kp=30", false));
  TEST_ASSERT_EQUAL_STRING("err car moving\n", out);
  TEST_ASSERT_EQUAL(ConstantsAction::none, cmd(s, "cal save 20261106", false));
  TEST_ASSERT_EQUAL(ConstantsAction::none, cmd(s, "cal erase", false));
  TEST_ASSERT_EQUAL(ConstantsAction::none, cmd(s, "cal defaults", false));
  TEST_ASSERT_EQUAL(ConstantsAction::none, cmd(s, "cal get", false)); // reading is fine
  TEST_ASSERT_TRUE(replyHas("ok get"));
}

void test_save_checks_the_date_and_finishes_on_readback()
{
  ConstantsSession s = fresh();
  cmd(s, "cal set pid.ff_rev_us=110");
  TEST_ASSERT_EQUAL(ConstantsAction::none, cmd(s, "cal save"));
  TEST_ASSERT_EQUAL(ConstantsAction::none, cmd(s, "cal save 20261306"));
  TEST_ASSERT_EQUAL(ConstantsAction::save, cmd(s, "cal save 20261106"));
  TEST_ASSERT_EQUAL_UINT32(20261106, ymd_out);

  encodeBlock(s.live, ymd_out, block); // what the caller writes and reads back
  TEST_ASSERT_TRUE(finishSave(s, block, out, sizeof(out)) > 0);
  TEST_ASSERT_TRUE(replyHas("ok save\n"));
  TEST_ASSERT_EQUAL_STRING("eeprom", constantsSource(s, D));
  TEST_ASSERT_EQUAL_UINT32(20261106, s.written_ymd);

  cmd(s, "cal get");
  TEST_ASSERT_TRUE(replyHas("cal.written=2026-11-06\n"));
}

void test_save_reports_a_bad_readback()
{
  ConstantsSession s = fresh();
  cmd(s, "cal save 20261106");
  memset(block, 0xFF, sizeof(block)); // the write did not land
  finishSave(s, block, out, sizeof(out));
  TEST_ASSERT_EQUAL_STRING("err readback empty\n", out);
  TEST_ASSERT_EQUAL_STRING("builtin", constantsSource(s, D));
}

void test_revert_and_defaults()
{
  SpeedConstants c = D;
  c.kp = 25.0f;
  encodeBlock(c, 20261106, block);
  ConstantsSession s = openSession(block, D);
  cmd(s, "cal set pid.kp=30");
  TEST_ASSERT_EQUAL_STRING("eeprom_modified", constantsSource(s, D));
  TEST_ASSERT_EQUAL(ConstantsAction::apply, cmd(s, "cal revert"));
  TEST_ASSERT_EQUAL_FLOAT(25.0f, s.live.kp);
  TEST_ASSERT_EQUAL(ConstantsAction::apply, cmd(s, "cal defaults"));
  TEST_ASSERT_EQUAL_FLOAT(20.0f, s.live.kp);
  TEST_ASSERT_EQUAL_STRING("eeprom_modified", constantsSource(s, D));
}

void test_erase_finishes_on_an_empty_readback()
{
  encodeBlock(D, 20261106, block);
  ConstantsSession s = openSession(block, D);
  TEST_ASSERT_EQUAL(ConstantsAction::erase, cmd(s, "cal erase"));
  memset(block, 0xFF, sizeof(block));
  finishErase(s, block, out, sizeof(out));
  TEST_ASSERT_EQUAL_STRING("ok erase\n", out);
  TEST_ASSERT_EQUAL(BlockStatus::empty, s.eeprom);
}

void test_unknown_lines_are_errors()
{
  ConstantsSession s = fresh();
  TEST_ASSERT_EQUAL(ConstantsAction::none, cmd(s, "hello"));
  TEST_ASSERT_EQUAL_STRING("err not a cal command\n", out);
  TEST_ASSERT_EQUAL(ConstantsAction::none, cmd(s, "cal fly"));
  TEST_ASSERT_EQUAL_STRING("err unknown command\n", out);
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_defaults_match_the_fleet_config);
  RUN_TEST(test_block_round_trip);
  RUN_TEST(test_erased_eeprom_is_empty);
  RUN_TEST(test_any_flipped_byte_is_rejected);
  RUN_TEST(test_wrong_format_is_rejected);
  RUN_TEST(test_out_of_range_value_with_good_crc_is_rejected);
  RUN_TEST(test_set_by_key);
  RUN_TEST(test_every_field_has_a_distinct_pid_key);
  RUN_TEST(test_session_from_empty_eeprom_uses_defaults);
  RUN_TEST(test_session_from_saved_block_uses_it);
  RUN_TEST(test_get_lists_cal_and_pid_keys);
  RUN_TEST(test_set_applies_all_or_nothing);
  RUN_TEST(test_changes_refused_while_moving);
  RUN_TEST(test_save_checks_the_date_and_finishes_on_readback);
  RUN_TEST(test_save_reports_a_bad_readback);
  RUN_TEST(test_revert_and_defaults);
  RUN_TEST(test_erase_finishes_on_an_empty_readback);
  RUN_TEST(test_unknown_lines_are_errors);
  return UNITY_END();
}
