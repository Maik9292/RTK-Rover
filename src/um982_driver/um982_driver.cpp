#include "um982_driver.h"
#include "../../user_config.h"
#include "../settings_manager/settings_manager.h"
#include "../storage/nmea_logger.h"
#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include <time.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

static RoverUIData_t     gnss_state = {};
static Um982PortStats_t  port_stats = {};
volatile GpsTimeSync_t   gps_time_sync = {};
static char              com1_line_buf[512];
static uint16_t          com1_line_pos      = 0;
static bool              com1_line_overflow = false;
static SemaphoreHandle_t s_com1_tx_mutex    = NULL;
static volatile bool     s_com1_tx_boot_complete = false;

static bool              rtc_synced_from_gnss    = false;
static volatile bool     rtc_sync_pending        = false;
static volatile time_t   s_pending_rtc_epoch     = 0;
static volatile uint32_t s_pending_rtc_sync_ms   = 0;
static uint32_t          s_last_rtc_chip_sync_ms  = 0;
static uint32_t          s_last_sysclock_sync_ms  = 0;
static uint32_t          s_last_com1_poll_ms      = 0;
static bool              s_com1_rx_backlog_active = false;

static volatile bool     version_ack_received    = false;
static volatile bool     saveconfig_ack_received = false;
static volatile bool     mode_query_ack_received = false;
static volatile bool     config_query_ack_received  = false;
static volatile bool     loglist_query_ack_received = false;

static char              s_cfg_pvtalg[16]   = "--";
static char              s_cfg_hdg_mode[16] = "--";
static char              s_cfg_hdg_len[24]  = "--";
static char              s_cfg_hdg_dyn[16]  = "--";
static char              s_cfg_com1[16]     = "115200";
static uint8_t           s_loglist_com1_cnt = 0;
static uint8_t           s_loglist_com2_cnt = 0;
static uint8_t           s_loglist_mask     = 0; // Bitmask: 0=GGA,1=RMC,2=GST,3=GSV,4=HDG,5=GGAH,6=BASE

static uint8_t  gsv_sats_gp = 0;
static uint8_t  gsv_sats_gl = 0;
static uint8_t  gsv_sats_ga = 0;
static uint8_t  gsv_sats_gb = 0;
static uint8_t  gsv_sats_gq = 0;
static uint32_t gsv_last_gp_ms = 0;
static uint32_t gsv_last_gl_ms = 0;
static uint32_t gsv_last_ga_ms = 0;
static uint32_t gsv_last_gb_ms = 0;
static uint32_t gsv_last_gq_ms = 0;
static uint32_t s_last_offline_probe_ms = 0;
static bool     s_was_com1_online       = false;

static double parse_nmea_deg_min(const char *str, const char *hemisphere)
{
    if (!str || str[0] == '\0') return 0.0;
    double raw = strtod(str, NULL);
    int deg = (int)(raw / 100.0);
    double min = raw - (deg * 100.0);
    double dec = (double)deg + (min / 60.0);
    if (hemisphere && (hemisphere[0] == 'S' || hemisphere[0] == 'W')) {
        dec = -dec;
    }
    return dec;
}

static int split_csv_fields(char *line, char **fields, int max_fields)
{
    if (!line || max_fields == 0) return 0;
    int count = 0;
    char *p = line;
    fields[count++] = p;
    while (*p && count < max_fields) {
        if (*p == ',' || *p == '*') {
            char sep = *p;
            *p = '\0';
            fields[count++] = p + 1;
            if (sep == '*') break;
        }
        p++;
    }
    return count;
}

static void update_gps_utc_sync_from_nmea_field(const char *time_field, uint32_t now_ms)
{
    if (!time_field || strlen(time_field) < 6) return;
    for (int i = 0; i < 6; i++) {
        if (time_field[i] < '0' || time_field[i] > '9') return;
    }

    uint32_t hh = (uint32_t)((time_field[0] - '0') * 10 + (time_field[1] - '0'));
    uint32_t mm = (uint32_t)((time_field[2] - '0') * 10 + (time_field[3] - '0'));
    uint32_t ss = (uint32_t)((time_field[4] - '0') * 10 + (time_field[5] - '0'));
    if (hh >= 24U || mm >= 60U || ss >= 60U) return;

    uint32_t frac_ms = 0;
    if (time_field[6] == '.' && time_field[7] != '\0') {
        double frac = atof(time_field + 6);
        if (frac >= 0.0 && frac < 1.0) {
            frac_ms = (uint32_t)(frac * 1000.0 + 0.5);
            if (frac_ms >= 1000U) frac_ms = 999U;
        }
    }

    // Anchor last_sync_millis to the exact integer UTC second boundary (.000),
    // compensating for fractional epoch timestamp and ~15 ms NMEA serial transit latency.
    gps_time_sync.last_utc_sec     = (hh * 3600U) + (mm * 60U) + ss;
    gps_time_sync.last_sync_millis = now_ms - frac_ms - 15U;
    gps_time_sync.time_synced      = true;
}

static void handle_gga_sentence(char *line)
{
    char *f[16] = {0};
    int n = split_csv_fields(line, f, 16);
    if (n < 14) return;

    uint32_t now_ms = millis();
    gps_time_sync.last_gga_ms = now_ms;

    int qual = f[6] ? atoi(f[6]) : 0;
    if (qual > 0 && f[1] && f[1][0] && !s_com1_rx_backlog_active && Serial1.available() < 180) {
        update_gps_utc_sync_from_nmea_field(f[1], now_ms);
    }
    gnss_state.fix_type = (GnssFixType_t)qual;
    gnss_state.num_sats = f[7] ? (uint8_t)atoi(f[7]) : 0;
    if (f[8] && f[8][0]) gnss_state.hdop = (float)atof(f[8]);

    if (qual > 0 && f[2] && f[2][0] && f[4] && f[4][0]) {
        gnss_state.raw_lat   = parse_nmea_deg_min(f[2], f[3]);
        gnss_state.raw_lon   = parse_nmea_deg_min(f[4], f[5]);
        gnss_state.raw_alt_m = strtod(f[9], NULL);
    }

    if (f[13] && f[13][0]) {
        gnss_state.diff_age_s = (float)atof(f[13]);
    } else {
        gnss_state.diff_age_s = 0.0f;
    }
    if (n > 14 && f[14] && f[14][0]) {
        gnss_state.base_station_id = (uint16_t)atoi(f[14]);
    }

    if (gnss_state.h_acc_m <= 0.0001f || qual != GNSS_FIX_RTK_FIXED) {
        if (qual == GNSS_FIX_RTK_FIXED)      gnss_state.h_acc_m = 0.014f;
        else if (qual == GNSS_FIX_RTK_FLOAT) gnss_state.h_acc_m = 0.180f;
        else if (qual == GNSS_FIX_DGPS)      gnss_state.h_acc_m = 0.650f;
        else if (qual == GNSS_FIX_SINGLE)    gnss_state.h_acc_m = 1.500f;
        else                                 gnss_state.h_acc_m = 99.9f;
    }
}

static void handle_ggah_sentence(char *line)
{
    char *f[16] = {0};
    int n = split_csv_fields(line, f, 16);
    if (n < 9) return;

    int qual = f[6] ? atoi(f[6]) : 0;
    // Note: Only record single/DGPS standalone qual; moving-baseline RTK status is governed by UNIHEADINGA
    if (gnss_state.slave_fix_type == GNSS_FIX_NONE) {
        if (qual == 1)      gnss_state.slave_fix_type = GNSS_FIX_SINGLE;
        else if (qual == 2) gnss_state.slave_fix_type = GNSS_FIX_DGPS;
    }
    gnss_state.slave_num_sats = f[7] ? (uint8_t)atoi(f[7]) : 0;
    if (f[8] && f[8][0]) gnss_state.slave_hdop  = (float)atof(f[8]);
    if (f[9] && f[9][0]) gnss_state.slave_alt_m = (float)atof(f[9]);

    if (f[2] && f[2][0] && f[4] && f[4][0]) {
        gnss_state.slave_lat = parse_nmea_deg_min(f[2], f[3]);
        gnss_state.slave_lon = parse_nmea_deg_min(f[4], f[5]);
    }
}

static time_t utc_ymdhms_to_epoch(int year, int month, int day, int hour, int minute, int second)
{
    int y = year - (month <= 2 ? 1 : 0);
    int era = (y >= 0 ? y : y - 399) / 400;
    int yoe = y - era * 400;
    int doy = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
    int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    int64_t days = (int64_t)era * 146097LL + (int64_t)doe - 719468LL;
    return (time_t)(days * 86400LL + (int64_t)hour * 3600LL + (int64_t)minute * 60LL + (int64_t)second);
}

static void handle_rmc_sentence(char *line)
{
    char *f[14] = {0};
    int n = split_csv_fields(line, f, 14);
    if (n < 10) return;

    uint32_t now_ms = millis();
    gps_time_sync.last_rmc_ms = now_ms;

    // Field 2: Status ('A'=Active, 'V'=Void, or 'D'=Differential)
    bool pos_active  = (f[2] && (f[2][0] == 'A' || f[2][0] == 'D'));
    bool clock_fresh = (!s_com1_rx_backlog_active && Serial1.available() < 180);
    if (pos_active && f[1] && f[1][0] && clock_fresh) {
        update_gps_utc_sync_from_nmea_field(f[1], now_ms);
    }

    // Field 7: Speed over ground in knots (1 knot = 0.514444 m/s)
    if (f[7] && f[7][0]) {
        gnss_state.speed_m_s = (float)atof(f[7]) * 0.514444f;
    }

    // Field 8: Track angle / Course Over Ground (COG) in degrees True
    if (pos_active && f[8] && f[8][0]) {
        gnss_state.cog_deg = (float)atof(f[8]);

        // Kinematic Motion Fallback:
        // If dual-antenna baseline heading is invalid or currently steering via COG/HOLD:
        if (!gnss_state.heading_valid || gnss_state.heading_source == HDG_SRC_COG || gnss_state.heading_source == HDG_SRC_HOLD) {
            if (gnss_state.speed_m_s >= 0.40f) {
                // Moving forward at normal walking pace (> 0.40 m/s / ~0.8 knots / ~1.4 km/h)
                gnss_state.true_heading_deg = gnss_state.cog_deg;
                gnss_state.heading_valid    = true;
                gnss_state.heading_is_fixed = false;
                gnss_state.heading_source   = HDG_SRC_COG;
            } else if (gnss_state.heading_source == HDG_SRC_COG && gnss_state.speed_m_s < 0.20f) {
                // Surveyor has stopped walking; hold the last confirmed heading
                gnss_state.heading_source = HDG_SRC_HOLD;
            }
        }
    }

    if (pos_active && clock_fresh && strlen(f[1]) >= 6 && strlen(f[9]) == 6) {
        bool digits_ok = true;
        for (int i = 0; i < 6; i++) {
            if (f[1][i] < '0' || f[1][i] > '9' || f[9][i] < '0' || f[9][i] > '9') {
                digits_ok = false;
                break;
            }
        }
        if (digits_ok) {
            int hh = (f[1][0] - '0') * 10 + (f[1][1] - '0');
            int mm = (f[1][2] - '0') * 10 + (f[1][3] - '0');
            int ss = (f[1][4] - '0') * 10 + (f[1][5] - '0');

            int dd = (f[9][0] - '0') * 10 + (f[9][1] - '0');
            int mo = (f[9][2] - '0') * 10 + (f[9][3] - '0');
            int yy = 2000 + (f[9][4] - '0') * 10 + (f[9][5] - '0');

            if (yy >= 2025 && mo >= 1 && mo <= 12 && dd >= 1 && dd <= 31 &&
                hh >= 0 && hh <= 23 && mm >= 0 && mm <= 59 && ss >= 0 && ss <= 59) {
                time_t rmc_epoch = utc_ymdhms_to_epoch(yy, mo, dd, hh, mm, ss);
                uint32_t cur_ms = millis();
                uint32_t sync_ms = gps_time_sync.last_sync_millis;
                uint32_t elapsed_ms = cur_ms - sync_ms;
                time_t target_sec = rmc_epoch + (time_t)(elapsed_ms / 1000U);
                suseconds_t target_usec = (suseconds_t)((elapsed_ms % 1000U) * 1000U);

                struct timeval cur_tv;
                gettimeofday(&cur_tv, NULL);
                int64_t diff_us = ((int64_t)target_sec - (int64_t)cur_tv.tv_sec) * 1000000LL +
                                  ((int64_t)target_usec - (int64_t)cur_tv.tv_usec);

                if (llabs(diff_us) >= 5000LL || s_last_sysclock_sync_ms == 0U || (cur_ms - s_last_sysclock_sync_ms >= 60000U)) {
                    struct timeval tv = { .tv_sec = target_sec, .tv_usec = target_usec };
                    settimeofday(&tv, NULL);
                    s_last_sysclock_sync_ms = (cur_ms == 0U) ? 1U : cur_ms;
                }

                if (!rtc_synced_from_gnss || llabs(diff_us) >= 2000000LL || (cur_ms - s_last_rtc_chip_sync_ms >= 900000U)) {
                    s_pending_rtc_epoch   = rmc_epoch;
                    s_pending_rtc_sync_ms = sync_ms;
                    __sync_synchronize();
                    rtc_synced_from_gnss    = true;
                    s_last_rtc_chip_sync_ms = (cur_ms == 0U) ? 1U : cur_ms;
                    rtc_sync_pending        = true;
                }
            }
        }
    }
}

static void update_gsv_max_band(uint8_t *slot, uint32_t *last_ms, uint8_t sv_in_view, uint32_t now)
{
    if (sv_in_view >= *slot || (now - *last_ms > 1200)) {
        *slot = sv_in_view;
        *last_ms = now;
    }
}

static void handle_gsv_sentence(char *line)
{
    if (strlen(line) < 7) return;
    char talker1 = line[1];
    char talker2 = line[2];
    char *f[6] = {0};
    int n = split_csv_fields(line, f, 6);
    if (n < 4 || !f[3] || !f[3][0]) return;

    uint32_t now = millis();
    uint8_t sv_in_view = (uint8_t)atoi(f[3]);
    if (talker1 == 'G' && talker2 == 'P')      update_gsv_max_band(&gsv_sats_gp, &gsv_last_gp_ms, sv_in_view, now);
    else if (talker1 == 'G' && talker2 == 'L') update_gsv_max_band(&gsv_sats_gl, &gsv_last_gl_ms, sv_in_view, now);
    else if (talker1 == 'G' && talker2 == 'A') update_gsv_max_band(&gsv_sats_ga, &gsv_last_ga_ms, sv_in_view, now);
    else if ((talker1 == 'G' && talker2 == 'B') || (talker1 == 'B' && talker2 == 'D')) update_gsv_max_band(&gsv_sats_gb, &gsv_last_gb_ms, sv_in_view, now);
    else if (talker1 == 'G' && talker2 == 'Q') update_gsv_max_band(&gsv_sats_gq, &gsv_last_gq_ms, sv_in_view, now);

    uint16_t total_view = (uint16_t)gsv_sats_gp + gsv_sats_gl + gsv_sats_ga + gsv_sats_gb + gsv_sats_gq;
    if (total_view > 255) total_view = 255;
    gnss_state.sats_in_view = (uint8_t)total_view;
}

static void handle_gst_sentence(char *line)
{
    char *f[10] = {0};
    int n = split_csv_fields(line, f, 10);
    if (n >= 9 && f[6] && f[6][0] && f[7] && f[7][0]) {
        float lat_e = (float)atof(f[6]);
        float lon_e = (float)atof(f[7]);
        float alt_e = (float)atof(f[8]);
        float h_err = sqrtf(lat_e * lat_e + lon_e * lon_e);
        if (h_err > 0.001f) gnss_state.h_acc_m = h_err;
        if (alt_e > 0.001f) gnss_state.v_acc_m = alt_e;
    }
}

static void handle_hdt_sentence(char *line)
{
    char *f[4] = {0};
    int n = split_csv_fields(line, f, 4);
    if (n >= 2 && f[1] && f[1][0]) {
        gnss_state.true_heading_deg = (float)atof(f[1]);
        gnss_state.heading_valid    = true;
        port_stats.heading_sentences_ok++;
    }
}

static void handle_ths_sentence(char *line)
{
    char *f[4] = {0};
    int n = split_csv_fields(line, f, 4);
    if (n >= 3 && f[1] && f[1][0] && f[2]) {
        // Mode indicator: 'A'=Autonomous, 'D'=Differential, 'E'=Estimated, 'M'=Manual
        if (f[2][0] == 'A' || f[2][0] == 'D' || f[2][0] == 'E' || f[2][0] == 'M') {
            gnss_state.true_heading_deg = (float)atof(f[1]);
            gnss_state.heading_valid    = true;
            port_stats.heading_sentences_ok++;
        }
    }
}

static void handle_uniheadinga_sentence(char *line)
{
    char *semi = strchr(line, ';');
    if (!semi) return;
    char *f[12] = {0};
    int n = split_csv_fields(semi + 1, f, 12);
    if (n >= 5 && f[0] && f[1] && f[2] && f[3] && f[4]) {
        float bl_m = (float)atof(f[2]);
        gnss_state.baseline_length_m = bl_m;

        if (strstr(f[0], "SOL_COMPUTED") != NULL && strstr(f[1], "NONE") == NULL && strstr(f[1], "INSUFFICIENT") == NULL) {
            float expected_bl = (current_settings.antenna_spacing_m > 0.05f) ? current_settings.antenna_spacing_m : 0.555f;
            // Physical sanity gate: Dual-antenna baseline must be within +/- 15cm of expected beam spacing
            bool bl_physically_sane = (fabsf(bl_m - expected_bl) <= 0.150f);
            bool is_int = (strstr(f[1], "INT") != NULL);

            if (is_int && bl_physically_sane) {
                gnss_state.true_heading_deg   = (float)atof(f[3]);
                gnss_state.baseline_pitch_deg = (float)atof(f[4]);
                gnss_state.heading_valid      = true;
                gnss_state.heading_is_fixed   = true;
                gnss_state.heading_source     = HDG_SRC_DUAL_FIX;
                gnss_state.slave_fix_type     = GNSS_FIX_RTK_FIXED;
                gps_time_sync.last_uniheadinga_ms = millis();
                port_stats.heading_sentences_ok++;
            } else if (bl_physically_sane) {
                // Float with sane physical baseline
                gnss_state.true_heading_deg   = (float)atof(f[3]);
                gnss_state.baseline_pitch_deg = (float)atof(f[4]);
                gnss_state.heading_valid      = true;
                gnss_state.heading_is_fixed   = false;
                gnss_state.heading_source     = HDG_SRC_DUAL_FLT;
                gnss_state.slave_fix_type     = GNSS_FIX_RTK_FLOAT;
                gps_time_sync.last_uniheadinga_ms = millis();
                port_stats.heading_sentences_ok++;
            } else {
                // Wild baseline vector (e.g. 4.88m in Float) - reject dual-antenna heading!
                gnss_state.heading_is_fixed = false;
                gnss_state.slave_fix_type   = GNSS_FIX_NONE;
                if (gnss_state.heading_source == HDG_SRC_DUAL_FIX || gnss_state.heading_source == HDG_SRC_DUAL_FLT) {
                    gnss_state.heading_valid  = false;
                    gnss_state.heading_source = HDG_SRC_NONE;
                }
            }
        } else {
            gnss_state.heading_is_fixed = false;
            gnss_state.slave_fix_type   = GNSS_FIX_NONE;
            if (gnss_state.heading_source == HDG_SRC_DUAL_FIX || gnss_state.heading_source == HDG_SRC_DUAL_FLT) {
                gnss_state.heading_valid  = false;
                gnss_state.heading_source = HDG_SRC_NONE;
            }
        }
    }
}

static void ecef_to_wgs84(double x, double y, double z, double *lat_deg, double *lon_deg, double *alt_m)
{
    const double a = 6378137.0;
    const double f = 1.0 / 298.257223563;
    const double b = a * (1.0 - f);
    const double e2 = (a * a - b * b) / (a * a);
    const double ep2 = (a * a - b * b) / (b * b);

    double p = sqrt(x * x + y * y);
    if (p < 1e-6) {
        *lat_deg = (z > 0) ? 90.0 : -90.0;
        *lon_deg = 0.0;
        *alt_m = fabs(z) - b;
        return;
    }

    double th = atan2(a * z, b * p);
    double sin_th = sin(th);
    double cos_th = cos(th);

    double phi = atan2(z + ep2 * b * sin_th * sin_th * sin_th,
                       p - e2 * a * cos_th * cos_th * cos_th);
    double lam = atan2(y, x);

    double sin_phi = sin(phi);
    double N = a / sqrt(1.0 - e2 * sin_phi * sin_phi);

    *lat_deg = phi * (180.0 / M_PI);
    *lon_deg = lam * (180.0 / M_PI);
    *alt_m   = (p / cos(phi)) - N;
}

static void handle_baseinfoa_sentence(char *line)
{
    char *semi = strchr(line, ';');
    if (!semi) return;

    char *f[8] = {0};
    int n = split_csv_fields(semi + 1, f, 8);
    if (n < 4) return;

    double x = (n > 1 && f[1]) ? strtod(f[1], NULL) : 0.0;
    double y = (n > 2 && f[2]) ? strtod(f[2], NULL) : 0.0;
    double z = (n > 3 && f[3]) ? strtod(f[3], NULL) : 0.0;

    if (fabs(x) > 1000.0 && fabs(y) > 1000.0) {
        gnss_state.base_ecef_x = x;
        gnss_state.base_ecef_y = y;
        gnss_state.base_ecef_z = z;

        ecef_to_wgs84(x, y, z, &gnss_state.base_lat, &gnss_state.base_lon, &gnss_state.base_alt_m);
        gnss_state.base_coords_valid = true;

        if (n > 4 && f[4] && f[4][0]) {
            const char *sid_str = f[4];
            if (sid_str[0] == '"') sid_str++;
            gnss_state.base_station_id = (uint16_t)atoi(sid_str);
        }

        Serial.printf("[UM982 BASE] ID:%u Valid:1 Pos:%.8f, %.8f, %.3fm (ECEF: %.2f, %.2f, %.2f)\n",
                      gnss_state.base_station_id,
                      gnss_state.base_lat, gnss_state.base_lon, gnss_state.base_alt_m,
                      x, y, z);
    }
}

static bool verify_nmea_xor_checksum(const char *line)
{
    if (!line || line[0] != '$') return true;
    const char *star = strrchr(line, '*');
    if (!star || !star[1] || !star[2]) return true; // Allow if unchecksummed
    uint8_t calc = 0;
    for (const char *p = line + 1; p < star; p++) {
        calc ^= (uint8_t)(*p);
    }
    uint8_t expected = (uint8_t)strtoul(star + 1, NULL, 16);
    return (calc == expected);
}

static void strip_checksum_in_place(char *str)
{
    if (!str) return;
    char *star = strrchr(str, '*');
    if (star) *star = '\0';
}

static void rebuild_known_state_summary(void)
{
    snprintf(port_stats.um982_config_summary, sizeof(port_stats.um982_config_summary),
             "PVT:%s | HDG:%s %s %s | C1:%s",
             s_cfg_pvtalg, s_cfg_hdg_mode, s_cfg_hdg_len, s_cfg_hdg_dyn, s_cfg_com1);

    snprintf(port_stats.um982_loglist, sizeof(port_stats.um982_loglist),
             "COM1(%u):%s%s%s%s%s%s%s | COM2:%u",
             (unsigned)s_loglist_com1_cnt,
             (s_loglist_mask & (1U << 0)) ? " GGA"  : "",
             (s_loglist_mask & (1U << 1)) ? " RMC"  : "",
             (s_loglist_mask & (1U << 2)) ? " GST"  : "",
             (s_loglist_mask & (1U << 3)) ? " GSV"  : "",
             (s_loglist_mask & (1U << 4)) ? " HDG"  : "",
             (s_loglist_mask & (1U << 5)) ? " GGAH" : "",
             (s_loglist_mask & (1U << 6)) ? " BASE" : "",
             (unsigned)s_loglist_com2_cnt);

    bool survey_ok = (strstr(port_stats.um982_mode, "SURVEY") != NULL);
    bool logs_ok   = (s_loglist_mask == 0x7FU && s_loglist_com2_cnt == 0);
    port_stats.known_state_ok = (survey_ok && logs_ok);
}

static void parse_config_response_line(const char *line)
{
    if (strstr(line, "CONFIG PVTALG ") != NULL) {
        const char *p = strstr(line, "CONFIG PVTALG ") + 14;
        strncpy(s_cfg_pvtalg, p, sizeof(s_cfg_pvtalg) - 1);
        s_cfg_pvtalg[sizeof(s_cfg_pvtalg) - 1] = '\0';
        strip_checksum_in_place(s_cfg_pvtalg);
    } else if (strstr(line, "CONFIG HEADING FIXLENGTH") != NULL) {
        strncpy(s_cfg_hdg_mode, "FIXLEN", sizeof(s_cfg_hdg_mode) - 1);
    } else if (strstr(line, "CONFIG HEADING VARLENGTH") != NULL) {
        strncpy(s_cfg_hdg_mode, "VARLEN", sizeof(s_cfg_hdg_mode) - 1);
    } else if (strstr(line, "CONFIG HEADING LENGTH ") != NULL) {
        const char *p = strstr(line, "CONFIG HEADING LENGTH ") + 22;
        strncpy(s_cfg_hdg_len, p, sizeof(s_cfg_hdg_len) - 1);
        s_cfg_hdg_len[sizeof(s_cfg_hdg_len) - 1] = '\0';
        strip_checksum_in_place(s_cfg_hdg_len);
    } else if (strstr(line, "CONFIG HEADING LOWDYNAMIC") != NULL) {
        strncpy(s_cfg_hdg_dyn, "LOWDYN", sizeof(s_cfg_hdg_dyn) - 1);
    } else if (strstr(line, "CONFIG COM1 ") != NULL) {
        const char *p = strstr(line, "CONFIG COM1 ") + 12;
        strncpy(s_cfg_com1, p, sizeof(s_cfg_com1) - 1);
        s_cfg_com1[sizeof(s_cfg_com1) - 1] = '\0';
        strip_checksum_in_place(s_cfg_com1);
    }
    rebuild_known_state_summary();
}

static void parse_loglist_response_line(const char *line)
{
    // Unicore #LOGLIST format:
    // #LOGLIST,97,GPS,FINE,...;<count>,COM1,GPGGA,ONTIME,0.5,COM1,GPRMC,ONTIME,1,...*XX
    const char *body = strchr(line, ';');
    if (!body) body = line;
    else body++;

    char buf[512];
    strncpy(buf, body, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    strip_checksum_in_place(buf);

    // If this is the header of a #LOGLIST;count,... dump, reset counters before scanning tokens
    if (strncmp(line, "#LOGLIST", 8) == 0) {
        s_loglist_com1_cnt = 0;
        s_loglist_com2_cnt = 0;
        s_loglist_mask     = 0;
    }

    char *tok = strtok(buf, ",");
    while (tok != NULL) {
        if (strcmp(tok, "COM1") == 0) {
            s_loglist_com1_cnt++;
            char *msg = strtok(NULL, ",");
            if (msg) {
                if (strstr(msg, "GGAH") != NULL)           s_loglist_mask |= (1U << 5);
                else if (strstr(msg, "GGA") != NULL)       s_loglist_mask |= (1U << 0);
                else if (strstr(msg, "RMC") != NULL)       s_loglist_mask |= (1U << 1);
                else if (strstr(msg, "GST") != NULL)       s_loglist_mask |= (1U << 2);
                else if (strstr(msg, "GSV") != NULL)       s_loglist_mask |= (1U << 3);
                else if (strstr(msg, "UNIHEADING") != NULL)s_loglist_mask |= (1U << 4);
                else if (strstr(msg, "BASEINFO") != NULL)  s_loglist_mask |= (1U << 6);
            }
        } else if (strcmp(tok, "COM2") == 0 || strcmp(tok, "COM3") == 0) {
            s_loglist_com2_cnt++;
        }
        tok = strtok(NULL, ",");
    }
    rebuild_known_state_summary();
}

static void mark_com1_rx_alive(void)
{
    port_stats.last_com1_rx_ms  = millis();
    port_stats.com1_online      = true;
    port_stats.com1_test_passed = true;
}

static void record_cmd_response_line(const char *line)
{
    mark_com1_rx_alive();
    port_stats.cmd_responses_ok++;
    strncpy(port_stats.last_cmd_ack_line, line, sizeof(port_stats.last_cmd_ack_line) - 1);
    port_stats.last_cmd_ack_line[sizeof(port_stats.last_cmd_ack_line) - 1] = '\0';
}

static void process_com1_line(char *line, uint16_t line_len)
{
    if (!line || line[0] == '\0') return;

    // =====================================================================
    // Stream 3: Unicore Command / Query Responses ($command, #VERSIONA, #MODE, $CONFIG, #LOGLIST)
    // Handled first so command responses NEVER pollute .nmea logs or overwrite last_com1_line!
    // =====================================================================
    if (strncmp(line, "#VERSIONA", 9) == 0) {
        record_cmd_response_line(line);
        char *semi = strchr(line, ';');
        if (semi) {
            strncpy(port_stats.um982_version, semi + 1, sizeof(port_stats.um982_version) - 1);
            port_stats.um982_version[sizeof(port_stats.um982_version) - 1] = '\0';
            strip_checksum_in_place(port_stats.um982_version);
        } else {
            strncpy(port_stats.um982_version, "UM982_ONLINE", sizeof(port_stats.um982_version) - 1);
        }
        version_ack_received = true;
        Serial.printf("[UM982 CMD RX] %s\n", line);
        return;
    }

    if (strncmp(line, "#MODE", 5) == 0 || strncmp(line, "$MODE", 5) == 0) {
        record_cmd_response_line(line);
        const char *semi = strchr(line, ';');
        const char *m_str = semi ? (semi + 1) : (strchr(line, ',') ? strchr(line, ',') + 1 : line);
        if (strncmp(m_str, "MODE ", 5) == 0) m_str += 5;
        strncpy(port_stats.um982_mode, m_str, sizeof(port_stats.um982_mode) - 1);
        port_stats.um982_mode[sizeof(port_stats.um982_mode) - 1] = '\0';
        strip_checksum_in_place(port_stats.um982_mode);
        rebuild_known_state_summary();
        mode_query_ack_received = true;
        Serial.printf("[UM982 CMD RX] %s\n", line);
        return;
    }

    if (strncmp(line, "$CONFIG,", 8) == 0 || strncmp(line, "#CONFIG,", 8) == 0) {
        record_cmd_response_line(line);
        parse_config_response_line(line);
        config_query_ack_received = true;
        Serial.printf("[UM982 CMD RX] %s\n", line);
        return;
    }

    if (strncmp(line, "#LOGLIST", 8) == 0 || strncmp(line, "$LOGLIST", 8) == 0) {
        record_cmd_response_line(line);
        parse_loglist_response_line(line);
        loglist_query_ack_received = true;
        Serial.printf("[UM982 CMD RX] %s\n", line);
        return;
    }

    if (strncmp(line, "$command,", 9) == 0) {
        record_cmd_response_line(line);
        if (strstr(line, "SAVECONFIG") != NULL && strstr(line, "response:") != NULL) {
            saveconfig_ack_received = true;
        }
        Serial.printf("[UM982 CMD RX] %s\n", line);
        return;
    }

    // =====================================================================
    // Stream 1 & 2: Periodic NMEA 0183 & Unicore ASCII Navigation Logs
    // Only valid navigation sentences reach nmea_logger_feed() and last_com1_line.
    // =====================================================================
    if (line[0] == '$' && !verify_nmea_xor_checksum(line)) {
        return;
    }
    if (line[0] == '$' || line[0] == '#') {
        mark_com1_rx_alive();
    }

    nmea_logger_feed((const uint8_t *)line, line_len);
    nmea_logger_feed((const uint8_t *)"\r\n", 2);

    strncpy(port_stats.last_com1_line, line, sizeof(port_stats.last_com1_line) - 1);
    port_stats.last_com1_line[sizeof(port_stats.last_com1_line) - 1] = '\0';

    if (strncmp(line, "$GNGGAH", 7) == 0 || strncmp(line, "$GPGGAH", 7) == 0) {
        port_stats.nmea_sentences_ok++;
        handle_ggah_sentence(line);
    } else if (strncmp(line, "$GNGGA", 6) == 0 || strncmp(line, "$GPGGA", 6) == 0) {
        port_stats.nmea_sentences_ok++;
        handle_gga_sentence(line);
    } else if (strncmp(line, "$GNRMC", 6) == 0 || strncmp(line, "$GPRMC", 6) == 0) {
        port_stats.nmea_sentences_ok++;
        handle_rmc_sentence(line);
    } else if (strncmp(line, "$GNGST", 6) == 0 || strncmp(line, "$GPGST", 6) == 0) {
        port_stats.nmea_sentences_ok++;
        handle_gst_sentence(line);
    } else if (strncmp(line, "$GNHDT", 6) == 0 || strncmp(line, "$GPHDT", 6) == 0) {
        port_stats.nmea_sentences_ok++;
        handle_hdt_sentence(line);
    } else if (strncmp(line, "$GNTHS", 6) == 0 || strncmp(line, "$GPTHS", 6) == 0) {
        port_stats.nmea_sentences_ok++;
        handle_ths_sentence(line);
    } else if (strncmp(line, "$GNGSV", 6) == 0 || strncmp(line, "$GPGSV", 6) == 0) {
        port_stats.nmea_sentences_ok++;
        handle_gsv_sentence(line);
    } else if (strncmp(line, "#UNIHEADINGA", 12) == 0) {
        handle_uniheadinga_sentence(line);
    } else if (strncmp(line, "#BASEINFOA", 10) == 0) {
        handle_baseinfoa_sentence(line);
    }
}

extern "C" void send_um982_com1_command(const char *cmd)
{
    if (!cmd) return;
    size_t len = strlen(cmd);
    if (s_com1_tx_mutex) {
        if (xSemaphoreTake(s_com1_tx_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
            port_stats.com1_tx_mutex_timeouts++;
            return;
        }
    }
    if (Serial) Serial.printf("[UM982 COM1 TX] %s\n", cmd);
    Serial1.write((const uint8_t *)cmd, len);
    Serial1.write((const uint8_t *)"\r\n", 2);
    port_stats.com1_tx_bytes += (uint32_t)(len + 2);
    if (s_com1_tx_mutex) xSemaphoreGive(s_com1_tx_mutex);
}

extern "C" void send_um982_com2_test_pulse(void)
{
    // Legacy stub: COM2 is consolidated onto Single-UART COM1 in v0.8.0+
    um982_query_receiver_state();
}

extern "C" void um982_query_receiver_state(void)
{
    Serial.println("\n--- [UM982 Known-State Audit: VERSIONA / MODE / CONFIG / LOGLIST] ---");

    // 1. VERSIONA
    version_ack_received = false;
    send_um982_com1_command("VERSIONA");
    uint32_t t0 = millis();
    while (!version_ack_received && (millis() - t0 < 500)) {
        um982_poll_com1();
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // 2. MODE
    mode_query_ack_received = false;
    send_um982_com1_command("MODE");
    t0 = millis();
    while (!mode_query_ack_received && (millis() - t0 < 500)) {
        um982_poll_com1();
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // 3. CONFIG (returns multiple $CONFIG,... lines)
    config_query_ack_received = false;
    send_um982_com1_command("CONFIG");
    t0 = millis();
    while (millis() - t0 < 600) {
        um982_poll_com1();
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // 4. LOGLIST (verifies all 7 logs on COM1 and 0 logs on COM2/COM3)
    loglist_query_ack_received = false;
    send_um982_com1_command("LOGLIST");
    t0 = millis();
    while (!loglist_query_ack_received && (millis() - t0 < 600)) {
        um982_poll_com1();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    // Drain any trailing lines
    t0 = millis();
    while (millis() - t0 < 100) {
        um982_poll_com1();
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    port_stats.single_uart_audit_passed = port_stats.com1_test_passed;
    Serial.printf("[UM982 AUDIT SUMMARY] State:%s | Ver:%s | Mode:%s | Cfg:[%s] | Logs:[%s]\n",
                  port_stats.known_state_ok ? "KNOWN STATE OK" : "CHECK LOGS/MODE",
                  port_stats.um982_version,
                  port_stats.um982_mode,
                  port_stats.um982_config_summary,
                  port_stats.um982_loglist);
}

extern "C" void um982_configure_rover_defaults(void)
{
    float spacing_m = (current_settings.antenna_spacing_m > 0.05f) ? current_settings.antenna_spacing_m : 0.555f;
    float spacing_cm = spacing_m * 100.0f;
    char exp_len_prefix[16];
    snprintf(exp_len_prefix, sizeof(exp_len_prefix), "%.1f", spacing_cm);

    // Query existing NVM receiver state first to prevent redundant SAVECONFIG flash wear on warm boots
    um982_query_receiver_state();
    if (port_stats.known_state_ok &&
        strstr(port_stats.um982_mode, "SURVEY") != NULL &&
        strstr(s_cfg_hdg_len, exp_len_prefix) != NULL) {
        Serial.println("[UM982] Warm boot: Receiver already in KNOWN STATE OK -> Skipping reconfig & SAVECONFIG.");
        return;
    }

    // Phase 1: Quiet-First Setup (Silence periodic logs on COM1 & COM2 so commands do not collide with NMEA)
    send_um982_com1_command("CONFIG COM1 115200");
    vTaskDelay(pdMS_TO_TICKS(40));

    send_um982_com1_command("UNLOGALL COM1");
    vTaskDelay(pdMS_TO_TICKS(40));

    send_um982_com1_command("UNLOGALL COM2");
    vTaskDelay(pdMS_TO_TICKS(40));

    // Phase 2: Receiver Algorithm, Mode & Fixed-Baseline Heading Configuration (while COM1 is quiet)
    send_um982_com1_command("CONFIG PVTALG MULTI");
    vTaskDelay(pdMS_TO_TICKS(40));

    // Put UM982 into precision surveying mode (overrides factory default UAV drone mode)
    send_um982_com1_command("MODE ROVER SURVEY");
    vTaskDelay(pdMS_TO_TICKS(40));

    send_um982_com1_command("CONFIG HEADING LOWDYNAMIC");
    vTaskDelay(pdMS_TO_TICKS(40));

    send_um982_com1_command("CONFIG HEADING FIXLENGTH");
    vTaskDelay(pdMS_TO_TICKS(40));

    char hdg_cmd[64];
    // Parameter 1: baseline length in centimeters (Table 4-14 in manual: 55.5 cm)
    // Parameter 2: tolerance in centimeters (12 cm tolerance)
    snprintf(hdg_cmd, sizeof(hdg_cmd), "CONFIG HEADING LENGTH %.1f 12", spacing_cm);
    send_um982_com1_command(hdg_cmd);
    vTaskDelay(pdMS_TO_TICKS(40));

    // Phase 3: Enable the 7 Required Periodic Logs strictly on COM1
    send_um982_com1_command("GPGGA COM1 0.5");
    vTaskDelay(pdMS_TO_TICKS(40));
    send_um982_com1_command("GPRMC COM1 1");
    vTaskDelay(pdMS_TO_TICKS(40));
    send_um982_com1_command("GPGST COM1 0.5");
    vTaskDelay(pdMS_TO_TICKS(40));
    send_um982_com1_command("GPGSV COM1 1");
    vTaskDelay(pdMS_TO_TICKS(40));
    // UNIHEADINGA provides heading, pitch, baseline length, and NARROW_INT fix type at 2 Hz
    send_um982_com1_command("UNIHEADINGA COM1 0.5");
    vTaskDelay(pdMS_TO_TICKS(40));
    // GNGGAH provides Slave Antenna (ANT2) fix quality, satellites used, HDOP at 1 Hz
    send_um982_com1_command("GNGGAH COM1 1");
    vTaskDelay(pdMS_TO_TICKS(40));
    // BASEINFOA provides base station ID, health status, and ECEF coordinates at 1 Hz
    send_um982_com1_command("BASEINFOA COM1 1");
    vTaskDelay(pdMS_TO_TICKS(40));

    // Phase 4: Persist to UM982 Non-Volatile Flash
    saveconfig_ack_received = false;
    send_um982_com1_command("SAVECONFIG");
    uint32_t t_save = millis();
    while (!saveconfig_ack_received && (millis() - t_save < 1500)) {
        um982_poll_com1();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    // Phase 5: Execute Full 4-Command Known-State Audit (VERSIONA, MODE, CONFIG, LOGLIST)
    um982_query_receiver_state();
}

extern "C" void um982_run_powerup_selftest(void)
{
    Serial.println("\n--- [UM982 Single-UART (COM1) Power-Up Self-Test] ---");

    // Preserve any warm-boot NMEA/log configuration during self-test so um982_configure_rover_defaults()
    // can audit the live receiver state without unnecessary UNLOGALL/SAVECONFIG flash wear.
    while (Serial1.available()) Serial1.read();

    Serial.println("[UM982 COM1] Testing Full-Duplex UART1 (RX=IO17, TX=IO18 @ 115200) with VERSIONA...");
    version_ack_received = false;
    send_um982_com1_command("VERSIONA");
    uint32_t t0 = millis();
    while (!version_ack_received && (millis() - t0 < 600)) {
        um982_poll_com1();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (port_stats.com1_test_passed) {
        port_stats.single_uart_audit_passed = true;
        Serial.printf("[UM982 COM1 TEST] PASS — RX=%lu B | Version: %s | Last ACK: %s\n",
                      (unsigned long)port_stats.com1_rx_bytes,
                      port_stats.um982_version,
                      port_stats.last_cmd_ack_line);
    } else {
        Serial.println("[UM982 COM1 TEST] FAIL — No response on IO17/18 @ 115200");
    }
}

extern "C" void um982_driver_init(void)
{
    if (s_com1_tx_mutex == NULL) {
        s_com1_tx_mutex = xSemaphoreCreateMutex();
    }
    s_com1_tx_boot_complete = false;

    strncpy(port_stats.um982_version, "Waiting COM1...", sizeof(port_stats.um982_version) - 1);
    port_stats.um982_version[sizeof(port_stats.um982_version) - 1] = '\0';
    strncpy(port_stats.um982_mode, "INIT...", sizeof(port_stats.um982_mode) - 1);
    port_stats.um982_mode[sizeof(port_stats.um982_mode) - 1] = '\0';
    strncpy(port_stats.um982_config_summary, "Querying CONFIG...", sizeof(port_stats.um982_config_summary) - 1);
    port_stats.um982_config_summary[sizeof(port_stats.um982_config_summary) - 1] = '\0';
    strncpy(port_stats.um982_loglist, "Querying LOGLIST...", sizeof(port_stats.um982_loglist) - 1);
    port_stats.um982_loglist[sizeof(port_stats.um982_loglist) - 1] = '\0';
    strncpy(port_stats.last_com1_line, "No COM1 NMEA yet", sizeof(port_stats.last_com1_line) - 1);
    port_stats.last_com1_line[sizeof(port_stats.last_com1_line) - 1] = '\0';
    strncpy(port_stats.last_cmd_ack_line, "No CMD ACK yet", sizeof(port_stats.last_cmd_ack_line) - 1);
    port_stats.last_cmd_ack_line[sizeof(port_stats.last_cmd_ack_line) - 1] = '\0';

    pinMode(ROVER_PIN_UM982_TXD0, INPUT_PULLDOWN);
    pinMode(ROVER_PIN_UM982_RXD0, INPUT_PULLDOWN);
    delay(2);
    Serial.printf("[UM982 LINE PROBE] Single-UART COM1: IO17(ESP_RX)=%d IO18(ESP_TX)=%d | COM2(IO47/48): FREED\n",
                  digitalRead(ROVER_PIN_UM982_TXD0),
                  digitalRead(ROVER_PIN_UM982_RXD0));

    // Expand Serial1 TX buffer to 4096B to absorb full-duplex RTCM3 bursts without blocking
    Serial1.setRxBufferSize(4096);
    Serial1.setTxBufferSize(4096);
    Serial1.begin(ROVER_UM982_COM1_BAUD, SERIAL_8N1, ROVER_PIN_UM982_TXD0, ROVER_PIN_UM982_RXD0);

    um982_run_powerup_selftest();
    um982_configure_rover_defaults();

    // Allow Core 0 to begin forwarding atomic CRC24Q-verified RTCM3 frames onto COM1 (Serial1)
    s_com1_tx_boot_complete = true;
}

extern "C" void um982_forward_rtcm(const uint8_t *data, size_t len)
{
    if (!s_com1_tx_boot_complete || !data || len == 0) return;
    if (s_com1_tx_mutex) {
        if (xSemaphoreTake(s_com1_tx_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
            port_stats.com1_tx_mutex_timeouts++;
            return;
        }
    }
    size_t written = Serial1.write(data, len);
    gps_time_sync.last_rtcm_forward_ms = millis();
    port_stats.rtcm_tx_bytes += (uint32_t)written; // Tracks RTCM3 bytes forwarded to UM982
    port_stats.com1_tx_bytes += (uint32_t)written;
    if (s_com1_tx_mutex) xSemaphoreGive(s_com1_tx_mutex);
}

extern "C" void um982_forward_rtcm_to_com2(const uint8_t *data, size_t len)
{
    um982_forward_rtcm(data, len);
}

extern "C" void um982_poll_com1(void)
{
    uint32_t entry_ms = millis();
    int entry_avail = Serial1.available();
    bool has_backlog = (s_last_com1_poll_ms > 0U && (entry_ms - s_last_com1_poll_ms) > 60U) ||
                       (entry_avail > 256);
    int backlog_bytes_remaining = has_backlog ? (entry_avail + (int)com1_line_pos + 64) : 0;

    while (Serial1.available() > 0) {
        char c = (char)Serial1.read();
        port_stats.com1_rx_bytes++;
        if (backlog_bytes_remaining > 0) {
            backlog_bytes_remaining--;
        }

        if (c == '\r' || c == '\n') {
            if (com1_line_overflow) {
                com1_line_overflow = false;
                com1_line_pos = 0;
            } else if (com1_line_pos > 0) {
                com1_line_buf[com1_line_pos] = '\0';
                // Suppress GNSS wall-clock discipline while draining stale NMEA queued across blocking Core 0 work
                s_com1_rx_backlog_active = (backlog_bytes_remaining > 0);
                process_com1_line(com1_line_buf, com1_line_pos);
                com1_line_pos = 0;
            }
        } else if (!com1_line_overflow) {
            if (com1_line_pos < sizeof(com1_line_buf) - 1) {
                com1_line_buf[com1_line_pos++] = c;
            } else {
                // Line exceeded 511 bytes: discard remainder until next \r/\n so tail never masquerades as a new line
                com1_line_overflow = true;
                com1_line_pos = 0;
            }
        }
    }
    s_com1_rx_backlog_active = false;

    uint32_t now_ms = millis();
    bool is_online = (port_stats.last_com1_rx_ms > 0) &&
                     ((now_ms - port_stats.last_com1_rx_ms) <= 3000U);
    if (is_online && !s_was_com1_online) {
        s_was_com1_online = true;
        if (s_com1_tx_boot_complete && !port_stats.known_state_ok) {
            Serial.println("[UM982] COM1 link came ONLINE after boot -> Running receiver state check/config.");
            um982_configure_rover_defaults();
        }
    } else if (!is_online) {
        if (s_was_com1_online) {
            Serial.println("[UM982] WARNING: COM1 link went OFFLINE (>3.0s silence).");
            s_was_com1_online = false;
        }
        port_stats.com1_online      = false;
        port_stats.com1_test_passed = false;
        if (s_com1_tx_boot_complete && (now_ms - s_last_offline_probe_ms >= 3000U)) {
            s_last_offline_probe_ms = now_ms;
            // Lightweight non-blocking probe so an unconfigured/reset UM982 responds when powered on
            if (!s_com1_tx_mutex || xSemaphoreTake(s_com1_tx_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                Serial1.write((const uint8_t *)"VERSIONA\r\n", 10);
                port_stats.com1_tx_bytes += 10U;
                if (s_com1_tx_mutex) xSemaphoreGive(s_com1_tx_mutex);
            }
        }
    }

    s_last_com1_poll_ms = millis();
}

extern "C" void um982_poll_com2(void)
{
    // No-op in v0.8.0+ Single-UART COM1 architecture (Serial2 disabled, freeing IO47/48 & ~6KB DRAM0)
}

extern "C" void um982_get_latest_state(RoverUIData_t *state_out)
{
    if (!state_out) return;

    // Age-of-data expiration: expire stale GGA, GPS UTC sync, and dual-antenna heading on UART/antenna dropout
    uint32_t now_ms = millis();
    bool com1_alive = (port_stats.last_com1_rx_ms > 0) &&
                      ((now_ms - port_stats.last_com1_rx_ms) <= 3000U);
    port_stats.com1_online      = com1_alive;
    port_stats.com1_test_passed = com1_alive;
    if (!com1_alive) {
        gnss_state.fix_type          = GNSS_FIX_NONE;
        gnss_state.num_sats          = 0;
        gnss_state.sats_in_view      = 0;
        gnss_state.h_acc_m           = 99.9f;
        gnss_state.v_acc_m           = 99.9f;
        gnss_state.slave_fix_type    = GNSS_FIX_NONE;
        gnss_state.slave_num_sats    = 0;
        gnss_state.baseline_length_m = 0.0f;
    }
    if (gps_time_sync.last_gga_ms == 0 || (now_ms - gps_time_sync.last_gga_ms > 2000U)) {
        gnss_state.fix_type = GNSS_FIX_NONE;
        gnss_state.h_acc_m  = 99.9f;
        gnss_state.v_acc_m  = 99.9f;
    }
    if (gps_time_sync.time_synced &&
        (now_ms - gps_time_sync.last_gga_ms > 15000U) &&
        (now_ms - gps_time_sync.last_rmc_ms > 15000U)) {
        gps_time_sync.time_synced = false;
    }
    if ((gnss_state.heading_source == HDG_SRC_DUAL_FIX || gnss_state.heading_source == HDG_SRC_DUAL_FLT) &&
        (gps_time_sync.last_uniheadinga_ms == 0 || (now_ms - gps_time_sync.last_uniheadinga_ms > 3000U))) {
        gnss_state.heading_valid    = false;
        gnss_state.heading_is_fixed = false;
        gnss_state.heading_source   = HDG_SRC_NONE;
        gnss_state.slave_fix_type   = GNSS_FIX_NONE;
    }

    state_out->raw_lat            = gnss_state.raw_lat;
    state_out->raw_lon            = gnss_state.raw_lon;
    state_out->raw_alt_m          = gnss_state.raw_alt_m;
    state_out->fix_type           = gnss_state.fix_type;
    state_out->num_sats           = gnss_state.num_sats;
    state_out->sats_in_view       = gnss_state.sats_in_view;
    state_out->hdop               = gnss_state.hdop;
    state_out->h_acc_m            = gnss_state.h_acc_m;
    state_out->v_acc_m            = gnss_state.v_acc_m;
    state_out->diff_age_s         = gnss_state.diff_age_s;
    state_out->heading_valid      = gnss_state.heading_valid;
    state_out->heading_is_fixed   = gnss_state.heading_is_fixed;
    state_out->heading_source     = gnss_state.heading_source;
    state_out->true_heading_deg   = gnss_state.true_heading_deg;
    state_out->baseline_pitch_deg = gnss_state.baseline_pitch_deg;
    state_out->baseline_length_m  = gnss_state.baseline_length_m;
    state_out->speed_m_s          = gnss_state.speed_m_s;
    state_out->cog_deg            = gnss_state.cog_deg;
    state_out->slave_fix_type     = gnss_state.slave_fix_type;
    state_out->slave_num_sats     = gnss_state.slave_num_sats;
    state_out->slave_hdop         = gnss_state.slave_hdop;
    state_out->slave_alt_m        = gnss_state.slave_alt_m;

    // Slave Antenna (ANT2) Coordinates
    if (gnss_state.slave_lat != 0.0 && gnss_state.slave_lon != 0.0) {
        state_out->slave_lat      = gnss_state.slave_lat;
        state_out->slave_lon      = gnss_state.slave_lon;
    } else if (gnss_state.heading_valid && gnss_state.baseline_length_m > 0.05f && gnss_state.raw_lat != 0.0) {
        // Geodetic projection from ANT1 to ANT2 along verified dual-antenna baseline vector
        double lat_rad = gnss_state.raw_lat * (M_PI / 180.0);
        double hdg_rad = gnss_state.true_heading_deg * (M_PI / 180.0);
        double pitch_rad = gnss_state.baseline_pitch_deg * (M_PI / 180.0);
        double horiz_len = gnss_state.baseline_length_m * cos(pitch_rad);
        double d_north = horiz_len * cos(hdg_rad);
        double d_east  = horiz_len * sin(hdg_rad);
        double m_per_deg_lat = 111132.954 - 559.822 * cos(2.0 * lat_rad) + 1.175 * cos(4.0 * lat_rad);
        double m_per_deg_lon = (111412.84 * cos(lat_rad) - 93.5 * cos(3.0 * lat_rad));
        state_out->slave_lat      = gnss_state.raw_lat + (d_north / m_per_deg_lat);
        state_out->slave_lon      = gnss_state.raw_lon + (d_east / m_per_deg_lon);
    } else {
        state_out->slave_lat      = 0.0;
        state_out->slave_lon      = 0.0;
    }

    // Base Station Telemetry
    state_out->base_station_id    = gnss_state.base_station_id;
    state_out->base_coords_valid  = gnss_state.base_coords_valid;
    state_out->base_lat           = gnss_state.base_lat;
    state_out->base_lon           = gnss_state.base_lon;
    state_out->base_alt_m         = gnss_state.base_alt_m;
    state_out->base_ecef_x        = gnss_state.base_ecef_x;
    state_out->base_ecef_y        = gnss_state.base_ecef_y;
    state_out->base_ecef_z        = gnss_state.base_ecef_z;
}

extern "C" void um982_get_port_stats(Um982PortStats_t *stats_out)
{
	if (stats_out) {
		*stats_out = port_stats;
	}
}

extern "C" bool um982_poll_pending_rtc_sync(uint16_t *yy, uint8_t *mo, uint8_t *dd, uint8_t *hh, uint8_t *mm, uint8_t *ss)
{
	if (!rtc_sync_pending) return false;
	time_t latched_epoch = s_pending_rtc_epoch;
	uint32_t latched_ms  = s_pending_rtc_sync_ms;
	rtc_sync_pending     = false;

	uint32_t elapsed_s = (millis() - latched_ms + 500U) / 1000U;
	time_t live_epoch  = latched_epoch + (time_t)elapsed_s;
	struct tm tm_utc;
	if (!gmtime_r(&live_epoch, &tm_utc)) return false;

	if (yy) *yy = (uint16_t)(tm_utc.tm_year + 1900);
	if (mo) *mo = (uint8_t)(tm_utc.tm_mon + 1);
	if (dd) *dd = (uint8_t)tm_utc.tm_mday;
	if (hh) *hh = (uint8_t)tm_utc.tm_hour;
	if (mm) *mm = (uint8_t)tm_utc.tm_min;
	if (ss) *ss = (uint8_t)tm_utc.tm_sec;
	return true;
}