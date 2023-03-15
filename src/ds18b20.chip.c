// Wokwi DS18B20 Custom Chip - For information and examples see:
// https://link.wokwi.com/custom-chips-alpha
//
// SPDX-License-Identifier: MIT
// Copyright (C) 2022 Bonny Rais

#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include "ow.h"

#define DEBUG 1

#define max(a, b) ({__typeof__(a) _a = (a); __typeof__(b) _b = b; _a > _b ? _a : b; })
#define min(a, b) ({__typeof__(a) _a = (a); __typeof__(b) _b = b; _a < _b ? _a : b; })
#define constrain(v, a, b) ({__typeof__(v) _v = (v); __typeof__(a) _a = (a); __typeof__(b) _b = b; min(max(v,a),b); })
#define in_range(v, a, b) ({__typeof__(v) _v = (v); __typeof__(a) _a = (a); __typeof__(b) _b = b; _a <= _v && _v <= _b; })

// --------------- Debug Macros -----------------------
#ifdef DEBUG
#define DEBUGF(...)      { if (chip->gen_debug) {printf("%lld ", get_sim_nanos()/1000); printf(__VA_ARGS__);} }
char buf[200];


const char*debugBinStr(char *p, size_t c) {
    char *pb = buf;
    c = constrain(c, 1, 16);

    for (; c > 0; --c) {
        uint8_t b = *p++;
        for ( int i = 0; i < 8; i++) {
            *pb++ = (b & 1 << i) ? '1' : '0'; 
        }
        *pb++ = ' ';
    }
    *pb = 0;
    return buf;
}

const char*debugHexStr(uint8_t *p, size_t c) {
    char *pb = (char *)buf;
    c = constrain(c, 1, 16);
    for (; c--;) {
        pb += sprintf(pb, "%02x ", *p++);
    }
    *pb = 0;
    return buf;
}

#endif //  DEBUG
// --------------- Debug Macros -----------------------

// buffers
#define BUF_LEN 32
#define SERIAL_LEN 8
#define SCRATCH_LEN 9
#define EEPROM_LEN 3
#define CUR_BIT(chip) ((chip->buffer[chip->byte_ndx] & (1 << chip->bit_ndx)) != 0)

// Temperature consts
#define MAX_TEMPERATURE (125)
#define MIN_TEMPERATURE (-55)

// Temperature sensor family codes (byte 0 of serial number)
#define DS_FC_18S20     0x10
#define DS_FC_18B20     0x28
#define DS_FC_1822      0x22
// #define DS_FC_1825      0x3B
// #define DS_FC_28EA00    0x42

// rom commands
#define OW_CMD_SEARCH           0xF0
#define OW_CMD_READ             0x33
#define OW_CMD_MATCH            0x55
#define OW_CMD_SKIP             0xCC
#define OW_CMD_ALM_SEARCH       0xEC

#define DS_CMD_CONVERT          0x44
#define DS_CMD_WR_SCRATCH       0x4E
#define DS_CMD_RD_SCRATCH       0xBE
#define DS_CMD_CP_SCRATCH       0x48
#define DS_CMD_RECALL           0xB8
#define DS_CMD_RD_PWD           0xB4

// scratch pad offsets (DS18B20)
#define CHIP_SP_TEMP_LOW_OFF    0x00
#define CHIP_SP_TEMP_HI_OFF     0x01
#define CHIP_SP_USER_BYTE_1_OFF 0x02
#define CHIP_SP_USER_BYTE_2_OFF 0x03
#define CHIP_SP_CFG_REG_OFF     0x04
#define CHIP_SP_RSVD_1_OFF      0x05
#define CHIP_SP_RSVD_2_OFF      0x06
#define CHIP_SP_RSVD_3_OFF      0x07
#define CHIP_SP_CRC_OFF         0x08

// scratch pad offsets  - diff only (DS18S20)
#define CHIP_SP_REMAIN_CNT_OFF  0x06
#define CHIP_SP_CNT_PER_C_OFF   0x07

// EEPROM offsets
#define CHIP_EE_TEMP_LOW_OFF    0x00
#define CHIP_EE_TEMP_HI_OFF     0x01
#define CHIP_EE_CFG_REG_OFF     0x02

// Configuration register bits
#define CHIP_CFG_TEMP_BITS_MASK 0x60
#define CHIP_CFG_TEMP_BITS_OFF  5      


typedef enum {
    TW_FIXED,
    TW_SQUARE,
    TW_SINE,
    TW_TRIANGLE
} wave_mode_t;

typedef enum {
    ST_SIG_BIT_MODE,
    ST_SIG_BYTE_MODE,
} sig_mode_t;

typedef enum {
    ST_INIT_SEQ,
    ST_WAIT_CMD,
    ST_WAIT_FN_CMD,
    ST_EXEC_CMD,

    ST_MAX
} chip_state_t;

typedef enum {
    ST_MASTER_SEARCH_INIT,
    ST_MASTER_SEARCH_WRITE_BIT,
    ST_MASTER_SEARCH_WRITE_INV_BIT,
    ST_MASTER_SEARCH_READ_BIT,
    ST_MASTER_SEARCH_MAX
} search_state_t;

typedef enum {
    ST_MASTER_MATCH_READ_BIT,
    ST_MASTER_MATCH_MAX
} match_state_t;

typedef enum {
    ST_MASTER_WR_SP_BYTE_READ,
    ST_MASTER_WR_SP_MAX
} wr_sp_state_t;

typedef enum {
    ST_MASTER_RD_BYTE_BYTE_WRITTEN,
    ST_MASTER_RD_BYTE_MAX
} rd_byte_state_t;

typedef enum {
    ST_MASTER_RD_BIT_BIT_WRITTEN,
    ST_MASTER_RD_BIT_MAX
} wr_bit_state_t;

typedef struct  {
    uint8_t bit_ndx;
} cmd_search_ctx_t;

typedef struct  {
} cmd_match_ctx_t;

typedef struct  {
    int byte_ndx;
    int resp_len;
    bool restart_when_done;
} cmd_byte_op_ctx_t;

typedef struct {
    uint32_t state;
    sm_t *cmd_sm;
    union {
        cmd_search_ctx_t search_ctx;
        cmd_match_ctx_t match_ctx;
        cmd_byte_op_ctx_t wr_sp_ctx;
        cmd_byte_op_ctx_t rd_byte_ctx;
    } cmd_data;
} cmd_ctx_t;

typedef struct
{
    uint8_t serial_no[SERIAL_LEN];
    uint8_t scratch_pad[SCRATCH_LEN];
    uint8_t eeprom[EEPROM_LEN];

    // onw wire helpers
    ow_ctx_t *ow_ctx;
    ow_byte_ctx_t *ow_write_byte_ctx;
    ow_byte_ctx_t *ow_read_byte_ctx;

    chip_state_t state;             // overwall chip one wire state
    sig_mode_t sig_mode;            // signaling mode - handling bits or bytes
    cmd_ctx_t cmd_ctx;              // current command context
    uint32_t rom_command;           // remember the current command (needed for search/almsearch)

    // comms buffer
    uint8_t buffer[BUF_LEN];        
    int bit_ndx;
    int byte_ndx;

    // the temperature from config and alarm value based on last conversion
    float temperature;
    uint8_t temperature_attr; // Allow dynamic reading ot temperature
    bool alarm;
    float min_temp;
    float max_temp;
    float temp_chg_freq;
    timer_t temp_wave_timer;
    wave_mode_t temp_mode;
    int temp_wave_slot;

    

    // the power pin, used to determine power mode
    pin_t vcc_pin;
    bool powered;

    // debug
    bool debug_timer;
    bool ow_debug;
    bool gen_debug;

} chip_desc_t;


typedef void (*ow_cmd_handler)(chip_desc_t *chip);
typedef struct {
    uint16_t cmd_key;
    ow_cmd_handler handler;
} cmd_entry_t;
typedef HASHMAP(uint16_t, cmd_entry_t) cmd_map_t;


// ==================== forward decls =========================
static void chip_reset_state(chip_desc_t *chip);

void on_timer_event(void *user_data);
void on_forced_reset_cb(void *d, uint32_t err, uint32_t data) ;
void on_reset_cb(void *d, uint32_t err, uint32_t data) ;
void on_bit_written_cb(void *d, uint32_t err, uint32_t data);
void on_bit_read_cb(void *d, uint32_t err, uint32_t data) ;
void on_byte_read_cb(void *d, uint32_t err, uint32_t data);
void on_byte_written_cb(void *d, uint32_t err, uint32_t data);

void on_search_bit_written_cb(void *d, uint32_t err, uint32_t data);
void on_search_bit_read_cb(void *d, uint32_t err, uint32_t data) ;

static void on_master_search_error(void *user_data, uint32_t data) ;
static void on_master_search_bit_written(void *user_data, uint32_t data);
static void on_master_search_inv_bit_written(void *user_data, uint32_t data);
static void on_master_search_bit_read(void *user_data, uint32_t data);

static void on_master_match_error(void *user_data, uint32_t data) ;
static void on_master_match_bit_read(void *user_data, uint32_t data);

static void on_master_wr_sp_byte_read(void *user_data, uint32_t data);
static void on_master_rd_byte_byte_written(void *user_data, uint32_t data);

static void on_master_rd_bit_bit_written(void *user_data, uint32_t data);

// --- command handlers 
static void on_rom_command(chip_desc_t *chip, uint8_t cmd);
static void on_func_command(chip_desc_t *chip, uint8_t cmd);

static void on_ow_search(chip_desc_t *chip);
static void on_ow_read_rom(chip_desc_t *chip);
static void on_ow_match(chip_desc_t *chip);
static void on_ow_skip(chip_desc_t *chip);
static void on_ow_alarm_search(chip_desc_t *chip);

static void on_ds_convert(chip_desc_t *chip);
static void on_ds_write_scratchpad(chip_desc_t *chip);
static void on_ds_read_scratchpad(chip_desc_t *chip);
static void on_ds_copy_scratchpad(chip_desc_t *chip);
static void on_ds_recall(chip_desc_t *chip);
static void on_ds_read_power(chip_desc_t *chip);



// ==== Search SM ====
static sm_entry_t sm_search_entries[] = { //[ST_MASTER_SEARCH_MAX][EV_MASTER_SEARCH_MAX] = {

    // ST_MASTER_SEARCH_WRITE_BIT
    SM_E(ST_MASTER_SEARCH_WRITE_BIT, EV_BIT_WRITTEN, on_master_search_bit_written),
    SM_E(ST_MASTER_SEARCH_WRITE_BIT, EV_BIT_READ, on_master_search_error),

        // ST_MASTER_SEARCH_WRITE_BIT
    SM_E(ST_MASTER_SEARCH_WRITE_INV_BIT, EV_BIT_WRITTEN, on_master_search_inv_bit_written),
    SM_E(ST_MASTER_SEARCH_WRITE_INV_BIT, EV_BIT_READ, on_master_search_error),

        // ST_MASTER_SEARCH_READ_BIT
    SM_E(ST_MASTER_SEARCH_READ_BIT, EV_BIT_WRITTEN, on_master_search_error),
    SM_E(ST_MASTER_SEARCH_READ_BIT, EV_BIT_READ, on_master_search_bit_read),

};

static sm_cfg_t sm_search_cfg = {
        .name = "sm_search",
        .sm_entries = sm_search_entries,
        .num_entries = sizeof(sm_search_entries)/sizeof(sm_entry_t),
};

static sm_t *sm_search = &(sm_t){.cfg = &sm_search_cfg};


// ==== Match SM ====

static sm_entry_t sm_match_entries[] = { //[ST_MASTER_MATCH_MAX][EV_MASTER_MATCH_MAX] = {

        // ST_MASTER_MATCH_READ_BIT
        SM_E(ST_MASTER_MATCH_READ_BIT, EV_BIT_WRITTEN, on_master_match_error),
        SM_E(ST_MASTER_MATCH_READ_BIT, EV_BIT_READ, on_master_match_bit_read),

};

static sm_cfg_t sm_match_cfg = {
        .name = "sm_match",
        .sm_entries = sm_match_entries,
        .num_entries = sizeof(sm_match_entries)/sizeof(sm_entry_t),
};

static sm_t *sm_match = &(sm_t){.cfg = &sm_match_cfg};


// ==== Write Scratchpad SM ====

static sm_cfg_t sm_wr_sp_cfg = {
        .name = "sm_wr_sp_cfg",
        .sm_entries = (sm_entry_t[]){ //[ST_MASTER_MATCH_MAX][EV_MASTER_MATCH_MAX] = {
                // ST_MASTER_MATCH_READ_BIT
                SM_E(ST_MASTER_WR_SP_BYTE_READ, EV_BYTE_READ, on_master_wr_sp_byte_read),
        },
        .num_entries = 1,
};

static sm_t *sm_wr_sp = &(sm_t){.cfg = &sm_wr_sp_cfg};


// ==== Read Byte SM ====

static sm_cfg_t sm_rd_byte_cfg = {
        .name = "sm_rd_byte",
        .sm_entries = (sm_entry_t[]){
                // ST_MASTER_RD_SP_BYTE_RD
                SM_E(ST_MASTER_RD_BYTE_BYTE_WRITTEN, EV_BYTE_WRITTEN, on_master_rd_byte_byte_written),
        },
        .num_entries = 1,
};

static sm_t *sm_rd_byte = &(sm_t){.cfg = &sm_rd_byte_cfg};


static sm_cfg_t sm_wr_bit_cfg = {
        .name = "sm_wr_bit",
        .sm_entries = (sm_entry_t[]){
                // ST_MASTER_RD_SP_BYTE_RD
                SM_E(ST_MASTER_RD_BIT_BIT_WRITTEN, EV_BIT_WRITTEN, on_master_rd_bit_bit_written),
        },
        .num_entries = 1,
};

static sm_t *sm_wr_bit = &(sm_t){.cfg = &sm_wr_bit_cfg};


static uint8_t supported_family_codes[] = { 
    DS_FC_18S20, DS_FC_18B20//, DS_FC_1822
};

static cmd_entry_t cmd_entries[] = {
    { ST_WAIT_CMD << 8 | OW_CMD_MATCH, on_ow_match },
    { ST_WAIT_CMD << 8 | OW_CMD_SKIP, on_ow_skip },
    { ST_WAIT_CMD << 8 | OW_CMD_SEARCH, on_ow_search },
    { ST_WAIT_CMD << 8 | OW_CMD_ALM_SEARCH, on_ow_alarm_search },
    { ST_WAIT_CMD << 8 | OW_CMD_READ, on_ow_read_rom },

    { ST_WAIT_FN_CMD << 8 | DS_CMD_WR_SCRATCH, on_ds_write_scratchpad },
    { ST_WAIT_FN_CMD << 8 | DS_CMD_RD_SCRATCH, on_ds_read_scratchpad },
    { ST_WAIT_FN_CMD << 8 | DS_CMD_CP_SCRATCH, on_ds_copy_scratchpad },
    { ST_WAIT_FN_CMD << 8 | DS_CMD_CONVERT, on_ds_convert },
    { ST_WAIT_FN_CMD << 8 | DS_CMD_RECALL, on_ds_recall },
    { ST_WAIT_FN_CMD << 8 | DS_CMD_RD_PWD, on_ds_read_power },
};

static cmd_map_t *cmd_map;

// Dow-CRC using polynomial X^8 + X^5 + X^4 + X^0
// Tiny 2x16 entry CRC table created by Arjen Lentz
// See http://lentz.com.au/blog/calculating-crc-with-a-tiny-32-entry-lookup-table
static const uint8_t dscrc2x16_table[] = {
	0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
	0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
	0x00, 0x9D, 0x23, 0xBE, 0x46, 0xDB, 0x65, 0xF8,
	0x8C, 0x11, 0xAF, 0x32, 0xCA, 0x57, 0xE9, 0x74
};

// Compute a Dallas Semiconductor 8 bit CRC. These show up in the ROM
// and the registers.  (Use tiny 2x16 entry CRC table)
uint8_t crc8(const uint8_t *addr, uint8_t len)
{
	uint8_t crc = 0;

	while (len--) {
		crc = *addr++ ^ crc;  // just re-using crc as intermediate
		crc = dscrc2x16_table[crc & 0x0f] ^ dscrc2x16_table[ 16 + ((crc >> 4) & 0x0f)];
	}

	return crc;
}

void update_crc8(chip_desc_t *chip) {
    chip->scratch_pad[CHIP_SP_CRC_OFF] = crc8(chip->scratch_pad, SCRATCH_LEN - 1);
}


int cmd_key_compare(const uint16_t *k1, const uint16_t *k2 ) { return *k1 == *k2 ? 0 : *k1 > *k2 ? 1 : -1; }
size_t cmd_key_hash(const uint16_t *k1 ) { return hashmap_hash_default(k1, sizeof(*k1)); }
uint16_t cmd_to_key(uint16_t state, uint16_t cmd) { return state << 8 | cmd; }
void cmd_init_hash() {
    cmd_map = calloc(1, sizeof(cmd_map_t));

    hashmap_init(cmd_map, cmd_key_hash, cmd_key_compare);
    for( int i= 0; i < sizeof(cmd_entries)/sizeof(cmd_entry_t); i++) {
        cmd_entry_t *e = cmd_entries+i;
        hashmap_put(cmd_map, &e->cmd_key, e);
    }
}



// ==================== Implementation =========================

void chip_attr_init(chip_desc_t *chip) {
    
    // read config attributes
    uint32_t attr;
    uint32_t len;
    char str_attr[SERIAL_LEN * 2];
    
    attr = attr_init("ow_debug", false); chip->ow_debug = attr_read(attr) != 0;
    attr = attr_init("gen_debug", false); chip->gen_debug = attr_read(attr) != 0;
    attr = attr_init("debug_timer", false); chip->debug_timer = attr_read(attr) != 0;

    attr = attr_init_float("temperature", 0);
    chip->temperature_attr = attr; // Store the attribute to allow dynamic reading
    chip->temperature = constrain(attr_read_float(attr), MIN_TEMPERATURE, MAX_TEMPERATURE);
    attr = attr_init_float("min_temp", MIN_TEMPERATURE);
    chip->min_temp = constrain(attr_read_float(attr), MIN_TEMPERATURE, MAX_TEMPERATURE);
    attr = attr_init_float("max_temp", MAX_TEMPERATURE);
    chip->max_temp = constrain(attr_read_float(attr), MIN_TEMPERATURE, MAX_TEMPERATURE);
    attr = attr_init_float("temp_wave_freq", 0);
    chip->temp_chg_freq = constrain(attr_read_float(attr), 0, 100);
    attr = attr_string_init("temp_wave_form");
    len = string_read(attr, str_attr, 8 + 1 );  // allowing for none|sine|square|triangle 8 + NULL
    printf("reading temp mode: %s\n", str_attr);
    for(int i = 0; str_attr[i]; i++){ str_attr[i] = tolower(str_attr[i]); }
    chip->temp_mode = TW_FIXED;
    if (!strcmp(str_attr, "sine")) chip->temp_mode = TW_SINE;
    if (!strcmp(str_attr, "square")) chip->temp_mode = TW_SQUARE;
    if (!strcmp(str_attr, "triangle")) chip->temp_mode = TW_TRIANGLE;

    if (chip->min_temp > chip->max_temp) {
        float t = chip->min_temp;
        chip->min_temp = chip->max_temp;
        chip->max_temp = t;
    }

    // initialise device id
    attr = attr_init("family_code", DS_FC_18S20); chip->serial_no[0] = attr_read(attr) & 0xFF;
    if (memchr(supported_family_codes, chip->serial_no[0], sizeof(supported_family_codes)) == NULL) {
        printf("*** DS18B20 device family code not supported (%d), expect errors...\n", chip->serial_no[0]);
    }

    attr = attr_string_init("device_id"); 
    len = string_read(attr, str_attr, 13 );  // expecting 12 Hex Digits + NULL
    if (len < 12) {
        printf("*** DS18B20 device id too short (%d), expect errors...\n", len);
    }

    for (int i = 0; i < 6; i++) {
        char tmp = str_attr[i * 2 + 2];
        str_attr[i*2+2] = 0;
        chip->serial_no[i + 1] = strtol(str_attr + i * 2, NULL, 16 );
        str_attr[i*2+2] = tmp;
    }

    chip->serial_no[7] = crc8(chip->serial_no, 7);

//    attr = attr_init("presence_wait_time", PR_DUR_WAIT_PRESENCE);
//    chip->presence_wait_time = attr_read(attr);
//    attr = attr_init("presence_time", PR_DUR_PULL_PRESENCE);
//    chip->presence_time = attr_read(attr);

    printf("*** DS18B20 setting attributes:\n  gen_debug: %d\n  ow_debug: %d\n  temperature: %f\n  family_code: %2x\n"
           "  min_temp: %f\n  max_temp: %f\n  temp_freq: %f\n  temp_mode: %d\n", 
    chip->gen_debug, chip->ow_debug, chip->temperature, chip->serial_no[0],
    chip->min_temp, chip->max_temp, chip->temp_chg_freq, chip->temp_mode);

    printf("  device_id: ");
    for (int i = 0; i < SERIAL_LEN; i++ ){
        printf("%02x", chip->serial_no[i]);
    }
    printf("\n");
}

void chip_init()
{
//    setvbuf(stdout, NULL, _IOLBF, 1024);
    printf("*** DS18B20 chip initialising...\n");
    chip_desc_t *chip = calloc(1, sizeof(chip_desc_t));

    chip_attr_init(chip);

    ow_ctx_cfg_t cfg = {
            .bit_written_cb = on_bit_written_cb,
            .bit_read_cb = on_bit_read_cb,
            .reset_cb = on_reset_cb,
            .forced_reset_cb = on_forced_reset_cb,
            .pin_name = "DQ",
            .data = chip,
    };
    ow_ctx_t *ow_ctx = ow_ctx_init(&cfg);


    chip->ow_ctx = ow_ctx;
    chip->ow_read_byte_ctx = ow_read_byte_ctx_init(chip, on_byte_read_cb, ow_ctx);
    chip->ow_write_byte_ctx = ow_write_byte_ctx_init(chip, on_byte_written_cb, ow_ctx);

    // initialise command map
    cmd_init_hash();

    chip->vcc_pin = pin_init("VCC", INPUT);
    chip->powered = pin_read(chip->vcc_pin);

    // initialise temperature timers if needed
    if (chip->temp_mode != TW_FIXED && chip->temp_chg_freq > 0.001) {
        timer_config_t timer_cfg = {
            .user_data = chip,
            .callback = on_timer_event,
        };

        chip->temperature = (chip->max_temp - chip->min_temp) / 2;
        chip->temp_wave_timer = timer_init(&timer_cfg);
        timer_start(chip->temp_wave_timer, (1e4 / chip->temp_chg_freq), true);
    }

    chip_reset_state(chip);
    printf("DS18B20 chip initialised\n");
}

static void chip_reset_state(chip_desc_t *chip) {
    DEBUGF("resetting chip state\n");
    ow_ctx_reset_state(chip->ow_ctx);

    chip->sig_mode = ST_SIG_BIT_MODE;
    chip->state = ST_INIT_SEQ;
    memset(&chip->cmd_ctx, 0, sizeof(cmd_ctx_t));

    chip->bit_ndx = 0;
    chip->byte_ndx = 0;
    memset(chip->buffer, 0, BUF_LEN);

    // setup scratch pad based on family code
    switch (chip->serial_no[0]) {
        default:
        case DS_FC_18S20:
            //see  https://www.analog.com/media/en/technical-documentation/data-sheets/ds18s20.pdf (measuring temp)
            // chip->scratch_pad[CHIP_SP_REMAIN_CNT_OFF] = 0; 
            chip->scratch_pad[CHIP_SP_CNT_PER_C_OFF] = 16; 

        case DS_FC_18B20:
            chip->scratch_pad[CHIP_SP_RSVD_1_OFF] = 0xFF; 
            chip->scratch_pad[CHIP_SP_RSVD_3_OFF] = 0x10; 
    }
}

static void chip_ready_for_next_cmd_byte(chip_desc_t *chip, chip_state_t state, const char *type) {
    DEBUGF("readying chip state for next %s command\n", type);
    ow_ctx_set_master_write_state(chip->ow_ctx, false);

    chip->sig_mode = ST_SIG_BYTE_MODE;
    chip->state = state;
    memset(&chip->cmd_ctx, 0, sizeof(cmd_ctx_t));

    chip->bit_ndx = 0;
    chip->byte_ndx = 0;
    memset(chip->buffer, 0, BUF_LEN);

}

static void chip_ready_for_next_cmd(chip_desc_t *chip) {
    chip_ready_for_next_cmd_byte(chip, ST_WAIT_CMD, "rom");
}
static void chip_ready_for_next_func_cmd(chip_desc_t *chip) {
    chip_ready_for_next_cmd_byte(chip, ST_WAIT_FN_CMD, "func");
}

// ==================== API handlers =========================
void push_cmd_sm_event(chip_desc_t *chip, sm_t *sm, uint32_t state, uint32_t ev, uint32_t ev_data) {

    if (!sm->hash) {
        DEBUGF("%s Initialising hash\n", sm->cfg->name);
        sm_init_hash(sm);
    }

    uint64_t key = state_event_to_key(state, ev);
    sm_entry_t *e = hashmap_get((sm_entry_map_t *)sm->hash, &key);

    if (e == NULL || e->handler == NULL) {
        DEBUGF("(%s) SM error: unhandled event %d in state %d (e %p)), resetting\n", sm->cfg->name, ev, state, e);
        chip_reset_state(chip);
        return;
    } else {
        DEBUGF("%s %s[%s]: %s( %d ) -> %p\n",
               sm->cfg->name, e->st_name,
               e->ev_name, e->name, ev_data, e->handler);
        e->handler(chip, ev_data);
    }


    key = state_event_to_key(chip->cmd_ctx.state, ev);
    e = hashmap_get((sm_entry_map_t *)sm->hash, &key);
    const char *n = e != NULL ? e->st_name : "invalid state";
    DEBUGF("(%s) new state -> %s\n", sm->cfg->name, n);
}


// ==================== API handlers =========================
void on_timer_event(void *data) {
    chip_desc_t *chip = data;
    int32_t slot  = (chip->temp_wave_slot++) % 100;
    float y = 0;
    float r = (chip->max_temp - chip->min_temp);

    switch(chip->temp_mode) {
        case TW_TRIANGLE: y = (0.04 * abs((((slot - 25 % 100) + 100) % 100) - 50) - 1) / 2 ; break;
        case TW_SINE:   y = sin(2 * M_PI * chip->temp_chg_freq * slot) / 2; break;
        case TW_SQUARE:   y = slot < 50 ? .5 : -.5; break;
        case TW_FIXED: y = 0.5;
    }

    chip->temperature = chip->min_temp + r * ( y + 0.5);
}

void on_forced_reset_cb(void *d, uint32_t err, uint32_t data) {
    chip_desc_t *chip = d;
    DEBUGF("on_force_reset_cb\n");

    chip_reset_state(chip);
}


void on_reset_cb(void *d, uint32_t err, uint32_t data) {
    chip_desc_t *chip = d;
    DEBUGF("on_reset_cb\n");

    // reset is done, now wait for master to write command byte, defer to byte SM
    chip_ready_for_next_cmd(chip);
}

// callback used when a bit has been written to the master via the signalling SM
void on_bit_written_cb(void *d, uint32_t err, uint32_t data) {
    chip_desc_t *chip = d;

    if (chip->sig_mode == ST_SIG_BYTE_MODE) {
        DEBUGF("on_bit_written_cb: calling bit_callback %p (%02x)\n", chip->ow_read_byte_ctx, data);
        chip->ow_write_byte_ctx->bit_callback(chip->ow_write_byte_ctx, err, data);
    } else {
        // current command is directly handling bits, run its state machine
        DEBUGF("on_bit_written_cb: calling own sm with data (%02x)\n", data);
        push_cmd_sm_event(chip, chip->cmd_ctx.cmd_sm, chip->cmd_ctx.state, EV_BIT_WRITTEN, data);
    }
}

// callback used when a bit has been read from the master via the signalling SM
void on_bit_read_cb(void *d, uint32_t err, uint32_t data) {
    chip_desc_t *chip = d;

    if (chip->sig_mode == ST_SIG_BYTE_MODE) {
        // current command is deferring bits to byte writer
        DEBUGF("on_bit_read_cb: calling bit_callback %p (%d)\n", chip->ow_write_byte_ctx, data);
        chip->ow_read_byte_ctx->bit_callback(chip->ow_read_byte_ctx, err, data);
    } else {
        // current command is directly handling bits, run its state machine
        DEBUGF("on_bit_read_cb: calling own sm (%d)\n", data);
        push_cmd_sm_event(chip, chip->cmd_ctx.cmd_sm, chip->cmd_ctx.state, EV_BIT_READ, data);
    }
}

// callback used when a byte has been read from the master via the signalling SM
void on_byte_read_cb(void *d, uint32_t err, uint32_t data) {
    chip_desc_t *chip = d;

    DEBUGF("on_master_byte_read_cb\n");
    // if we're waiting on command code, we handle directly, otherwise the byte is passed to the current command handlers
    if (chip->sig_mode == ST_SIG_BYTE_MODE ) {
        if (chip->state == ST_WAIT_CMD) {
            DEBUGF("on_master_byte_read_cb - processing rom command %02X\n", data);
            on_rom_command(chip, data);
            return;
        } if (chip->state == ST_WAIT_FN_CMD) {
             DEBUGF("on_master_byte_read_cb - processing func command %02X\n", data);
            on_func_command(chip, data);
            return;
        }
    }

    // todo(bonnyr): only handle if we're in the middle of command
    push_cmd_sm_event(chip, chip->cmd_ctx.cmd_sm, chip->cmd_ctx.state, EV_BYTE_READ, data);
}


// callback used when a byte has been written from the master via the signalling SM
void on_byte_written_cb(void *d, uint32_t err, uint32_t data) {
    chip_desc_t *chip = d;
    DEBUGF("on_byte_written_cb\n");
    push_cmd_sm_event(chip, chip->cmd_ctx.cmd_sm, chip->cmd_ctx.state, EV_BYTE_WRITTEN, data);
}


// ------------- Seach command SM -----------------

void on_search_bit_written_cb(void *d, uint32_t err, uint32_t data) {
    chip_desc_t *chip = d;

    if (err != 0) {
        DEBUGF("write byte: Error occurred while waiting for bit to be read")
        chip_reset_state(chip);
        return;
    }

    push_cmd_sm_event(chip, chip->cmd_ctx.cmd_sm, chip->cmd_ctx.state, EV_BIT_WRITTEN, data);
}

void on_search_bit_read_cb(void *d, uint32_t err, uint32_t data) {
    chip_desc_t *chip = d;

    if (err != 0) {
        DEBUGF("write byte: Error occurred while waiting for bit to be read")
        chip_reset_state(chip);
        return;
    }

    push_cmd_sm_event(chip, chip->cmd_ctx.cmd_sm, chip->cmd_ctx.state, EV_BIT_READ, data);
}

static void ds_func_cmd_prime_next_bit(chip_desc_t *chip) {
    ow_ctx_set_master_read_state(chip->ow_ctx, CUR_BIT(chip));
    DEBUGF("ds_func_cmd_prime_next_bit: ow_ctx: %p, ow_ctx->state: %d\n", chip->ow_ctx, chip->ow_ctx->state);
}

static void on_master_search_error(void *user_data, uint32_t data) {
    chip_desc_t *chip = user_data;

    DEBUGF("on_master_search_error: \n");
    chip_reset_state(chip);
}

static void on_master_search_bit_written(void *user_data, uint32_t data) {
    chip_desc_t *chip = user_data;
    DEBUGF("on_master_search_bit_written: %d (cur bit: %d)\n", data, CUR_BIT(chip));

    chip->cmd_ctx.state = ST_MASTER_SEARCH_WRITE_INV_BIT;

    // todo(bonnyr): refactor to a function in signalling
    ow_ctx_set_master_read_state(chip->ow_ctx, !CUR_BIT(chip));
}

static void on_master_search_inv_bit_written(void *user_data, uint32_t data) {
    chip_desc_t *chip = user_data;
    DEBUGF("on_master_search_inv_bit_written: %d (cur bit: %d)\n", data, !CUR_BIT(chip));

    chip->cmd_ctx.state = ST_MASTER_SEARCH_READ_BIT;
    // todo(bonnyr): refactor to a function in signalling
    ow_ctx_set_master_write_state(chip->ow_ctx, CUR_BIT(chip));
}


static void on_master_search_bit_read(void *user_data, uint32_t data) {
    chip_desc_t *chip = user_data;

    DEBUGF("on_master_search_bit_read: comparing bit %d - m:%d, d:%d\n", chip->byte_ndx * 8 + chip->bit_ndx, data, CUR_BIT(chip))
    // if master transmitted bit does not match ours, reset
    if (data != CUR_BIT(chip))  {
        chip_reset_state(chip);
        return;
    }

    // if this is an alarm search and the chip did not record it, terminate
    // this is only done after the first bit
    if (chip->bit_ndx == 0 && chip->rom_command == OW_CMD_ALM_SEARCH && !chip->alarm) {
        DEBUGF("on_master_search_bit_read: alarm search terminates since we are not alarmed\n");
        chip_reset_state(chip);
        return;
    }

    chip->bit_ndx++;
    if (chip->bit_ndx >= 8) {
        chip->bit_ndx = 0;
        chip->byte_ndx++;
    }

    if (chip->byte_ndx == SERIAL_LEN) {
        DEBUGF("on_master_search_bit_read: *** finished search, going back to init\n");
        chip_reset_state(chip);
        return;
    }

    // go for more
    ds_func_cmd_prime_next_bit(chip);
    chip->cmd_ctx.state = ST_MASTER_SEARCH_WRITE_BIT;
}

// ------------- Write Scratchppad command SM -----------------
static void on_master_write_scratchpad_error(void *user_data, uint32_t data) {
    chip_desc_t *chip = user_data;

    DEBUGF("on_master_write_scratchpad_error: \n");
    chip_reset_state(chip);
}

static void on_master_wr_sp_byte_read(void *user_data, uint32_t data) {
    chip_desc_t *chip = user_data;
    bool done = false;

    chip->cmd_ctx.cmd_data.wr_sp_ctx.byte_ndx++;

    if (chip->cmd_ctx.cmd_data.wr_sp_ctx.byte_ndx == 1) {
        chip->scratch_pad[CHIP_SP_USER_BYTE_1_OFF] = data & 0xFF;
        DEBUGF("on_master_wr_sp_byte_read: writing TH byte %02x\n", data);
    } 
    
    if (chip->cmd_ctx.cmd_data.wr_sp_ctx.byte_ndx == 2 ) {
        chip->scratch_pad[CHIP_SP_USER_BYTE_2_OFF] = data & 0xFF;
        DEBUGF("on_master_wr_sp_byte_read: writing TL byte %02x\n", data);
    } 

    if (chip->cmd_ctx.cmd_data.wr_sp_ctx.byte_ndx == 3 ) {
        chip->scratch_pad[CHIP_SP_CFG_REG_OFF] = data & 0xFF;
        DEBUGF("on_master_wr_sp_byte_read: writing CFG byte %02x\n", data);
    }

    // 3rd byte depends on family code    
    switch (chip->serial_no[0]) {
        case DS_FC_18S20:
            done = chip->cmd_ctx.cmd_data.wr_sp_ctx.byte_ndx >= 2;
            break;

        case DS_FC_18B20:
        default:
            done = chip->cmd_ctx.cmd_data.wr_sp_ctx.byte_ndx >= 3;
    }

    if (done)
    {
        update_crc8(chip);
        chip_reset_state(chip);
        DEBUGF("on_master_wr_sp_byte_read: *** finished, starting next cycle\n");
    }
    DEBUGF("on_master_wr_sp_byte_read: scratchpad: %s\n", debugHexStr(chip->scratch_pad, SCRATCH_LEN));  
}

// ------------- Read Byte command SM -----------------
static void on_master_rd_byte_prime_next_byte(chip_desc_t *chip, int ndx) {
    ow_ctx_set_master_read_state(chip->ow_ctx, false);
    ow_write_byte_ctx_reset_state(chip->ow_write_byte_ctx, chip->buffer[ndx]);
}

static void on_master_rd_byte_byte_written(void *user_data, uint32_t data) {
    chip_desc_t *chip = user_data;

    chip->cmd_ctx.cmd_data.rd_byte_ctx.byte_ndx++;
    if (chip->cmd_ctx.cmd_data.rd_byte_ctx.byte_ndx == chip->cmd_ctx.cmd_data.rd_byte_ctx.resp_len) {
        DEBUGF("on_master_rd_byte_byte_written: *** finished, starting next cycle\n");
        for (int i = 0;i < chip->cmd_ctx.cmd_data.rd_byte_ctx.resp_len; i++)
            DEBUGF("on_master_rd_byte_byte_written: byte %d: %02x\n", i, chip->buffer[i]);
        
        if (chip->cmd_ctx.cmd_data.rd_byte_ctx.restart_when_done)            
            chip_reset_state(chip);
        else 
            chip_ready_for_next_func_cmd(chip);

        return;
    }

    DEBUGF("on_master_rd_byte_byte_written: writing byte %d = %02x\n", chip->cmd_ctx.cmd_data.rd_byte_ctx.byte_ndx, chip->buffer[chip->cmd_ctx.cmd_data.rd_byte_ctx.byte_ndx]);
    on_master_rd_byte_prime_next_byte(chip, chip->cmd_ctx.cmd_data.rd_byte_ctx.byte_ndx);
}


// ------------- Match command SM -----------------
static void match_start_prime_next_bit(chip_desc_t *chip) {
    ow_ctx_set_master_write_state(chip->ow_ctx, CUR_BIT(chip));
    DEBUGF("match_start_prime_next_bit: ow_ctx: %p, ow_ctx->state: %d\n", chip->ow_ctx, chip->ow_ctx->state);
}

static void on_master_match_error(void *user_data, uint32_t data) {
    chip_desc_t *chip = user_data;

    DEBUGF("on_master_match_error: \n");
    chip_reset_state(chip);
}

static void on_master_match_bit_read(void *user_data, uint32_t data) {
    chip_desc_t *chip = user_data;

    DEBUGF("on_master_match_bit_read: comparing bit %d - m:%d, s:%d\n", chip->byte_ndx * 8 + chip->bit_ndx, data, CUR_BIT(chip))
    // if master transmitted bit does not match ours, reset
    if (data != CUR_BIT(chip))  {
        chip_reset_state(chip);
        return;
    }

    chip->bit_ndx++;
    if (chip->bit_ndx >= 8) {
        chip->bit_ndx = 0;
        chip->byte_ndx++;
    }

    if (chip->byte_ndx == SERIAL_LEN) {
        DEBUGF("on_master_match_bit_read: *** finished match, waiting for function command\n");
        chip_ready_for_next_func_cmd(chip);
        return;
    }

    // go for more
    match_start_prime_next_bit(chip);
    chip->cmd_ctx.state = ST_MASTER_MATCH_READ_BIT;
}


static void on_master_rd_bit_bit_written(void *user_data, uint32_t data) {
    chip_desc_t *chip = user_data;
    DEBUGF("on_master_rd_bit_bit_written: %d\n", data);

    chip_reset_state(chip);
}


// ==================== Logic Implementation =========================
static void on_command_word(chip_desc_t *chip, uint8_t cmd, const char *cmd_type_name, chip_state_t st) {
    DEBUGF("on_%s_command %2X\n", cmd_type_name, cmd);
    uint16_t key = cmd_to_key(st, cmd);
    cmd_entry_t *e = hashmap_get(cmd_map, &key);
    if (e != NULL) {
        e->handler(chip);
        return;
    }

    DEBUGF("**** %s command %02x not implemented\n", cmd_type_name, cmd);
    chip_reset_state(chip);
}

static void on_rom_command(chip_desc_t *chip, uint8_t cmd) {
    on_command_word(chip, cmd, "rom", ST_WAIT_CMD);
    chip->rom_command = cmd;
}

static void on_func_command(chip_desc_t *chip, uint8_t cmd) {
    on_command_word(chip, cmd, "func", ST_WAIT_FN_CMD);
}


static void set_next_state_based_on_power_mode(chip_desc_t *chip) {
    if (chip->powered) {
        chip->sig_mode = ST_SIG_BIT_MODE;
        chip->state = ST_EXEC_CMD;
        chip->cmd_ctx.state = ST_MASTER_RD_BIT_BIT_WRITTEN;
        chip->cmd_ctx.cmd_sm = sm_wr_bit;
        chip->bit_ndx = 0;
        chip->byte_ndx = 0;
        chip->buffer[0] = chip->powered;
        ds_func_cmd_prime_next_bit(chip);
    } else {
        chip_reset_state(chip);
    }
}

// master is searching for us. we need to report our id
// we're expecting master to initiate read bit
static void on_ow_search(chip_desc_t *chip) {
    DEBUGF("on_ow_search\n");
    memcpy(chip->buffer, chip->serial_no, SERIAL_LEN);
    chip->sig_mode = ST_SIG_BIT_MODE;
    chip->cmd_ctx.state = ST_MASTER_SEARCH_WRITE_BIT;
    chip->cmd_ctx.cmd_sm = sm_search;
    chip->bit_ndx = 0;
    chip->byte_ndx = 0;

    ds_func_cmd_prime_next_bit(chip);
    DEBUGF("on_ow_search started with %s\n", debugBinStr((char *)chip->buffer, SERIAL_LEN));
}

static void on_ow_read_rom(chip_desc_t *chip) {
    DEBUGF("on_ow_read_rom\n");
   memcpy(chip->buffer, chip->serial_no, SERIAL_LEN);

    chip->state = ST_EXEC_CMD;
    chip->cmd_ctx.state = ST_MASTER_RD_BYTE_BYTE_WRITTEN;
    chip->cmd_ctx.cmd_sm = sm_rd_byte;
    chip->cmd_ctx.cmd_data.rd_byte_ctx.restart_when_done = false;
    chip->cmd_ctx.cmd_data.rd_byte_ctx.resp_len = SERIAL_LEN;
    chip->bit_ndx = 0;
    chip->byte_ndx = 0;

    on_master_rd_byte_prime_next_byte(chip, 0);
}

static void on_ow_match(chip_desc_t *chip) {
    DEBUGF("on_ow_match\n");
    memcpy(chip->buffer, chip->serial_no, SERIAL_LEN);
    chip->sig_mode = ST_SIG_BIT_MODE;
    chip->cmd_ctx.state = ST_MASTER_MATCH_READ_BIT;
    chip->cmd_ctx.cmd_sm = sm_match;
    chip->bit_ndx = 0;
    chip->byte_ndx = 0;

    match_start_prime_next_bit(chip);
    DEBUGF("on_ow_match started\n");
}

static void on_ow_skip(chip_desc_t *chip) {
    DEBUGF("on_ow_skip\n");
    chip_ready_for_next_func_cmd(chip);
}

static void on_ow_alarm_search(chip_desc_t *chip) {
    DEBUGF("on_ow_alarm_search\n");
    on_ow_search(chip);
}

static void on_ds_convert(chip_desc_t *chip) {
    DEBUGF("on_ds_convert: scratch pad - : %s\n", debugHexStr(chip->scratch_pad, 9));
    int16_t tv;
    int16_t tv_frac;
    // Allow to set the temperature via GUI
    if (chip->temp_mode == TW_FIXED) chip->temperature = attr_read_float(chip->temperature_attr);
    
    // write our temp into the scratch pad, depending on family code. 
    switch (chip->serial_no[0]) {
        case DS_FC_18S20:
            tv = (int16_t)round(chip->temperature);
            tv_frac = 12 + 16 *(tv - chip->temperature);
            DEBUGF("on_ds_convert: DS_FC_18S20 temperature - chip: %f tv: %02x(%d) h7: %02x f: %02x\n", chip->temperature, tv, tv, ((tv & 0x7F) << 1), tv_frac);
            chip->scratch_pad[CHIP_SP_TEMP_HI_OFF] = tv < 0 ? 0xFF : 0;   // if any sign bit is on, value is 0xFF
            chip->scratch_pad[CHIP_SP_TEMP_LOW_OFF] = ((tv & 0x7F) << 1) | ((tv_frac & 0x8) ? 1 : 0);  
            chip->scratch_pad[CHIP_SP_REMAIN_CNT_OFF] = tv_frac;   // update REMAIN_CNT (use 3 or 4 bits?)
            break;

        case DS_FC_18B20:
        default: {
            // the configuration register determines how many significant bits we're using for conversion
            tv = (16 * chip->temperature);
            tv_frac =  tv & 0xF;
            uint16_t mask = 0xFFF0 | (0xF0 >>  (1 + ((chip->scratch_pad[CHIP_SP_CFG_REG_OFF] & CHIP_CFG_TEMP_BITS_MASK) >> CHIP_CFG_TEMP_BITS_OFF)));
            DEBUGF("on_ds_convert: DS_FC_18B20 temperature - chip: %f tv: %02X, mask:%04x, frac: %x, tv >> 12: %X\n", chip->temperature, tv, mask, tv_frac, tv >> 8);
            chip->scratch_pad[CHIP_SP_TEMP_HI_OFF] = tv >> 8;
            chip->scratch_pad[CHIP_SP_TEMP_LOW_OFF] = tv & 0xFF & mask;
        }
    }

    // set alarm flag as needed. Since threshold registers are only 7b + S, shift right to remove fractional temp
    char t = (char)chip->temperature;
    chip->alarm = (t > ((char)chip->scratch_pad[CHIP_SP_USER_BYTE_1_OFF]) || t < ((char)chip->scratch_pad[CHIP_SP_USER_BYTE_2_OFF]));
    DEBUGF("on_ds_convert: Setting alarm - : t: %d High %d %02x, Low: %d %02x: %d %s\n", t,
        (char)(chip->scratch_pad[CHIP_SP_USER_BYTE_1_OFF]), (chip->scratch_pad[CHIP_SP_USER_BYTE_1_OFF]), 
        (char)(chip->scratch_pad[CHIP_SP_USER_BYTE_2_OFF]), (chip->scratch_pad[CHIP_SP_USER_BYTE_2_OFF]), 
    chip->alarm, chip->alarm ? "yes" : "no");

    update_crc8(chip);
    set_next_state_based_on_power_mode(chip);
    DEBUGF("on_ds_convert: scratch pad - : %s\n", debugHexStr(chip->scratch_pad, 9));
}

static void on_ds_write_scratchpad(chip_desc_t *chip) {
    DEBUGF("on_ds_write_scratchpad: %s\n", debugHexStr(chip->scratch_pad, 9));
    chip->state = ST_EXEC_CMD;
    chip->cmd_ctx.state = ST_MASTER_WR_SP_BYTE_READ;
    chip->cmd_ctx.cmd_sm = sm_wr_sp;
    chip->bit_ndx = 0;
    chip->byte_ndx = 0;

    ow_ctx_set_master_write_state(chip->ow_ctx, false);
    ow_read_byte_ctx_reset_state(chip->ow_read_byte_ctx);
}

static void on_ds_read_scratchpad(chip_desc_t *chip) {
    DEBUGF("on_ds_read_scratchpad: %s\n", debugHexStr(chip->scratch_pad, 9));

    memcpy(chip->buffer, chip->scratch_pad, SCRATCH_LEN);
    chip->state = ST_EXEC_CMD;
    chip->cmd_ctx.state = ST_MASTER_RD_BYTE_BYTE_WRITTEN;
    chip->cmd_ctx.cmd_sm = sm_rd_byte;
    chip->cmd_ctx.cmd_data.rd_byte_ctx.restart_when_done = true;
    chip->cmd_ctx.cmd_data.rd_byte_ctx.resp_len = SCRATCH_LEN;
    chip->bit_ndx = 0;
    chip->byte_ndx = 0;

    on_master_rd_byte_prime_next_byte(chip, 0);
}

static void on_ds_copy_scratchpad(chip_desc_t *chip) {
    // write our temp into the scratch pad. Assume we're in parasitic mode for now
    chip->eeprom[CHIP_EE_TEMP_HI_OFF] = chip->scratch_pad[CHIP_SP_TEMP_HI_OFF];
    chip->eeprom[CHIP_EE_TEMP_LOW_OFF] = chip->scratch_pad[CHIP_SP_TEMP_LOW_OFF];

    switch (chip->serial_no[0]) {
        case DS_FC_18S20:
            break;

        case DS_FC_18B20:
        default: {
            chip->eeprom[CHIP_EE_CFG_REG_OFF] = chip->scratch_pad[CHIP_SP_CFG_REG_OFF];
        }
    }

    set_next_state_based_on_power_mode(chip);
    DEBUGF("on_ds_copy_scratchpad: *** finished, starting next cycle, scratchpad: %s\n", debugHexStr(chip->scratch_pad, 9));
}

static void on_ds_recall(chip_desc_t *chip) {
    DEBUGF("on_ds_recall\n");
    chip->scratch_pad[CHIP_SP_TEMP_HI_OFF] = chip->eeprom[CHIP_EE_TEMP_HI_OFF];
    chip->scratch_pad[CHIP_SP_TEMP_LOW_OFF] = chip->eeprom[CHIP_EE_TEMP_LOW_OFF];

    switch (chip->serial_no[0]) {
        case DS_FC_18S20:
            break;

        case DS_FC_18B20:
        default: {
            chip->scratch_pad[CHIP_SP_CFG_REG_OFF] = chip->eeprom[CHIP_EE_CFG_REG_OFF];
        }
    }

    update_crc8(chip);
    set_next_state_based_on_power_mode(chip);
    DEBUGF("on_ds_recall started: scratchpad: %s\n", debugHexStr(chip->scratch_pad, 9));
}

static void on_ds_read_power(chip_desc_t *chip) {
    chip->sig_mode = ST_SIG_BIT_MODE;
    chip->state = ST_EXEC_CMD;
    chip->cmd_ctx.state = ST_MASTER_RD_BIT_BIT_WRITTEN;
    chip->cmd_ctx.cmd_sm = sm_wr_bit;
    chip->bit_ndx = 0;
    chip->byte_ndx = 0;
    chip->buffer[0] = chip->powered;

    ds_func_cmd_prime_next_bit(chip);
    DEBUGF("on_ds_read_power: scratchpad: %s\n", debugHexStr(chip->scratch_pad, 9));
}
