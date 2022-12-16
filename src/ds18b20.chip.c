// Wokwi DS18B20 Custom Chip - For information and examples see:
// https://link.wokwi.com/custom-chips-alpha
//
// SPDX-License-Identifier: MIT
// Copyright (C) 2022 Bonny Rais

#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ow.h"

#define DEBUG 1

// --------------- Debug Macros -----------------------
#ifdef DEBUG
#define DEBUGF(...)      { if (chip->gen_debug) {printf(__VA_ARGS__);} }
#endif //  DEBUG
// --------------- Debug Macros -----------------------

// buffers
#define BUF_LEN 32
#define SERIAL_LEN 8
#define SCRATCH_LEN 9
#define EEPROM_LEN 3
#define CUR_BIT(chip) ((chip->buffer[chip->byte_ndx] & (1 << chip->bit_ndx)) != 0)

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

// scratch pad offsets
#define CHIP_SP_TEMP_LOW_OFF    0x00
#define CHIP_SP_TEMP_HI_OFF     0x01
#define CHIP_SP_USER_BYTE_1_OFF 0x02
#define CHIP_SP_USER_BYTE_2_OFF 0x03
#define CHIP_SP_CFG_REG_OFF     0x04
#define CHIP_SP_RSVD_1_OFF      0x05
#define CHIP_SP_RSVD_2_OFF      0x06
#define CHIP_SP_RSVD_3_OFF      0x07
#define CHIP_SP_CRC_OFF         0x08

// EEPROM offsets
#define CHIP_EE_TEMP_LOW_OFF    0x00
#define CHIP_EE_TEMP_HI_OFF     0x01
#define CHIP_EE_CFG_REG_OFF     0x02

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

    // comms buffer
    uint8_t buffer[BUF_LEN];        
    int bit_ndx;
    int byte_ndx;
    bool cur_bit;

    // configurable timing vars
    uint32_t presence_wait_time;
    uint32_t presence_time;

    // the temperature from config
    uint32_t temperature;

    // the Dallas Family Code to use
    uint8_t family_code;

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
static void on_ow_alarm_seach(chip_desc_t *chip);

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
        .max_states = ST_MASTER_SEARCH_MAX,
        .max_events = EV_MAX
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
        .max_states = ST_MASTER_MATCH_MAX,
        .max_events = EV_MAX
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
        .max_states = ST_MASTER_MATCH_MAX,
        .max_events = EV_MAX
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
        .max_states = ST_MASTER_RD_BYTE_MAX,
        .max_events = EV_MAX
};

static sm_t *sm_rd_byte = &(sm_t){.cfg = &sm_rd_byte_cfg};


static sm_cfg_t sm_wr_bit_cfg = {
        .name = "sm_wr_bit",
        .sm_entries = (sm_entry_t[]){
                // ST_MASTER_RD_SP_BYTE_RD
                SM_E(ST_MASTER_RD_BIT_BIT_WRITTEN, EV_BIT_WRITTEN, on_master_rd_bit_bit_written),
        },
        .num_entries = 1,
        .max_states = ST_MASTER_RD_BIT_MAX,
        .max_events = EV_MAX
};

static sm_t *sm_wr_bit = &(sm_t){.cfg = &sm_wr_bit_cfg};


static uint8_t supported_family_codes[] = { 
    DS_FC_18S20, DS_FC_18B20, DS_FC_1822
};

static cmd_entry_t cmd_entries[] = {
    { ST_WAIT_CMD << 8 | OW_CMD_MATCH, on_ow_match },
    { ST_WAIT_CMD << 8 | OW_CMD_SKIP, on_ow_skip },
    { ST_WAIT_CMD << 8 | OW_CMD_SEARCH, on_ow_search },
    { ST_WAIT_CMD << 8 | OW_CMD_ALM_SEARCH, on_ow_alarm_seach },
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
		crc = *(dscrc2x16_table + (crc & 0x0f)) ^ *(dscrc2x16_table + 16 + ((crc >> 4) & 0x0f));
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


const char*debugBinStr(char *p, size_t c) {
    static char buf[200];
    char *pb = buf;
    if (c > 16) {
        c = 16;        
    }

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

// ==================== Implementation =========================
void chip_init()
{
//    setvbuf(stdout, NULL, _IOLBF, 1024);
    printf("*** DS18B20 chip initialising...\n");
    chip_desc_t *chip = calloc(1, sizeof(chip_desc_t));

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
    printf("ow_write_byte_ctx: %p, ow_ctx: %p, wb_ow_ctx: %p\n", chip->ow_write_byte_ctx, ow_ctx, chip->ow_write_byte_ctx->ow_ctx);

    // initialise command map
    cmd_init_hash();

    // read config attributes
    uint32_t attr;
    uint32_t len;
    char dev_id_attr[SERIAL_LEN * 2];
    
    attr = attr_init("ow_debug", false); chip->ow_debug = attr_read(attr) != 0;
    attr = attr_init("gen_debug", false); chip->gen_debug = attr_read(attr) != 0;
    attr = attr_init("debug_timer", false); chip->debug_timer = attr_read(attr) != 0;

    attr = attr_init("temperature", 0); chip->temperature = attr_read(attr);

    // initialise device id
    attr = attr_init("family_code", DS_FC_18S20); chip->serial_no[0] = attr_read(attr) & 0xFF;
    if (memchr(supported_family_codes, chip->serial_no[0], sizeof(supported_family_codes)) == NULL) {
        printf("*** DS18B20 device family code not supported (%d), expect errors...\n", chip->serial_no[0]);
    }

    attr = attr_string_init("device_id"); 
    len = string_read(attr, dev_id_attr, 13 );  // expecting 12 Hex Digits + NULL
    if (len < 12) {
        printf("*** DS18B20 device id too short (%d), expect errors...\n", len);
    }

    for (int i = 0; i < 6; i++) {
        char tmp = dev_id_attr[i * 2 + 2];
        dev_id_attr[i*2+2] = 0;
        chip->serial_no[i + 1] = strtol(dev_id_attr + i * 2, NULL, 16 );
        dev_id_attr[i*2+2] = tmp;
    }

    chip->serial_no[7] = crc8(chip->serial_no, 7);

//    attr = attr_init("presence_wait_time", PR_DUR_WAIT_PRESENCE);
//    chip->presence_wait_time = attr_read(attr);
//    attr = attr_init("presence_time", PR_DUR_PULL_PRESENCE);
//    chip->presence_time = attr_read(attr);
   

    printf("*** DS18B20 setting attributes:\n  gen_debug: %d\n  ow_debug: %d\n  temperature: %d\n", 
    chip->gen_debug, chip->ow_debug, chip->temperature);

    printf("  device_id: ");
    for (int i = 0; i < SERIAL_LEN; i++ ){
        printf("%02x", chip->serial_no[i]);
    }
    printf("\n");

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
        DEBUGF("%08lld %s Initialising hash\n", get_sim_nanos(),sm->cfg->name);
        sm_init_hash(sm);
    }

    uint64_t key = state_event_to_key(state, ev);
    sm_entry_t *e = hashmap_get((sm_entry_map_t *)sm->hash, &key);

//    sm_entry_t e = *(sm->sm_entries + sm->max_events * state + event);

    if (e == NULL || e->handler == NULL) {
        DEBUGF("(%s) SM error: unhandled event %d in state %d (e %p)), resetting\n", sm->cfg->name, ev, state, e);
        chip_reset_state(chip);
        return;
    } else {
        DEBUGF("%08lld %s %s[%s]: %s( %d ) -> %p\n",
               get_sim_nanos(), sm->cfg->name, e->st_name,
               e->ev_name, e->name, ev_data, e->handler);
        e->handler(chip, ev_data);
    }


    key = state_event_to_key(chip->cmd_ctx.state, ev);
    e = hashmap_get((sm_entry_map_t *)sm->hash, &key);
    const char *n = e != NULL ? e->st_name : "invalid state";
    DEBUGF("(%s) new state -> %s\n", sm->cfg->name, n);
}


// ==================== API handlers =========================
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
        DEBUGF("on_bit_read_cb: calling bit_callback %p\n", chip->ow_write_byte_ctx);
        chip->ow_read_byte_ctx->bit_callback(chip->ow_read_byte_ctx, err, data);
    } else {
        // current command is directly handling bits, run its state machine
        DEBUGF("on_bit_read_cb: calling own sm\n");
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
    chip->cur_bit = CUR_BIT(chip);

    ow_ctx_set_master_read_state(chip->ow_ctx, chip->cur_bit);
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


    if (chip->cmd_ctx.cmd_data.wr_sp_ctx.byte_ndx == 0) {
        chip->scratch_pad[CHIP_SP_TEMP_HI_OFF] = data & 0xFF;
        DEBUGF("on_master_wr_sp_byte_read: writing TH byte %02x\n", data);
    } else if (chip->cmd_ctx.cmd_data.wr_sp_ctx.byte_ndx == 1 ) {
        chip->scratch_pad[CHIP_SP_TEMP_LOW_OFF] = data & 0xFF;
        DEBUGF("on_master_wr_sp_byte_read: writing TL byte %02x\n", data);
    } if (chip->cmd_ctx.cmd_data.wr_sp_ctx.byte_ndx == 2 ) {
        chip->scratch_pad[CHIP_SP_CFG_REG_OFF] = data & 0xFF;
        DEBUGF("on_master_wr_sp_byte_read: writing CFG byte %02x\n", data);
    }

    chip->cmd_ctx.cmd_data.wr_sp_ctx.byte_ndx++;
    if (chip->cmd_ctx.cmd_data.wr_sp_ctx.byte_ndx >= 3) {

        update_crc8(chip);

        DEBUGF("on_master_wr_sp_byte_read: *** finished, starting next cycle\n");
        for (int i = 0;i < SCRATCH_LEN; i++)
            DEBUGF("on_master_wr_sp_byte_read: byte %d: %02x\n", i, chip->scratch_pad[i]);  
        chip_reset_state(chip);
    }
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
    chip->cur_bit = CUR_BIT(chip);

    ow_ctx_set_master_write_state(chip->ow_ctx, chip->cur_bit);
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
static void on_rom_command(chip_desc_t *chip, uint8_t cmd) {
    DEBUGF("on_rom_command %2X\n", cmd);
    uint16_t key = cmd_to_key(ST_WAIT_CMD, cmd);
    cmd_entry_t *e = hashmap_get(cmd_map, &key);
    if (e != NULL) {
        e->handler(chip);
    } else {
        DEBUGF("**** rom command %02x not implemented\n", cmd);
        chip_reset_state(chip);
    }

}

static void on_func_command(chip_desc_t *chip, uint8_t cmd) {
    DEBUGF("on_func_command %2X\n", cmd);
    uint16_t key = cmd_to_key(ST_WAIT_FN_CMD, cmd);
    cmd_entry_t *e = hashmap_get(cmd_map, &key);
    if (e != NULL) {
        e->handler(chip);
        return;   
    }
    
    DEBUGF("**** func command %02x not implemented\n", cmd);
    chip_reset_state(chip);
}



// master is searching for us. we need to report our id
// we're expecting master to initiate read bit
static void on_ow_search(chip_desc_t *chip) {
    DEBUGF("on_ow_search\n");
    memcpy(chip->buffer, chip->serial_no, SERIAL_LEN);
    DEBUGF("on_ow_search %s\n", debugBinStr((char *)chip->serial_no, SERIAL_LEN));
    DEBUGF("on_ow_search b1 %s\n", debugBinStr((char *)chip->buffer, SERIAL_LEN));
    chip->sig_mode = ST_SIG_BIT_MODE;
    chip->cmd_ctx.state = ST_MASTER_SEARCH_WRITE_BIT;
    chip->cmd_ctx.cmd_sm = sm_search;
    DEBUGF("on_ow_search b2 %s\n", debugBinStr((char *)chip->buffer, SERIAL_LEN));
    chip->bit_ndx = 0;
    chip->byte_ndx = 0;
    DEBUGF("on_ow_search b2a %s\n", debugBinStr((char *)chip->buffer, SERIAL_LEN));
    chip->cur_bit = CUR_BIT(chip);
    DEBUGF("on_ow_search b2b %s\n", debugBinStr((char *)chip->buffer, SERIAL_LEN));
    // chip->resp_len = SERIAL_LEN;
    DEBUGF("on_ow_search %s\n", debugBinStr((char *)chip->serial_no, SERIAL_LEN));
    DEBUGF("on_ow_search b3 %s\n", debugBinStr((char *)chip->buffer, SERIAL_LEN));

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
    chip->cur_bit = CUR_BIT(chip);
    // chip->resp_len = SERIAL_LEN;

    match_start_prime_next_bit(chip);
    DEBUGF("on_ow_match started\n");
}

static void on_ow_skip(chip_desc_t *chip) {
    DEBUGF("on_ow_skip\n");
    chip_ready_for_next_func_cmd(chip);
}

static void on_ow_alarm_seach(chip_desc_t *chip) {

}

static void on_ds_convert(chip_desc_t *chip) {

    // write our temp into the scratch pad. Assume we're in parasitic mode for now
    chip->scratch_pad[CHIP_SP_TEMP_HI_OFF] = (chip->temperature >> 8) & 0xFF;
    chip->scratch_pad[CHIP_SP_TEMP_LOW_OFF] = (chip->temperature) & 0xFF;

    update_crc8(chip);

    chip_reset_state(chip);

}

static void on_ds_write_scratchpad(chip_desc_t *chip) {
    DEBUGF("on_ds_write_scratchpad\n");
    chip->state = ST_EXEC_CMD;
    chip->cmd_ctx.state = ST_MASTER_WR_SP_BYTE_READ;
    chip->cmd_ctx.cmd_sm = sm_wr_sp;
    chip->bit_ndx = 0;
    chip->byte_ndx = 0;
    // chip->resp_len = SERIAL_LEN;

    ow_ctx_set_master_write_state(chip->ow_ctx, false);
    ow_read_byte_ctx_reset_state(chip->ow_read_byte_ctx);
}

static void on_ds_read_scratchpad(chip_desc_t *chip) {
    DEBUGF("on_ds_read_scratchpad\n");
    memcpy(chip->buffer, chip->scratch_pad, SCRATCH_LEN);
    chip->state = ST_EXEC_CMD;
    chip->cmd_ctx.state = ST_MASTER_RD_BYTE_BYTE_WRITTEN;
    chip->cmd_ctx.cmd_sm = sm_rd_byte;
    chip->cmd_ctx.cmd_data.rd_byte_ctx.restart_when_done = true;
    chip->cmd_ctx.cmd_data.rd_byte_ctx.resp_len = SCRATCH_LEN;
    chip->bit_ndx = 0;
    chip->byte_ndx = 0;
    // chip->resp_len = SERIAL_LEN;

    for (int i = 0; i<SCRATCH_LEN; i++) {
        DEBUGF("on_ds_read_scratchpad: byte %d: %02x\n", i, chip->buffer[i]);
    }
    on_master_rd_byte_prime_next_byte(chip, 0);
}

static void on_ds_copy_scratchpad(chip_desc_t *chip) {
    // write our temp into the scratch pad. Assume we're in parasitic mode for now
    chip->eeprom[CHIP_EE_TEMP_HI_OFF] = chip->scratch_pad[CHIP_SP_TEMP_HI_OFF];
    chip->eeprom[CHIP_EE_TEMP_LOW_OFF] = chip->scratch_pad[CHIP_SP_TEMP_LOW_OFF];
    chip->eeprom[CHIP_EE_CFG_REG_OFF] = chip->scratch_pad[CHIP_SP_CFG_REG_OFF];

    // todo(bonnyr): if power mode is powered we need to write '1' to master. For now assume parasite power

    chip_reset_state(chip);
}

static void on_ds_recall(chip_desc_t *chip) {
    DEBUGF("on_ds_recall\n");
    chip->scratch_pad[CHIP_SP_TEMP_HI_OFF] = chip->eeprom[CHIP_EE_TEMP_HI_OFF];
    chip->scratch_pad[CHIP_SP_TEMP_LOW_OFF] = chip->eeprom[CHIP_EE_TEMP_LOW_OFF];
    chip->scratch_pad[CHIP_SP_CFG_REG_OFF] = chip->eeprom[CHIP_EE_CFG_REG_OFF];

    update_crc8(chip);

    chip->sig_mode = ST_SIG_BIT_MODE;
    chip->state = ST_EXEC_CMD;
    chip->cmd_ctx.state = ST_MASTER_RD_BIT_BIT_WRITTEN;
    chip->cmd_ctx.cmd_sm = sm_wr_bit;
    chip->bit_ndx = 0;
    chip->byte_ndx = 0;
    chip->buffer[0] = 0x01; // only write 1 bit of '1'

    ds_func_cmd_prime_next_bit(chip);
    DEBUGF("on_ds_recall started\n");

}

static void on_ds_read_power(chip_desc_t *chip) {
    chip->sig_mode = ST_SIG_BIT_MODE;
    chip->state = ST_EXEC_CMD;
    chip->cmd_ctx.state = ST_MASTER_RD_BIT_BIT_WRITTEN;
    chip->cmd_ctx.cmd_sm = sm_wr_bit;
    chip->bit_ndx = 0;
    chip->byte_ndx = 0;
    chip->buffer[0] = 0x00; // indicate we're running on parasite power

    ds_func_cmd_prime_next_bit(chip);
    DEBUGF("on_ds_read_power\n");
}
