// Wokwi Custom Chip - For information and examples see:
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

#define IN_RANGE(x, y, err) ( (y) > (x) ? (y) - (x) <= err : (x) - (y) <= err)


// buffers
#define BUF_LEN 32
#define SERIAL_LEN 8
#define SCRATCH_LEN 8
#define CUR_BIT(chip) ((chip->buffer[chip->byte_ndx] &= 1 << chip->bit_ndx) != 0)


// rom commands
#define OW_CMD_SEARCH           0xF0
#define OW_CMD_READ             0x33
#define OW_CMD_MATCH            0x55
#define OW_CMD_SKIP             0xCC
#define OW_CMD_ALM_SEARCH       0xEC
#define OW_CMD_CONVERT          0x44
#define OW_CMD_WR_SCRATCH       0x4E
#define OW_CMD_RD_SCRATCH       0xBE
#define OW_CMD_CP_SCRATCH       0x48
#define OW_CMD_RECALL           0xB8
#define OW_CMD_RD_PWD           0xB4


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

typedef enum {
    ST_SIG_BIT_MODE,
    ST_SIG_BYTE_MODE,
} sig_mode_t;

typedef enum {
    ST_INIT_SEQ,
    ST_WAIT_CMD,
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
    ST_MASTER_RD_SP_BYTE_WRITTEN,
    ST_MASTER_RD_SP_MAX
} wr_rd_state_t;

typedef struct  {
    uint8_t bit_ndx;
} cmd_search_ctx_t;

typedef struct  {
} cmd_match_ctx_t;

typedef struct  {
    int byte_ndx;
} cmd_sp_ctx_t;

typedef struct {
    uint32_t state;
    sm_t *cmd_sm;
    union {
        cmd_search_ctx_t search_ctx;
        cmd_match_ctx_t match_ctx;
        cmd_sp_ctx_t wr_sp_ctx;
        cmd_sp_ctx_t rd_sp_ctx;
    } cmd_data;
} cmd_ctx_t;

typedef struct
{
    uint8_t serial_no[SERIAL_LEN];
    uint8_t scratch_pad[SCRATCH_LEN];

    // onw wire helpers
    ow_ctx_t *ow_ctx;
    ow_byte_ctx_t *ow_write_byte_ctx;
    ow_byte_ctx_t *ow_read_byte_ctx;

    chip_state_t state;
    sig_mode_t sig_mode;
    cmd_ctx_t cmd_ctx;
    bool skip;

    // comms buffer
    uint8_t buffer[BUF_LEN];
    int bit_ndx;
    int byte_ndx;
    bool cur_bit;
    int resp_len;

    // configurable timing vars
    uint32_t presence_wait_time;
    uint32_t presence_time;

    // debug
    bool debug_timer;
    bool ow_debug;
    bool gen_debug;

} chip_desc_t;

// ==================== forward decls =========================
static void chip_reset_state(chip_desc_t *chip);


// search command handlers
//static void on_not_impl(chip_desc_t *chip, uint32_t data);

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
static void on_master_rd_sp_byte_written(void *user_data, uint32_t data);


// --- command handlers 
static void on_rom_command(chip_desc_t *chip, uint8_t cmd);
static void on_ow_reset(chip_desc_t *chip);
static void on_ow_search(chip_desc_t *chip);
static void on_ow_read(chip_desc_t *chip);
static void on_ow_match(chip_desc_t *chip);
static void on_ow_skip(chip_desc_t *chip);
static void on_ow_alarm_seach(chip_desc_t *chip);
static void on_ow_convert(chip_desc_t *chip);
static void on_ow_write_scratchpad(chip_desc_t *chip);
static void on_ow_read_scratchpad(chip_desc_t *chip);
static void on_ow_copy(chip_desc_t *chip);
static void on_ow_recall(chip_desc_t *chip);
static void on_ow_read_power(chip_desc_t *chip);



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
        .name = "sm_match",
        .sm_entries = (sm_entry_t[]){ //[ST_MASTER_MATCH_MAX][EV_MASTER_MATCH_MAX] = {
                // ST_MASTER_MATCH_READ_BIT
                SM_E(ST_MASTER_WR_SP_BYTE_READ, EV_BYTE_WRITTEN, on_master_wr_sp_byte_read),
        },
        .num_entries = 2,
        .max_states = ST_MASTER_MATCH_MAX,
        .max_events = EV_MAX
};

static sm_t *sm_wr_sp = &(sm_t){.cfg = &sm_wr_sp_cfg};


// ==== Write Scratchpad SM ====

static sm_cfg_t sm_rd_sp_cfg = {
        .name = "sm_rd_sp",
        .sm_entries = (sm_entry_t[]){
                // ST_MASTER_RD_SP_BYTE_RD
                SM_E(ST_MASTER_RD_SP_BYTE_WRITTEN, EV_BYTE_WRITTEN, on_master_rd_sp_byte_written),
        },
        .num_entries = 2,
        .max_states = ST_MASTER_MATCH_MAX,
        .max_events = EV_MAX
};

static sm_t *sm_rd_sp = &(sm_t){.cfg = &sm_rd_sp_cfg};

// ==================== Implementation =========================
void chip_init()
{
//    setvbuf(stdout, NULL, _IOLBF, 1024);
    printf("DS18B20 chip initialising...\n");
    chip_desc_t *chip = calloc(1, sizeof(chip_desc_t));

    ow_ctx_cfg_t cfg = {
            .bit_written_cb = on_bit_written_cb,
            .bit_read_cb = on_bit_read_cb,
            .reset_cb = on_reset_cb,
            .pin_name = "DQ",
            .data = chip,
    };
    ow_ctx_t *ctx = ow_ctx_init(&cfg);


    chip->ow_ctx = ctx;
    chip->ow_read_byte_ctx = ow_read_byte_ctx_init(chip, on_byte_read_cb, ctx);
    chip->ow_write_byte_ctx = ow_write_byte_ctx_init(chip, on_byte_written_cb, ctx);

    // read config attributes
    uint32_t attr;
    
    attr = attr_init("ow_debug", false); chip->ow_debug = attr_read(attr) != 0;
    attr = attr_init("gen_debug", false); chip->gen_debug = attr_read(attr) != 0;
    attr = attr_init("debug_timer", false); chip->debug_timer = attr_read(attr) != 0;

//    attr = attr_init("presence_wait_time", PR_DUR_WAIT_PRESENCE);
//    chip->presence_wait_time = attr_read(attr);
//    attr = attr_init("presence_time", PR_DUR_PULL_PRESENCE);
//    chip->presence_time = attr_read(attr);

    
    // attr = attr_init("device_id", "9F9D876799C4F707"); 
    const char *dev_id_str = "9F9D876799C4F707";//= attr_read(attr);
    uint64_t tmp = strtoull(dev_id_str, NULL, 16);
    printf("serial: %llx\n", tmp);

    // this is a hack. just until I can get string attributes...
    *(uint64_t *)chip->serial_no = tmp;
    for (int i = 0; i < 4; i++ ){
        uint8_t tmp = chip->serial_no[i];
        chip->serial_no[i] = chip->serial_no[7-i];
        chip->serial_no[7-i] = tmp;
    }
    for (int i = 0; i < 8; i++ ){
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
    chip->skip = false;
    memset(&chip->cmd_ctx, 0, sizeof(cmd_ctx_t));

    chip->bit_ndx = 0;
    chip->byte_ndx = 0;
    chip->resp_len = 0;
    memset(chip->buffer, 0, BUF_LEN);

}

static void chip_ready_for_next_cmd(chip_desc_t *chip) {
    DEBUGF("readying chip state for next command\n");
    ow_ctx_set_master_write_state(chip->ow_ctx, false);

    chip->sig_mode = ST_SIG_BYTE_MODE;
    chip->state = ST_WAIT_CMD;
    memset(&chip->cmd_ctx, 0, sizeof(cmd_ctx_t));

    chip->bit_ndx = 0;
    chip->byte_ndx = 0;
    chip->resp_len = 0;
    memset(chip->buffer, 0, BUF_LEN);

}


// ==================== API handlers =========================
void push_cmd_sm_event(chip_desc_t *chip, sm_t *sm, uint32_t state, uint32_t ev, uint32_t ev_data) {

    if (!sm->hash) {
        DEBUGF("%08lld %s Initialising hash\n", get_sim_nanos(),sm->cfg->name);
        sm_init_hash(sm);
        const uint64_t *k;
        const sm_entry_t *e;
        hashmap_foreach(k, e, ((sm_entry_map_t *)sm->hash)) {
            DEBUGF("%08lld %s %llx => %s(%d) [%s(%d)]: %s( %d ) -> %p\n",
                   get_sim_nanos(), sm->cfg->name, *k, e->st_name,
                   e->state, e->ev_name, e->event, e->name, ev_data, e->handler);
        }
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
//static void on_not_impl(chip_desc_t *chip, uint32_t data) {
//    printf("not implemented\n");
//}


void on_reset_cb(void *d, uint32_t err, uint32_t data) {
    chip_desc_t *chip = d;

    DEBUGF("on_reset_cb\n");


    // reset is done, now wait for master to write command byte, defer to byte SM
    chip_ready_for_next_cmd(chip);
}

// callback used when a bit has been written to the master via the signalling SM
void on_bit_written_cb(void *d, uint32_t err, uint32_t data) {
    chip_desc_t *chip = d;

    DEBUGF("on_bit_written_cb: %d\n", data);
    if (chip->sig_mode == ST_SIG_BYTE_MODE) {
        DEBUGF("on_bit_written_cb: calling bit_callback %p\n", chip->ow_read_byte_ctx);
        chip->ow_write_byte_ctx->bit_callback(chip->ow_write_byte_ctx, err, data);
    } else {
        // current command is directly handling bits, run its state machine
        DEBUGF("on_bit_written_cb: calling own sm\n");
        push_cmd_sm_event(chip, chip->cmd_ctx.cmd_sm, chip->cmd_ctx.state, EV_BIT_WRITTEN, data);
    }
}


// callback used when a bit has been read from the master via the signalling SM
void on_bit_read_cb(void *d, uint32_t err, uint32_t data) {
    chip_desc_t *chip = d;

    DEBUGF("on_bit_read_cb\n");
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


void on_byte_read_cb(void *d, uint32_t err, uint32_t data) {
    chip_desc_t *chip = d;

    DEBUGF("on_master_byte_read_cb\n");
    // if we're waiting on command code, we handle directly, otherwise the byte is passed the current command
    if (chip->sig_mode == ST_SIG_BYTE_MODE && chip->state == ST_WAIT_CMD) {
        DEBUGF("on_master_byte_read_cb - processing command %02X\n", data);
        on_rom_command(chip, data);
        return;
    }

    // todo(bonnyr): only handle if we're in the middle of command
    push_cmd_sm_event(chip, chip->cmd_ctx.cmd_sm, chip->cmd_ctx.state, EV_BYTE_READ, data);
}


void on_byte_written_cb(void *d, uint32_t err, uint32_t data) {
    chip_desc_t *chip = d;
    DEBUGF("on_byte_written_cb\n");
    push_cmd_sm_event(chip, chip->cmd_ctx.cmd_sm, chip->cmd_ctx.state, EV_BYTE_WRITTEN, data);
}


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


static void search_start_next_bit(chip_desc_t *chip) {
    chip->cur_bit = CUR_BIT(chip);

    ow_ctx_set_master_read_state(chip->ow_ctx, chip->cur_bit);
    DEBUGF("search_start_next_bit: ow_ctx: %p, ow_ctx->state: %d\n", chip->ow_ctx, chip->ow_ctx->state);
}

static void on_master_search_error(void *user_data, uint32_t data) {
    chip_desc_t *chip = user_data;

    DEBUGF("on_master_search_error: \n");
    chip_reset_state(chip);
}

static void on_master_search_bit_written(void *user_data, uint32_t data) {
    chip_desc_t *chip = user_data;
    DEBUGF("on_master_search_bit_written: %d\n", data);

    chip->cmd_ctx.state = ST_MASTER_SEARCH_WRITE_INV_BIT;

    // todo(bonnyr): refactor to a function in signalling
    ow_ctx_set_master_read_state(chip->ow_ctx, !CUR_BIT(chip));
}

static void on_master_search_inv_bit_written(void *user_data, uint32_t data) {
    chip_desc_t *chip = user_data;
    DEBUGF("on_master_search_inv_bit_written: %d\n", data);

    chip->cmd_ctx.state = ST_MASTER_SEARCH_READ_BIT;
    // todo(bonnyr): refactor to a function in signalling
    ow_ctx_set_master_write_state(chip->ow_ctx, chip->cur_bit);
}


static void on_master_search_bit_read(void *user_data, uint32_t data) {
    chip_desc_t *chip = user_data;

    DEBUGF("on_master_search_bit_read: comparing bit %d - m:%d, s:%d\n", chip->byte_ndx * 8 + chip->bit_ndx, data, CUR_BIT(chip))
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
    search_start_next_bit(chip);
    chip->cmd_ctx.state = ST_MASTER_SEARCH_WRITE_BIT;
}






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
    if (chip->byte_ndx == 3) {
        DEBUGF("on_master_wr_sp_byte_read: *** finished, starting next cycle\n");
        chip_reset_state(chip);
        return;
    }
}


static void on_master_rd_sp_byte_written(void *user_data, uint32_t data) {
    chip_desc_t *chip = user_data;


    if (chip->cmd_ctx.cmd_data.rd_sp_ctx.byte_ndx == 0) {
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
    if (chip->byte_ndx == 3) {
        DEBUGF("on_master_wr_sp_byte_read: *** finished, starting next cycle\n");
        chip_reset_state(chip);
        return;
    }
}







static void on_master_match_error(void *user_data, uint32_t data) {
    chip_desc_t *chip = user_data;

    DEBUGF("on_master_match_error: \n");
    chip_reset_state(chip);
}

static void on_master_match_bit_read(void *user_data, uint32_t data) {
    chip_desc_t *chip = user_data;

    DEBUGF("on_master_match_error: comparing bit %d - m:%d, s:%d\n", chip->byte_ndx * 8 + chip->bit_ndx, data, CUR_BIT(chip))
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
        DEBUGF("on_master_match_error: *** finished search, waiting for function command\n");
        chip_ready_for_next_cmd(chip);
        chip->skip = true;
        return;
    }

    // go for more
    search_start_next_bit(chip);
    chip->cmd_ctx.state = ST_MASTER_SEARCH_WRITE_BIT;
}


// ==================== Logic Implementation =========================
static void on_rom_command(chip_desc_t *chip, uint8_t cmd) {
    DEBUGF("on_rom_command %2X\n", cmd);

    if (!chip->skip) {
        switch(cmd) {
            case OW_CMD_SEARCH:
                on_ow_search(chip);
                return;
            case OW_CMD_READ:
                on_ow_read(chip);
                return;
            case OW_CMD_MATCH:
                on_ow_match(chip);
                return;
            case OW_CMD_SKIP:
                on_ow_skip(chip);
                return;
//            case OW_CMD_ALM_SEARCH:
            default:
                break;
        }

        DEBUGF("**** command %02x not implemented\n", cmd);
        chip_reset_state(chip);
        return;
    }

    // reset skip mode
    chip->skip = false;
    switch (cmd) {
        case OW_CMD_CONVERT:
            on_ow_convert(chip);
            return;
        case OW_CMD_WR_SCRATCH:
            on_ow_write_scratchpad(chip);
            return;
        case OW_CMD_RD_SCRATCH:
        case OW_CMD_CP_SCRATCH:
        case OW_CMD_RECALL:
        case OW_CMD_RD_PWD:
        default:
        DEBUGF("**** command %02x not implemented\n", cmd);
            chip_reset_state(chip);
            break;
    }
}



static void on_ow_reset(chip_desc_t *chip) {

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
    chip->cur_bit = CUR_BIT(chip);
    chip->resp_len = SERIAL_LEN;

    search_start_next_bit(chip);
    DEBUGF("on_ow_search started\n");
}
static void on_ow_read(chip_desc_t *chip) {
    DEBUGF("on_ow_read\n");
//    memcpy(chip->buffer, chip->serial_no, SERIAL_LEN);
//    chip->state = ST_MASTER_READ_INIT;
//    chip->bit_ndx = 0;
//    chip->byte_ndx = 0;
//    chip->resp_len = SERIAL_LEN;
}
static void on_ow_match(chip_desc_t *chip) {
    DEBUGF("on_ow_match\n");
    memcpy(chip->buffer, chip->serial_no, SERIAL_LEN);
    chip->sig_mode = ST_SIG_BIT_MODE;
    chip->cmd_ctx.state = ST_MASTER_MATCH_READ_BIT;
    chip->cmd_ctx.cmd_sm = sm_search;
    chip->bit_ndx = 0;
    chip->byte_ndx = 0;
    chip->cur_bit = CUR_BIT(chip);
    chip->resp_len = SERIAL_LEN;

    search_start_next_bit(chip);
    DEBUGF("on_ow_search started\n");
}
static void on_ow_skip(chip_desc_t *chip) {
    DEBUGF("on_ow_skip\n");
    chip->skip = true;
    chip_ready_for_next_cmd(chip);

}
static void on_ow_alarm_seach(chip_desc_t *chip) {

}
static void on_ow_convert(chip_desc_t *chip) {

}
static void on_ow_write_scratchpad(chip_desc_t *chip) {
    memcpy(chip->buffer, chip->serial_no, SERIAL_LEN);
    chip->state = ST_EXEC_CMD;
    chip->cmd_ctx.state = ST_MASTER_WR_SP_BYTE_READ;
    chip->cmd_ctx.cmd_sm = sm_search;
    chip->bit_ndx = 0;
    chip->byte_ndx = 0;
    chip->resp_len = SERIAL_LEN;

    ow_read_byte_ctx_reset_state(chip->ow_read_byte_ctx);
}
static void on_ow_read_scratchpad(chip_desc_t *chip) {

}
static void on_ow_copy(chip_desc_t *chip) {

}
static void on_ow_recall(chip_desc_t *chip) {

}
static void on_ow_read_power(chip_desc_t *chip) {

}
