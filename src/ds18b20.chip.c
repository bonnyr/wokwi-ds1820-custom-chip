// Wokwi Custom Chip - For information and examples see:
// https://link.wokwi.com/custom-chips-alpha
//
// SPDX-License-Identifier: MIT
// Copyright (C) 2022 Bonny Rais

#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#define DEBUG 1

// --------------- Debug Macros -----------------------
#ifdef DEBUG
#define DEBUGF(...)      { if (chip->gen_debug) {printf(__VA_ARGS__);} }
#define OW_DEBUGF(...)  { if (chip->ow_debug) {printf(__VA_ARGS__);}  }
#endif //  DEBUG
// --------------- Debug Macros -----------------------

#define MICROS_IN_SEC 1000000
#define SIG_TXN_DUR_NS 1000    // assume it takes 1us to stabilise the signal
#define IN_RANGE(x, y, err) ( (y) > (x) ? (y) - (x) <= err : (x) - (y) <= err)


// buffers
#define BUF_LEN 32
#define SERIAL_LEN 8
#define CUR_BIT(chip) ((chip->buffer[chip->byte_ndx] &= 1 << chip->bit_ndx) != 0)



// protocol definitions of various durations
#define PR_DUR_RESET 480
#define PR_DUR_WAIT_PRESENCE_L 15
#define PR_DUR_WAIT_PRESENCE_H 60
#define PR_DUR_WAIT_PRESENCE 30
#define PR_DUR_PULL_PRESENCE_L 60
#define PR_DUR_PULL_PRESENCE_H 240
#define PR_DUR_PULL_PRESENCE 120
#define PR_DUR_SAMPLE_WAIT 15
#define PR_DUR_WRITE_SLOT_END 45
#define PR_DUR_READ_SLOT 15
#define PR_DUR_READ_SLOT_END 45
#define PR_DUR_READ_INIT 1

// allow 2us of jitter 
#define PR_DUR_BUS_JITTER 2000

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

typedef enum {
    ST_INIT,
    ST_WAIT_RESET,
    ST_WAIT_PRESENCE,
    ST_PULL_PRESENCE,

    ST_MASTER_WRITE_INIT,
    ST_MASTER_WRITE_WAIT_RELEASE,
    ST_MASTER_WRITE_SLOT_END,
    ST_MASTER_READ_INIT,
    ST_MASTER_READ_INIT_WAIT_RELEASE,
    ST_MASTER_READ_SLOT_TIMER,

    ST_MASTER_SEARCH_BIT_INIT,
    ST_MASTER_SEARCH_BIT_WAIT_RELEASE,
    ST_MASTER_SEARCH_BIT_SLOT_TIMER,
    ST_MASTER_SEARCH_BIT_SLOT_END,
    ST_MASTER_SEARCH_INV_BIT_INIT,
    ST_MASTER_SEARCH_INV_BIT_WAIT_RELEASE,
    ST_MASTER_SEARCH_INV_BIT_SLOT_TIMER,
    ST_MASTER_SEARCH_INV_BIT_SLOT_END,
    ST_MASTER_SEARCH_TX_BIT_INIT,
    ST_MASTER_SEARCH_TX_BIT_WAIT_RELEASE,
    ST_MASTER_SEARCH_TX_BIT_SLOT_END,

    ST_MAX

} ow_state_t;

typedef enum {
    ST_WAIT_CMD,
    ST_READ_REQ_DATA,
    ST_WRITE_RESP_DATA,
} ow_prot_state_t;

typedef enum {
  EV_PIN_CHG,
  EV_TIMER_EXPIRED,

  EV_MAX
} ev_t;

typedef struct
{
    uint8_t serial_no[SERIAL_LEN];
    timer_t timer;
    timer_t reset_timer;
    pin_t pin;
    ow_state_t state;
    ow_prot_state_t prot_state;


    // receive buffer
    uint8_t buffer[BUF_LEN];
    int bit_ndx;
    int byte_ndx;
    bool cur_bit;
    int resp_len;

    // various state helpers
    uint64_t reset_time;
    bool reset_timer_expired;

    uint64_t read_slot_start;
    uint64_t write_slot_start;
    
    // configurable timing vars
    uint32_t presence_wait_time;
    uint32_t presence_time;

    // debug
    bool debug_timer;
    bool ow_debug;
    bool gen_debug;

} chip_desc_t;

typedef void (*sm_handler)(chip_desc_t *chip, uint32_t data);
typedef struct {
    const char *name;
    const char *st_name;
    const char *ev_name;
    sm_handler handler;
} sm_entry_t;

#define ST_NAME() (sm[chip->state][0].st_name)
#define SM_E(s, e, h) (sm_entry_t){.name = #h, .st_name = #s, .ev_name = #e, .handler = h}

// ==================== forward decls =========================
static void reset_state(chip_desc_t *chip);

static void on_reset_timer_event(void *chip);
static void on_timer_event(void *data);
static void on_pin_change(void *user_data, pin_t pin, uint32_t value);

static void on_not_impl(chip_desc_t *chip, uint32_t data);

static void on_init_pin_chg(chip_desc_t *chip, uint32_t data);
static void on_reset_pin_chg(chip_desc_t *chip, uint32_t data);
static void on_reset_timer_expired(chip_desc_t *chip, uint32_t data);
static void on_wait_presence_pin_chg(chip_desc_t *chip, uint32_t data);
static void on_wait_presence_timer_expired(chip_desc_t *chip, uint32_t data);
static void on_presence_pin_chg(chip_desc_t *chip, uint32_t data);
static void on_presence_timer_expired(chip_desc_t *chip, uint32_t data);
static void on_master_write_init_pin_chg(chip_desc_t *chip, uint32_t data);
static void on_master_write_pin_chg(chip_desc_t *chip, uint32_t data);
static void on_master_write_timer_expired(chip_desc_t *chip, uint32_t data);
static void on_master_write_slot_end_pin_chg(chip_desc_t *chip, uint32_t data);
static void on_master_read_init_pin_chg(chip_desc_t *chip, uint32_t data);
static void on_master_read_init_wait_pin_chg(chip_desc_t *chip, uint32_t data);
static void on_master_read_init_wait_timer_expired(chip_desc_t *chip, uint32_t data);
static void on_master_read_slot_pin_chg(chip_desc_t *chip, uint32_t data);
static void on_master_read_slot_timer_expired(chip_desc_t *chip, uint32_t data);



// search command handlers
static void on_master_search_bit_init_pin_chg(chip_desc_t *chip, uint32_t data);
static void on_master_search_bit_init_wait_pin_chg(chip_desc_t *chip, uint32_t data);
static void on_master_search_bit_init_wait_timer_expired(chip_desc_t *chip, uint32_t data);
static void on_master_search_bit_slot_pin_chg(chip_desc_t *chip, uint32_t data);
static void on_master_search_bit_slot_timer_expired(chip_desc_t *chip, uint32_t data);
static void on_master_search_bit_slot_wait_end_pin_chg(chip_desc_t *chip, uint32_t data);
static void on_master_search_bit_slot_wait_end_timer_expired(chip_desc_t *chip, uint32_t data);
static void on_master_search_inv_bit_init_pin_chg(chip_desc_t *chip, uint32_t data);
static void on_master_search_inv_bit_init_wait_pin_chg(chip_desc_t *chip, uint32_t data);
static void on_master_search_inv_bit_init_wait_timer_expired(chip_desc_t *chip, uint32_t data);
static void on_master_search_inv_bit_slot_pin_chg(chip_desc_t *chip, uint32_t data);
static void on_master_search_inv_bit_slot_timer_expired(chip_desc_t *chip, uint32_t data);
static void on_master_search_inv_bit_slot_wait_end_pin_chg(chip_desc_t *chip, uint32_t data);
static void on_master_search_inv_bit_slot_wait_end_timer_expired(chip_desc_t *chip, uint32_t data);
static void on_master_search_tx_bit_init_pin_chg(chip_desc_t *chip, uint32_t data);
static void on_master_search_tx_bit_wait_pin_chg(chip_desc_t *chip, uint32_t data);
static void on_master_search_tx_bit_timer_expired(chip_desc_t *chip, uint32_t data);
static void on_master_search_tx_bit_slot_end_pin_chg(chip_desc_t *chip, uint32_t data);
static void on_master_search_tx_bit_slot_end_timer_expired(chip_desc_t *chip, uint32_t data);


// --- command handlers 
static void on_rom_command(chip_desc_t *chip);
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





static sm_entry_t sm[ST_MAX][EV_MAX] = {
    // ST_INIT
    SM_E(ST_INIT, EV_PIN_CHG, on_init_pin_chg),
    SM_E(ST_INIT, EV_TIMER_EXPIRED, on_not_impl),

    // ST_WAIT_RESET
    SM_E(ST_WAIT_RESET, EV_PIN_CHG, on_reset_pin_chg),
    SM_E(ST_WAIT_RESET, EV_TIMER_EXPIRED, on_reset_timer_expired), 

    // ST_WAIT_PRESENCE
    SM_E(ST_WAIT_PRESENCE, EV_PIN_CHG, on_wait_presence_pin_chg),
    SM_E(ST_WAIT_PRESENCE, EV_TIMER_EXPIRED, on_wait_presence_timer_expired), 

    // ST_PULL_PRESENCE
    SM_E(ST_PULL_PRESENCE, EV_PIN_CHG, on_presence_pin_chg),
    SM_E(ST_PULL_PRESENCE, EV_TIMER_EXPIRED, on_presence_timer_expired), 

    // ST_MASTER_WRITE_INIT
    SM_E(ST_MASTER_WRITE_INIT, EV_PIN_CHG, on_master_write_init_pin_chg),
    SM_E(ST_MASTER_WRITE_INIT, EV_TIMER_EXPIRED, on_not_impl),
    
    // ST_MASTER_WRITE_WAIT_RELEASE
    SM_E(ST_MASTER_WRITE_WAIT_RELEASE, EV_PIN_CHG, on_master_write_pin_chg),
    SM_E(ST_MASTER_WRITE_WAIT_RELEASE, EV_TIMER_EXPIRED, on_master_write_timer_expired), 

    // ST_MASTER_WRITE_SLOT_END
    SM_E(ST_MASTER_WRITE_SLOT_END, EV_PIN_CHG, on_master_write_slot_end_pin_chg),
    SM_E(ST_MASTER_WRITE_SLOT_END, EV_TIMER_EXPIRED, on_not_impl),
    
    // ST_MASTER_READ_INIT
    SM_E(ST_MASTER_READ_INIT, EV_PIN_CHG, on_master_read_init_pin_chg),
    SM_E(ST_MASTER_READ_INIT, EV_TIMER_EXPIRED, on_not_impl),

    // ST_MASTER_READ_INIT_WAIT_RELEASE
    SM_E(ST_MASTER_READ_INIT_WAIT_RELEASE, EV_PIN_CHG, on_master_read_init_wait_pin_chg),
    SM_E(ST_MASTER_READ_INIT_WAIT_RELEASE, EV_TIMER_EXPIRED, on_master_read_init_wait_timer_expired), 

    // ST_MASTER_READ_SLOT_TIMER
    SM_E(ST_MASTER_READ_SLOT_TIMER, EV_PIN_CHG, on_master_read_slot_pin_chg),
    SM_E(ST_MASTER_READ_SLOT_TIMER, EV_TIMER_EXPIRED, on_master_read_slot_timer_expired), 



    // ST_MASTER_SEARCH_BIT_INIT
    SM_E(ST_MASTER_SEARCH_BIT_INIT, EV_PIN_CHG, on_master_search_bit_init_pin_chg),
    SM_E(ST_MASTER_SEARCH_BIT_INIT, EV_TIMER_EXPIRED, on_not_impl),

    // ST_MASTER_SEARCH_INIT_WAIT_RELEASE

    SM_E(ST_MASTER_SEARCH_BIT_WAIT_RELEASE, EV_PIN_CHG, on_master_search_bit_init_wait_pin_chg),
    SM_E(ST_MASTER_SEARCH_BIT_WAIT_RELEASE, EV_TIMER_EXPIRED, on_master_search_bit_init_wait_timer_expired), 

    // ST_MASTER_READ_SLOT_TIMER
    SM_E(ST_MASTER_SEARCH_BIT_SLOT_TIMER, EV_PIN_CHG, on_master_search_bit_slot_pin_chg),
    SM_E(ST_MASTER_SEARCH_BIT_SLOT_TIMER, EV_TIMER_EXPIRED, on_master_search_bit_slot_timer_expired), 

    // ST_MASTER_READ_SLOT_END
    SM_E(ST_MASTER_SEARCH_BIT_SLOT_END, EV_PIN_CHG, on_master_search_bit_slot_wait_end_pin_chg),
    SM_E(ST_MASTER_SEARCH_BIT_SLOT_END, EV_TIMER_EXPIRED, on_master_search_bit_slot_wait_end_timer_expired), 

    // ST_MASTER_SEARCH_INV_BIT_INIT
    SM_E(ST_MASTER_SEARCH_INV_BIT_INIT, EV_PIN_CHG, on_master_search_inv_bit_init_pin_chg),
    SM_E(ST_MASTER_SEARCH_INV_BIT_INIT, EV_TIMER_EXPIRED, on_not_impl),

    // ST_MASTER_SEARCH_INV_BIT_WAIT_RELEASE
    SM_E(ST_MASTER_SEARCH_INV_BIT_WAIT_RELEASE, EV_PIN_CHG, on_master_search_inv_bit_init_wait_pin_chg),
    SM_E(ST_MASTER_SEARCH_INV_BIT_WAIT_RELEASE, EV_TIMER_EXPIRED, on_master_search_inv_bit_init_wait_timer_expired), 

    // ST_MASTER_READ_SLOT_TIMER
    SM_E(ST_MASTER_SEARCH_INV_BIT_SLOT_TIMER, EV_PIN_CHG, on_master_search_inv_bit_slot_pin_chg),
    SM_E(ST_MASTER_SEARCH_INV_BIT_SLOT_TIMER, EV_TIMER_EXPIRED, on_master_search_inv_bit_slot_timer_expired), 

    // ST_MASTER_SEARCH_INV_BIT_SLOT_END
    SM_E(ST_MASTER_SEARCH_INV_BIT_SLOT_END, EV_PIN_CHG, on_master_search_inv_bit_slot_wait_end_pin_chg),
    SM_E(ST_MASTER_SEARCH_INV_BIT_SLOT_END, EV_TIMER_EXPIRED, on_master_search_inv_bit_slot_wait_end_timer_expired), 

    // ST_MASTER_SEARCH_BIT_INIT
    SM_E(ST_MASTER_SEARCH_TX_BIT_INIT, EV_PIN_CHG, on_master_search_tx_bit_init_pin_chg),
    SM_E(ST_MASTER_SEARCH_TX_BIT_INIT, EV_TIMER_EXPIRED, on_not_impl),

    // ST_MASTER_SEARCH_TX_BIT_WAIT_RELEASE
    SM_E(ST_MASTER_SEARCH_TX_BIT_WAIT_RELEASE, EV_PIN_CHG, on_master_search_tx_bit_wait_pin_chg),
    SM_E(ST_MASTER_SEARCH_TX_BIT_WAIT_RELEASE, EV_TIMER_EXPIRED, on_master_search_tx_bit_timer_expired), 

    // // ST_MASTER_READ_SLOT_END
    SM_E(ST_MASTER_SEARCH_TX_BIT_SLOT_END, EV_PIN_CHG, on_master_search_tx_bit_slot_end_pin_chg),
    SM_E(ST_MASTER_SEARCH_TX_BIT_SLOT_END, EV_TIMER_EXPIRED, on_master_search_tx_bit_slot_end_timer_expired), 


};

// ==================== Implementation =========================
void chip_init()
{
    setvbuf(stdout, NULL, _IOLBF, 1024);    
    printf("DS18B20 chip initialising...\n");

    chip_desc_t *chip = calloc(1, sizeof(chip_desc_t));

    timer_config_t timer_cfg = {
        .user_data = chip
    };
    timer_cfg.callback = on_timer_event;
    chip->timer = timer_init(&timer_cfg);
    timer_cfg.callback = on_reset_timer_event;
    chip->reset_timer = timer_init(&timer_cfg);

    chip->pin = pin_init("DQ", INPUT_PULLUP);
    const pin_watch_config_t watch_config = {
        .edge = BOTH,
        .pin_change = on_pin_change,
        .user_data = chip,
    };
    pin_watch(chip->pin, &watch_config);


    // read config attributes
    uint32_t attr;
    
    attr = attr_init("ow_debug", false); chip->ow_debug = attr_read(attr) != 0;
    attr = attr_init("gen_debug", false); chip->gen_debug = attr_read(attr) != 0;
    attr = attr_init("debug_timer", false); chip->debug_timer = attr_read(attr) != 0;

    attr = attr_init("presence_wait_time", PR_DUR_WAIT_PRESENCE); 
    chip->presence_wait_time = attr_read(attr);
    attr = attr_init("presence_time", PR_DUR_PULL_PRESENCE); 
    chip->presence_time = attr_read(attr);

    
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

    reset_state(chip);
    printf("DS18B20 chip initialised\n");
}

static void reset_state(chip_desc_t *chip) {
    DEBUGF("resetting state from %s\n", ST_NAME());
    chip->state = ST_INIT;
    chip->reset_time = 0;
    chip->reset_timer_expired = false;
    
    chip->read_slot_start = 0;
    chip->write_slot_start = 0;
    
    pin_mode(chip->pin, INPUT_PULLUP);

    timer_stop(chip->timer);
}


static void push_sm_event(chip_desc_t *chip, ev_t ev, uint32_t ev_data) {
    sm_entry_t h = sm[chip->state][ev];

    DEBUGF("%08lld %s[%s]: %s( %d )\n", get_sim_nanos(), h.st_name, h.ev_name, h.name, ev_data);
    if (h.handler == NULL) {
        DEBUGF("SM error: unhandled event %d in state %d, resetting\n", ev, chip->state);
        reset_state(chip);
        return;
  }

  h.handler(chip, ev_data);
    DEBUGF("new state -> %s\n", sm[chip->state][0].st_name);

}
// ==================== API handlers =========================
static void on_timer_event(void *data) {
    push_sm_event((chip_desc_t*)data, EV_TIMER_EXPIRED, 0);
}

// todo: implement
static void on_reset_timer_event(void *data) {
    push_sm_event((chip_desc_t*)data, EV_TIMER_EXPIRED, 0);
}

static void on_pin_change(void *data, pin_t pin, uint32_t value) {
  chip_desc_t *chip = (chip_desc_t*)data;

  if (pin != chip->pin) {
    DEBUGF("called back on wrong pin\n");
    return;
    }
    push_sm_event(chip, EV_PIN_CHG, value);
}

static void on_not_impl(chip_desc_t *chip, uint32_t data) {
    printf("not implemented\n");
}

static void on_init_pin_chg(chip_desc_t *chip, uint32_t data) {
    // pin transition to LOW starts a RESET sequence, arm the timer
    if (data == LOW) {
        timer_start(chip->timer, PR_DUR_RESET, false);
        chip->state = ST_WAIT_RESET;
        chip->reset_time = get_sim_nanos();
    } else {
        DEBUGF("L->H transition unexpected during initialisation, ignoring");
    }
}

static void on_reset_pin_chg(chip_desc_t *chip, uint32_t data) {
    uint64_t now = get_sim_nanos();
    
    timer_stop(chip->timer);

    // pin transition to LOW is unexpected since we're already supposed to be in this state
    if (data == LOW ) {
        DEBUGF("H->L transition unexpected while waiting for RESET sequence to complete, back to INIT");
        reset_state(chip);
        return;
    } 

    // if the timer has not expired, check if this is too quick
    uint64_t d = now - chip->reset_time;
    if (!chip->reset_timer_expired && !IN_RANGE(d, PR_DUR_RESET * 1000, PR_DUR_BUS_JITTER)) {
        DEBUGF("L->H unexpected during reset cycle, resetting %lld\n", now - chip->reset_time);
        reset_state(chip);
        return;
    }

    // we're ready for next phase, delay before presence
    // timer_start(chip->timer, chip->presence_wait_time, false);
    chip->state = ST_PULL_PRESENCE;
    pin_mode(chip->pin, OUTPUT_LOW); 
    timer_start(chip->timer, chip->presence_time, false);
}

static void on_reset_timer_expired(chip_desc_t *chip, uint32_t data) {
    chip->reset_timer_expired = true;
}

static void on_wait_presence_pin_chg(chip_desc_t *chip, uint32_t data) {
    uint64_t now = get_sim_nanos();

    // confirm this is a L->H transition
    if (data != HIGH) {
        DEBUGF("H->L transition unexpected during wait for presence cycle, resetting\n");
        reset_state(chip);
        return;
    }

    timer_stop(chip->timer);

    // it's our turn to pull the pin down
    pin_mode(chip->pin, OUTPUT_LOW); 
    timer_start(chip->timer, chip->presence_time, false);
}

static void on_wait_presence_timer_expired(chip_desc_t *chip, uint32_t data) {
    // pin has not changed yet, ignore for now
    DEBUGF("ignoring wait for presence bus release timer expiry\n");
}


static void on_presence_pin_chg(chip_desc_t *chip, uint32_t data) {
    DEBUGF("pin change transition unexpected during presence cycle, ignored\n");
    // reset_state(chip);
}

static void on_presence_timer_expired(chip_desc_t *chip, uint32_t data) {

    // release the bus and wait for master next write (bits)
    pin_mode(chip->pin, INPUT_PULLUP);
    chip->state = ST_MASTER_WRITE_INIT;
    chip->bit_ndx = 0; 
    chip->byte_ndx = 0; 
}



static void on_master_write_init_pin_chg(chip_desc_t *chip, uint32_t data) {
    if (data == HIGH) {
        DEBUGF("L->H transition unexpected while waiting for initial pull down during write time slot, resetting\n");
        reset_state(chip);
        return;
    }
    chip->state = ST_MASTER_WRITE_WAIT_RELEASE;
    chip->read_slot_start = get_sim_nanos();
    timer_start(chip->timer, PR_DUR_SAMPLE_WAIT, false);
}

static void on_master_write_pin_chg(chip_desc_t *chip, uint32_t data) {
    // todo(bonnyr): this needs to be confirmed as unexpected
    if (data == LOW) {
        DEBUGF("H->L transition unexpected while waiting for bus release during write time slot, resetting\n");
        reset_state(chip);
        return;
    }


    // ignore bit transition to HIGH  - this is the master releasing the bus. 
    // we wait for the timer to sample and then continue

    // todo(bonnyr): consider starting a protection timer
}

static void on_master_write_timer_expired(chip_desc_t *chip, uint32_t data) {
    bool bit = pin_read(chip->pin);

    chip->buffer[chip->byte_ndx] |= bit << chip->bit_ndx;

    DEBUGF("on_master_write_timer_expired: reading buf[%d]: %x (bit: %d<<%d)\n", chip->byte_ndx, chip->buffer[chip->byte_ndx], bit, chip->bit_ndx);


    chip->bit_ndx++;
    if (chip->bit_ndx >= 8) {

        chip->bit_ndx = 0;
        chip->byte_ndx++;
    }
    if (chip->byte_ndx >= BUF_LEN) {
        DEBUGF("unexpected master writes > buffer_LEN, resetting");
        reset_state(chip);
        return;
    }

    // if the bit is high, we can got back for more and don't need to wait
    if (bit) {
        chip->state = ST_MASTER_WRITE_INIT;
        if (chip->bit_ndx == 0 && chip->byte_ndx == 1) {
            on_rom_command(chip);
        }
        return;
    }

    chip->state = ST_MASTER_WRITE_SLOT_END;
}


static void on_master_write_slot_end_pin_chg(chip_desc_t *chip, uint32_t data) {
    // todo(bonnyr): this needs to be confirmed as unexpected
    if (data == LOW) {
        DEBUGF("H->L transition unexpected while waiting for bus release during write 0 time slot (after sampling), resetting");
        reset_state(chip);
        return;
    }

    // go back for more
    chip->state = ST_MASTER_WRITE_INIT;

    // need to check whether we have a complete message or we need to wait for more
    if (chip->bit_ndx == 0 && chip->byte_ndx == 1) {
        on_rom_command(chip);
    }
}



static void on_master_read_init_pin_chg(chip_desc_t *chip, uint32_t data) {
    if (data == HIGH) {
        DEBUGF("L->H transition unexpected while waiting for initial pull down during write time slot, resetting");
        reset_state(chip);
        return;
    }

    // pin dropped low, we need to wait >1us (but since we need to start pulling while the bus is also pulling, we'll do this for 1us)
    chip->state = ST_MASTER_READ_INIT_WAIT_RELEASE;
    chip->read_slot_start = get_sim_nanos();
    timer_start(chip->timer, PR_DUR_READ_INIT, false);
}


static void write_next_bit(chip_desc_t *chip) {
    chip->cur_bit = CUR_BIT(chip) ; // chip->buffer[chip->byte_ndx] &= 1 << chip->bit_ndx;  
    chip->bit_ndx++;
    if (chip->bit_ndx >= 8) {
        chip->bit_ndx = 0;
        chip->byte_ndx++;
    }

    if (!chip->cur_bit) {
        pin_mode(chip->pin, OUTPUT_LOW);
    }

    timer_start(chip->timer, PR_DUR_READ_SLOT - (get_sim_nanos() - chip->read_slot_start)/1000, false);
    chip->state = ST_MASTER_READ_SLOT_TIMER;

}
// this should not happen, but just in case, write the bit anyway
static void on_master_read_init_wait_pin_chg(chip_desc_t *chip, uint32_t data) {
    // todo(bonnyr): this needs to be confirmed as unexpected
    if (data == LOW) {
        DEBUGF("H->L transition unexpected while waiting for bus release during write time slot, resetting");
        reset_state(chip);
        return;
    }

    // bus has been released, we're ready to write the next bit we need to
    timer_stop(chip->timer);

    write_next_bit(chip);

}

static void on_master_read_init_wait_timer_expired(chip_desc_t *chip, uint32_t data) {
    write_next_bit(chip);
}



static void on_master_read_slot_pin_chg(chip_desc_t *chip, uint32_t data) {
    // todo(bonnyr): this needs to be confirmed as unexpected
    if (data == HIGH && !chip->cur_bit || data == LOW && chip->cur_bit) {
        DEBUGF("L->H or H->L transition unexpected while waiting for READ slot timer, resetting");
        reset_state(chip);
        return;
    }  

}

static void on_master_read_slot_timer_expired(chip_desc_t *chip, uint32_t data) {

    // check if we finished writing to master. This happens only when we
    // finish writing at least a byte worth of stuff
    if (chip->bit_ndx == 0 && chip->byte_ndx == chip->resp_len) {
        reset_state(chip);
        return;
    }

    // go back for more
    chip->state = ST_MASTER_READ_INIT;
}




// ---- search
static void write_bit(chip_desc_t *chip, bool bit, ow_state_t state) {

    DEBUGF("write_bit in %s: %d\n", ST_NAME(), bit);

    if (!bit) {
        pin_mode(chip->pin, OUTPUT_LOW);
    }

    timer_start(chip->timer, PR_DUR_READ_SLOT - (get_sim_nanos() - chip->read_slot_start)/1000, false);
    chip->state = state;

}

static void on_read_state_init_pin_chg(chip_desc_t *chip, uint32_t data, ow_state_t state) {
    if (data == HIGH) {
        DEBUGF("L->H transition unexpected while waiting for initial pull down during write time slot, resetting");
        reset_state(chip);
        return;
    }

    // pin dropped low, we need to wait >1us (but since we need to start pulling while the bus is also pulling, we'll do this for 1us)
    chip->state = state;
    chip->read_slot_start = get_sim_nanos();
    timer_start(chip->timer, PR_DUR_READ_INIT, false);

}



static void on_read_state_init_wait_pin_chg(chip_desc_t *chip, uint32_t data, bool bit, ow_state_t state) {
    // todo(bonnyr): this needs to be confirmed as unexpected
    if (data == LOW) {
        DEBUGF("H->L transition unexpected while waiting for bus release during write time slot, resetting");
        reset_state(chip);
        return;
    }

    // bus has been released, we're ready to write the next bit we need to
    timer_stop(chip->timer);

    write_bit(chip, bit, state);

}



static void on_read_slot_pin_chg(chip_desc_t *chip, uint32_t data, bool bit) {
    // todo(bonnyr): this needs to be confirmed as unexpected
        DEBUGF("ignoring expected L->H transition\n");
    // if (data == HIGH && !bit || data == LOW && bit) {
    //     DEBUGF("L->H or H->L transition unexpected while waiting for READ slot timer, resetting");
    //     reset_state(chip);
    //     return;
    // }  
}

static void on_read_slot_timer_expired(chip_desc_t *chip, bool bit, ow_state_t state) {

    // if we pulled the bus low, release it
    if (!bit) {
        DEBUGF("on_read_slot_timer_expired: pulling bit up\n");
        // this call caused the sim engine to call us back within the same call stack
        // hence we do not need to wait for the slot end, just assume next cycle starts by master
        pin_mode(chip->pin, INPUT_PULLUP);
        DEBUGF("on_read_slot_timer_expired: pulling bit up - done\n");
    }

    // check if we finished writing to master. This happens only when we
    // finish writing at least a byte worth of stuff
    if (chip->bit_ndx == 0 && chip->byte_ndx == chip->resp_len) {
        reset_state(chip);
        return;
    }

    timer_start(chip->timer, PR_DUR_READ_SLOT_END, false);
    chip->state = state;
}



static void on_read_slot_wait_end_pin_chg(chip_desc_t *chip) {
    DEBUGF("ignoring expected L->H transition");
}

static void on_read_slot_wait_end_timer_expired(chip_desc_t *chip, ow_state_t state) {
    // transition to next state
    chip->state = state;
}


static void on_write_state_init_pin_chg(chip_desc_t *chip, uint32_t data, ow_state_t state) {
    if (data == HIGH) {
        DEBUGF("on_write_state: L->H transition while waiting for initial pull down during write time slot, resetting\n");
        reset_state(chip);
        return;
    }
    chip->state = state;
    chip->read_slot_start = get_sim_nanos();
    timer_start(chip->timer, PR_DUR_SAMPLE_WAIT, false);
}

static void on_write_state_pin_chg(chip_desc_t *chip, uint32_t data) {
    // todo(bonnyr): this needs to be confirmed as unexpected
    if (data == LOW) {
        DEBUGF("on_write_state_pin_chg: H->L transition unexpected while waiting for bus release during write time slot, resetting\n");
        reset_state(chip);
        return;
    }


    // ignore bit transition to HIGH  - this is the master releasing the bus. 
    // we wait for the timer to sample and then continue

    // todo(bonnyr): consider starting a protection timer
}

static bool on_write_state_timer_expired(chip_desc_t *chip, ow_state_t state) {
    bool bit = pin_read(chip->pin);

    chip->state = state;
    return bit;
}


static void on_write_state_slot_end_pin_chg(chip_desc_t *chip, uint32_t data, ow_state_t state) {
    // todo(bonnyr): this needs to be confirmed as unexpected
    if (data == LOW) {
        DEBUGF("on_write_state_slot_end_pin_chg: H->L transition unexpected while waiting for bus release during write 0 time slot (after sampling), resetting");
        reset_state(chip);
        return;
    }

    // go back for more
    chip->state = state;
}








static void on_master_search_bit_init_pin_chg(chip_desc_t *chip, uint32_t data) {
    on_read_state_init_pin_chg(chip, data, ST_MASTER_SEARCH_BIT_WAIT_RELEASE);
}

// this should not happen, but just in case, write the bit anyway
static void on_master_search_bit_init_wait_pin_chg(chip_desc_t *chip, uint32_t data) {
    on_read_state_init_wait_pin_chg(chip, data, CUR_BIT(chip), ST_MASTER_SEARCH_BIT_SLOT_TIMER);
}

static void on_master_search_bit_init_wait_timer_expired(chip_desc_t *chip, uint32_t data) {
    write_bit(chip, CUR_BIT(chip), ST_MASTER_SEARCH_BIT_SLOT_TIMER);
}

static void on_master_search_bit_slot_pin_chg(chip_desc_t *chip, uint32_t data) {
    on_read_slot_pin_chg(chip, data, CUR_BIT(chip));
}

static void on_master_search_bit_slot_timer_expired(chip_desc_t *chip, uint32_t data) {
    on_read_slot_timer_expired(chip, CUR_BIT(chip), ST_MASTER_SEARCH_BIT_SLOT_END);
}


// these may not be needed for now due to re-entrant behavior of pin_mode in previous state
static void on_master_search_bit_slot_wait_end_pin_chg(chip_desc_t *chip, uint32_t data) {
    // should be ignored
    on_read_slot_wait_end_pin_chg(chip);
}

static void on_master_search_bit_slot_wait_end_timer_expired(chip_desc_t *chip, uint32_t data) {
    on_read_slot_wait_end_timer_expired(chip, ST_MASTER_SEARCH_INV_BIT_INIT);
}















static void on_master_search_inv_bit_init_pin_chg(chip_desc_t *chip, uint32_t data) {
    on_read_state_init_pin_chg(chip, data, ST_MASTER_SEARCH_INV_BIT_WAIT_RELEASE);
}

// this should not happen, but just in case, write the bit anyway
static void on_master_search_inv_bit_init_wait_pin_chg(chip_desc_t *chip, uint32_t data) {
    on_read_state_init_wait_pin_chg(chip, data, !(CUR_BIT(chip)), ST_MASTER_SEARCH_INV_BIT_SLOT_TIMER);
}

static void on_master_search_inv_bit_init_wait_timer_expired(chip_desc_t *chip, uint32_t data) {
    write_bit(chip, !(CUR_BIT(chip)), ST_MASTER_SEARCH_INV_BIT_SLOT_TIMER);
}

static void on_master_search_inv_bit_slot_pin_chg(chip_desc_t *chip, uint32_t data) {
    on_read_slot_pin_chg(chip, data, !(CUR_BIT(chip)));
}

static void on_master_search_inv_bit_slot_timer_expired(chip_desc_t *chip, uint32_t data) {
    on_read_slot_timer_expired(chip, !CUR_BIT(chip), ST_MASTER_SEARCH_INV_BIT_SLOT_END);
}

static void on_master_search_inv_bit_slot_wait_end_pin_chg(chip_desc_t *chip, uint32_t data) {
    on_read_slot_wait_end_pin_chg(chip);
}

static void on_master_search_inv_bit_slot_wait_end_timer_expired(chip_desc_t *chip, uint32_t data) {
    on_read_slot_wait_end_timer_expired(chip, ST_MASTER_SEARCH_TX_BIT_INIT);
}







static void on_master_search_tx_bit_init_pin_chg(chip_desc_t *chip, uint32_t data) {
    on_write_state_init_pin_chg(chip, data, ST_MASTER_SEARCH_TX_BIT_WAIT_RELEASE);
}

// this should not happen, but just in case, write the bit anyway
static void on_master_search_tx_bit_wait_pin_chg(chip_desc_t *chip, uint32_t data) {
    DEBUGF("on_master_search_tx_bit_wait_pin_chg: ignoring\n")
    // on_write_state_pin_chg(chip, data);
}

static void on_master_search_tx_bit_timer_expired(chip_desc_t *chip, uint32_t data) {
    bool bit = pin_read(chip->pin);


    DEBUGF("on_master_search_tx_bit_timer_expired: comparing bit %d - m:%d, s:%d\n", chip->byte_ndx * 8 + chip->bit_ndx, bit, CUR_BIT(chip))
    // if master transmitted bit does not match ours, reset
    if (bit != CUR_BIT(chip))  {
        reset_state(chip);
        return;
    }


    chip->bit_ndx++;
    if (chip->bit_ndx >= 8) {
        chip->bit_ndx = 0;
        chip->byte_ndx++;
    }

    // if we sampled '1', we do not need to wait before going to next bit, so change state here
    if (bit) {
        DEBUGF("on_master_search_tx_bit_timer_expired: bus transmitted '1', moving to next state\n");
        if (chip->byte_ndx == SERIAL_LEN) {
            DEBUGF("on_master_search_tx_bit_timer_expired: *** finished search, going back to init\n");
            reset_state(chip);
        } else {
            chip->state = ST_MASTER_SEARCH_BIT_INIT;
        }
        return;
    } 
    
    // we need to wait for the master to release the bus
    chip->state = ST_MASTER_SEARCH_TX_BIT_SLOT_END;

    // todo(bonnyr): consider setting a protection timer
    // timer_start(chip->timer, PR_DUR_WRITE_SLOT_END, false);

    
}

static void on_master_search_tx_bit_slot_end_pin_chg(chip_desc_t *chip, uint32_t data) {
    if (chip->byte_ndx == SERIAL_LEN) {
        DEBUGF("on_master_search_tx_bit_slot_end_pin_chg: *** finished search, going back to init\n");
        reset_state(chip);
        return;
    }
    chip->state = ST_MASTER_SEARCH_BIT_INIT;
    // DEBUGF("on_master_search_tx_bit_wait_pin_chg: ignoring\n")
}

static void on_master_search_tx_bit_slot_end_timer_expired(chip_desc_t *chip, uint32_t data) {
    DEBUGF("on_master_search_tx_bit_slot_end_timer_expired: should not happen \n")
    // chip->state = ST_MASTER_SEARCH_BIT_INIT;
}




// ==================== Logic Implementation =========================
static void on_rom_command(chip_desc_t *chip) {
    switch(chip->buffer[0]) {
    case OW_CMD_SEARCH: on_ow_search(chip); return; 
    case OW_CMD_READ:   on_ow_read(chip); return;
    case OW_CMD_MATCH:
    case OW_CMD_SKIP:
    case OW_CMD_ALM_SEARCH:
    case OW_CMD_CONVERT:
    case OW_CMD_WR_SCRATCH:
    case OW_CMD_RD_SCRATCH:
    case OW_CMD_CP_SCRATCH:
    case OW_CMD_RECALL:
    case OW_CMD_RD_PWD:
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
    chip->state = ST_MASTER_SEARCH_BIT_INIT;
    chip->bit_ndx = 0;
    chip->byte_ndx = 0;
    chip->resp_len = SERIAL_LEN;
}
static void on_ow_read(chip_desc_t *chip) {
    DEBUGF("on_ow_read\n");
    memcpy(chip->buffer, chip->serial_no, SERIAL_LEN);
    chip->state = ST_MASTER_READ_INIT;
    chip->bit_ndx = 0;
    chip->byte_ndx = 0;
    chip->resp_len = SERIAL_LEN;
}
static void on_ow_match(chip_desc_t *chip) {

}
static void on_ow_skip(chip_desc_t *chip) {

}
static void on_ow_alarm_seach(chip_desc_t *chip) {

}
static void on_ow_convert(chip_desc_t *chip) {

}
static void on_ow_write_scratchpad(chip_desc_t *chip) {

}
static void on_ow_read_scratchpad(chip_desc_t *chip) {

}
static void on_ow_copy(chip_desc_t *chip) {

}
static void on_ow_recall(chip_desc_t *chip) {

}
static void on_ow_read_power(chip_desc_t *chip) {

}
