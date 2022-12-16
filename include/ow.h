//
// Created by bonny on 5/12/2022.
//

#ifndef WOKWI_DS1820_CUSTOM_CHIP_OW_H
#define WOKWI_DS1820_CUSTOM_CHIP_OW_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hashmap.h"

#define DEBUG 1

// --------------- Debug Macros -----------------------
#ifdef DEBUG
#ifndef OW_DEBUGF
#define _DEBUGF(debug, ...)  { if (debug) {printf(__VA_ARGS__);}  }
#define OW_DEBUGF(...)  _DEBUGF(ctx->ow_debug, __VA_ARGS__)
#endif
#else
#ifndef OW_DEBUGF
#define OW_DEBUGF(...)
#endif
#endif //  DEBUG
// --------------- Debug Macros -----------------------

#define MICROS_IN_SEC 1000000
#define SIG_TXN_DUR_NS 1000    // assume it takes 1us to stabilise the signal
#define IN_RANGE(x, y, err) ( (y) > (x) ? (y) - (x) <= err : (x) - (y) <= err)
#define OW_ELAPSED(t) ((get_sim_nanos() - t))
#define OW_ELAPSED_US(t) (OW_ELAPSED(t) / 1000)
#define _NS(v) ((uint64_t)v *1000)
#define _US(v) ((uint64_t)v /1000)

#define TOO_EARLY(t, p, e) (get_sim_nanos() + e < t + p) // - (OW_ELAPSED(t) - p) < e)


// protocol definitions of various durations

// reset
// --\                /-------------\                      /----- ... v----\
//    \--------------/               \--------------------/                \
//   rst    reset  master   master  slv     slv          slv         slv
//   pull    dur     rls  rls-time  pull  pull-time      rls         ready
//
#define PR_DUR_RESET 480                    // initial MASTER reset duration
#define PR_DUR_FORCED_RESET 485             // forced reset (pin held low for at least this duration)
#define PR_DUR_RESET_MASTER_RELEASE 30      // SLAVE needs to wait between 15-60us for bus to be released/stabilised
#define PR_DUR_RESET_PULL_PRESENCE 120      // slv pull-time above
#define PR_DUR_RESET_SLOT_END  329          // time to slv-ready (1us before nominal end of reset cycle)



// write 0                       write 1
// --\                /------    --\        /-------^------
//    \-------^------/              \------/        |
//   write  sample  master         write  master  sample
//   pull    dur     rls           pull    rls     dur
//

#define PR_DUR_SAMPLE_WAIT 15
#define PR_DUR_WRITE_SLOT_END 45



// read 0                                       read 1
// --\                            /------     --\      //-------v------
//   \-------------------v--------/             \------//       |
//   read    master     slv  master             read  m+s     sample
//   pull   pull-time   pll-time                pull    rls     dur
//

#define PR_DUR_READ_SLOT 15
#define PR_DUR_READ_SLOT_END 45
#define PR_DUR_READ_INIT 1

// allow 2us of jitter
#define PR_DUR_BUS_JITTER _NS(2)


#define OW_ERR_NO_ERROR 0x0
#define OW_ERR_UNEXPECTED_BIT_STATE 0x8000
#define OW_ERR_WAITED_TOO_LONG 0x8001


typedef enum {
    ST_RESET_INIT,
    ST_RESET_WAIT_RELEASE,
    ST_RESET_WAIT_PRESENCE,
    ST_RESET_PULL_PRESENCE,
    ST_RESET_DONE,

    ST_MASTER_WRITE_INIT,
    ST_MASTER_WRITE_WAIT_SAMPLE,
    ST_MASTER_WRITE_SLOT_END,
    ST_MASTER_WRITE_DONE,


    ST_MASTER_READ_INIT,
    ST_MASTER_READ_WAIT_SAMPLE,
    ST_MASTER_READ_SLOT_END,
    ST_MASTER_READ_DONE,


    ST_SIG_MAX
} ow_sig_state_t;

typedef enum {
    ST_READ_RUNNING,
    ST_READ_DONE,

    ST_READ_MAX,
} ow_read_byte_state_t;

typedef enum {
    ST_WRITE_RUNNING,
    ST_WRITE_DONE,

    ST_WRITE_MAX,
} ow_write_byte_state_t;



typedef enum {
    EV_PIN_CHG,
    EV_TIMER_EXPIRED,
    EV_RESET_TIMER_EXPIRED,

    EV_MAX
} ev_t;

typedef enum {
    CB_RESET_DONE,
    CB_BIT_READ,
    CB_BIT_WRITTEN,
} cb_ev_t;


typedef enum {
    EV_BIT_READ,
    EV_BIT_WRITTEN,
//    EV_BIT_ERROR,

    EV_BIT_MAX
} ev_bit_t;


typedef enum {
    EV_BYTE_READ,
    EV_BYTE_WRITTEN,
    EV_BYTE_ERROR,

    EV_BYTE_MAX
} ev_byte_t;


typedef struct sm sm_t;
typedef void (*sig_cb)(void *user_data, uint32_t err, uint32_t cb_data);
typedef void (*reset_state)(void *ctx);

typedef struct ow_ctx {
    uint32_t state;     // since we're using enums, this can be stored as uint32_t. Allows using for multiple SM types
    timer_t timer;
    timer_t reset_detection_timer;
    pin_t pin;
    bool bit_buf;

    void *user_data;
    sig_cb forced_reset_callback;
    sig_cb reset_callback;
    sig_cb bit_read_callback;
    sig_cb bit_written_callback;
    reset_state reset_fn;

    uint64_t reset_time;
    uint64_t slot_start;
    bool reset_timer_expired;

    sm_t *cur_sm;

    bool ow_debug;
} ow_ctx_t;

typedef struct {
    void *data;
    const char *pin_name;
    sig_cb  forced_reset_cb;
    sig_cb  reset_cb;
    sig_cb  bit_read_cb;
    sig_cb  bit_written_cb;
} ow_ctx_cfg_t;

typedef void (*byte_cb)(void *user_data, uint32_t err, uint32_t cb_data);
typedef struct ow_byte_ctx {
    uint32_t state;     // since we're using enums, this can be stored as uint32_t. Allows using for multiple SM types
    uint8_t byte_buf;
    uint8_t bit_ndx;


    void *user_data;
    sig_cb bit_callback;    // allow user to access our own bit callback
    byte_cb callback;       // this is the user callback called when a whole byte is complete
    ow_ctx_t *ow_ctx;       // signalling context
    reset_state reset_fn;   // used by generic sm to reset state on error

    bool ow_debug;
} ow_byte_ctx_t;



// --- State Machines ---
typedef void (*sm_handler)(void *user_data, uint32_t data);
typedef struct sm_entry {
    const char *name;
    const char *st_name;
    const char *ev_name;
    uint32_t state;
    uint32_t event;
    sm_handler handler;
    uint64_t key;
} sm_entry_t;

typedef struct sm_cfg {
    const char *name;
    int max_states;
    int max_events;
    sm_entry_t *sm_entries;
    int num_entries;
} sm_cfg_t;

typedef HASHMAP(uint64_t, sm_entry_t ) sm_entry_map_t;


typedef struct sm {
    sm_cfg_t *cfg;
    const void *hash;
} sm_t;

// forward decl for sm
void sm_push_event(sm_t *sm, void *ctx, reset_state reset_fn, uint32_t state, uint32_t ev, uint32_t ev_data, bool debug);
void sm_init_hash(sm_t *sm);
uint64_t state_event_to_key(uint32_t state, uint32_t event);

// helper macros
#define ST_NAME(sm) (sm[0].st_name)
#define SM_E(s, e, h) (sm_entry_t){.name = #h, .st_name = #s, .ev_name = #e, .state = s, .event = e, .handler = h}
#define OW_CTX(d) ow_ctx_t *ctx = d


extern sm_t *sm_sig;

extern sm_t *sm_read_byte;
extern sm_t *sm_write_byte;



// forward decl for ow api
ow_ctx_t * ow_ctx_init(ow_ctx_cfg_t *cfg);
void ow_ctx_reset_state(ow_ctx_t *ctx);
void ow_ctx_set_master_write_state(ow_ctx_t *ctx, bool bit);
void ow_ctx_set_master_read_state(ow_ctx_t *ctx, bool bit);

void on_not_impl(void *chip, uint32_t data);

ow_byte_ctx_t *ow_read_byte_ctx_init(void *data, byte_cb cb, ow_ctx_t *ow_ctx);
ow_byte_ctx_t *ow_write_byte_ctx_init(void *data, byte_cb cb, ow_ctx_t *ow_ctx);
void ow_read_byte_ctx_reset_state(ow_byte_ctx_t *ctx);
void ow_write_byte_ctx_reset_state(ow_byte_ctx_t *ctx, uint8_t byte_buf);




#endif //WOKWI_DS1820_CUSTOM_CHIP_OW_H
