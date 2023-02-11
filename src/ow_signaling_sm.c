

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>

#include "wokwi-api.h"
#include "hashmap.h"
#include "ow.h"

// ================== Impl ============
static void on_reset_timer_event(void *ctx);
static void on_timer_event(void *data);
static void on_pin_change(void *user_data, pin_t pin, uint32_t value);

static void on_ignored(void *ctx, uint32_t data);
static void on_reset_detected(void *ctx, uint32_t data);

static void on_reset_init_pin_chg(void *ctx, uint32_t data);
static void on_reset_wait_release_pin_chg(void *ctx, uint32_t data);
static void on_reset_wait_release_timer_expired(void *ctx, uint32_t data);
static void on_reset_wait_presence_pin_chg(void *ctx, uint32_t data);
static void on_reset_wait_presence_timer_expired(void *ctx, uint32_t data);
static void on_reset_pull_presence_pin_chg(void *ctx, uint32_t data);
static void on_reset_pull_presence_timer_expired(void *ctx, uint32_t data);
static void on_reset_done_pin_chg(void *ctx, uint32_t data);
static void on_reset_done_timer_expired(void *ctx, uint32_t data);
static void on_master_write_init_pin_chg(void *d, uint32_t data);
static void on_master_write_wait_sample_pin_chg(void *d, uint32_t data);
static void on_master_write_wait_sample_timer_expired(void *d, uint32_t data);
static void on_master_write_wait_slot_end_pin_chg(void *d, uint32_t data);
static void on_master_write_wait_slot_end_timer_expired(void *d, uint32_t data);
static void on_master_write_done_pin_chg(void *d, uint32_t data);

static void on_master_read_init_pin_chg(void *d, uint32_t data);
static void on_master_read_wait_sample_pin_chg(void *d, uint32_t data);
static void on_master_read_wait_sample_timer_expired(void *ctx, uint32_t data);
static void on_master_read_slot_end_pin_chg(void *d, uint32_t data);
static void on_master_read_slot_end_timer_expired(void *d, uint32_t data);
static void on_master_read_done_pin_chg(void *d, uint32_t data);


sm_entry_t sm_sig_entries[] = { //[ST_RESET_MAX][EV_MAX]
        // ST_INIT
        SM_E(ST_RESET_INIT, EV_PIN_CHG, on_reset_init_pin_chg),
        SM_E(ST_RESET_INIT, EV_TIMER_EXPIRED, on_not_impl),
        SM_E(ST_RESET_INIT, EV_RESET_TIMER_EXPIRED, on_ignored),

        // ST_RESET_WAIT_RELEASE,
        SM_E(ST_RESET_WAIT_RELEASE, EV_PIN_CHG, on_reset_wait_release_pin_chg),
        SM_E(ST_RESET_WAIT_RELEASE, EV_TIMER_EXPIRED, on_reset_wait_release_timer_expired),
        SM_E(ST_RESET_WAIT_RELEASE, EV_RESET_TIMER_EXPIRED, on_ignored),

        // ST_RESET_WAIT_PRESENCE
        SM_E(ST_RESET_WAIT_PRESENCE, EV_PIN_CHG, on_reset_wait_presence_pin_chg),
        SM_E(ST_RESET_WAIT_PRESENCE, EV_TIMER_EXPIRED, on_reset_wait_presence_timer_expired),
        SM_E(ST_RESET_WAIT_PRESENCE, EV_RESET_TIMER_EXPIRED, on_reset_detected),

        // ST_RESET_PULL_PRESENCE
        SM_E(ST_RESET_PULL_PRESENCE, EV_PIN_CHG, on_reset_pull_presence_pin_chg),
        SM_E(ST_RESET_PULL_PRESENCE, EV_TIMER_EXPIRED, on_reset_pull_presence_timer_expired),
        SM_E(ST_RESET_PULL_PRESENCE, EV_RESET_TIMER_EXPIRED, on_reset_detected),

        // ST_RESET_DONE
        SM_E(ST_RESET_DONE, EV_PIN_CHG, on_reset_done_pin_chg),
        SM_E(ST_RESET_DONE, EV_TIMER_EXPIRED, on_reset_done_timer_expired),
        SM_E(ST_RESET_DONE, EV_RESET_TIMER_EXPIRED, on_reset_detected),

        // ST_MASTER_WRITE_INIT
        SM_E(ST_MASTER_WRITE_INIT, EV_PIN_CHG, on_master_write_init_pin_chg),
        SM_E(ST_MASTER_WRITE_INIT, EV_TIMER_EXPIRED, on_not_impl),
        SM_E(ST_MASTER_WRITE_INIT, EV_RESET_TIMER_EXPIRED, on_reset_detected),

        // ST_MASTER_WRITE_WAIT_RELEASE
        SM_E(ST_MASTER_WRITE_WAIT_SAMPLE, EV_PIN_CHG, on_master_write_wait_sample_pin_chg),
        SM_E(ST_MASTER_WRITE_WAIT_SAMPLE, EV_TIMER_EXPIRED, on_master_write_wait_sample_timer_expired),
        SM_E(ST_MASTER_WRITE_WAIT_SAMPLE, EV_RESET_TIMER_EXPIRED, on_reset_detected),

        // ST_MASTER_WRITE_SLOT_END
        SM_E(ST_MASTER_WRITE_SLOT_END, EV_PIN_CHG, on_master_write_wait_slot_end_pin_chg),
        SM_E(ST_MASTER_WRITE_SLOT_END, EV_TIMER_EXPIRED, on_master_write_wait_slot_end_timer_expired),
        SM_E(ST_MASTER_WRITE_SLOT_END, EV_RESET_TIMER_EXPIRED, on_reset_detected),

        // ST_MASTER_WRITE_DONE
        SM_E(ST_MASTER_WRITE_DONE, EV_PIN_CHG, on_master_write_done_pin_chg),
        SM_E(ST_MASTER_WRITE_DONE, EV_TIMER_EXPIRED, on_not_impl),
        SM_E(ST_MASTER_WRITE_DONE, EV_RESET_TIMER_EXPIRED, on_reset_detected),

        // ST_MASTER_READ_INIT
        SM_E(ST_MASTER_READ_INIT, EV_PIN_CHG, on_master_read_init_pin_chg),
        SM_E(ST_MASTER_READ_INIT, EV_TIMER_EXPIRED, on_not_impl),
        SM_E(ST_MASTER_READ_INIT, EV_RESET_TIMER_EXPIRED, on_reset_detected),

        // ST_MASTER_READ_INIT_WAIT_RELEASE
        SM_E(ST_MASTER_READ_WAIT_SAMPLE, EV_PIN_CHG, on_master_read_wait_sample_pin_chg),
        SM_E(ST_MASTER_READ_WAIT_SAMPLE, EV_TIMER_EXPIRED, on_master_read_wait_sample_timer_expired),
        SM_E(ST_MASTER_READ_WAIT_SAMPLE, EV_RESET_TIMER_EXPIRED, on_reset_detected),

        // ST_MASTER_READ_SLOT_TIMER
        SM_E(ST_MASTER_READ_SLOT_END, EV_PIN_CHG, on_master_read_slot_end_pin_chg),
        SM_E(ST_MASTER_READ_SLOT_END, EV_TIMER_EXPIRED, on_master_read_slot_end_timer_expired),
        SM_E(ST_MASTER_READ_SLOT_END, EV_RESET_TIMER_EXPIRED, on_reset_detected),

        // ST_MASTER_READ_DONE
        SM_E(ST_MASTER_READ_DONE, EV_PIN_CHG, on_master_read_done_pin_chg),
        SM_E(ST_MASTER_READ_DONE, EV_TIMER_EXPIRED, on_not_impl),
        SM_E(ST_MASTER_READ_DONE, EV_RESET_TIMER_EXPIRED, on_reset_detected),

};

sm_cfg_t sm_sig_cfg = {
        .name = "sm_sig",
        .sm_entries = sm_sig_entries,
        .num_entries = sizeof(sm_sig_entries)/sizeof(sm_entry_t),
};


sm_t *sm_sig  = &(sm_t){.cfg = &sm_sig_cfg};

int sm_key_compare(const uint64_t *k1, const uint64_t *k2 ) {
    return *k1 == *k2 ? 0 : *k1 > *k2 ? 1 : -1;
}

size_t sm_key_hash(const uint64_t *k1 ) {
    return hashmap_hash_default(k1, sizeof(*k1));
}

uint64_t state_event_to_key(uint32_t state, uint32_t event) {
    return ((uint64_t)state) << 32 | event;
}


void sm_init_hash(sm_t *sm) {
    sm_entry_map_t *hash = calloc(1, sizeof(sm_entry_map_t));
    sm->hash = hash;

    hashmap_init(hash, sm_key_hash, sm_key_compare);
    for( int i= 0; i < sm->cfg->num_entries; i++) {
        sm_entry_t *e = &sm->cfg->sm_entries[i];
        e->key = state_event_to_key(e->state, e->event);
        hashmap_put(hash, &e->key, e);
    }
}




void sm_push_event(sm_t *sm, void *ctx, reset_state reset_fn, uint32_t state, uint32_t event, uint32_t ev_data, bool debug) {
    if (!sm->hash) {
        sm_init_hash(sm);
    }

    uint64_t key = state_event_to_key(state, event);
    sm_entry_t *h = hashmap_get((sm_entry_map_t *)sm->hash, &key);

//    sm_entry_t h = *(sm->sm_entries + sm->max_events * state + event);

    if (h == NULL || h->handler == NULL) {
        _DEBUGF(debug, "SM error: unhandled event %d in state %d, resetting\n", event, state);
        reset_fn(ctx);
        return;
    } else if (h->handler == on_not_impl) {
        _DEBUGF(debug, "(%lld) %s[%s]: %s( %d ) - *** not implemented ***\n", OW_ELAPSED_US(((ow_ctx_t*)ctx)->reset_time),
                h->st_name, h->ev_name, h->name, ev_data);
    } else {
                _DEBUGF(debug, "%08lld sm_push_event> (%lld) %s (ctx:%p) %s[%s]: %s( %d ) -> %p\n",
                        get_sim_nanos(),
                        OW_ELAPSED_US(((ow_ctx_t*)ctx)->reset_time), sm->cfg->name, ctx, h->st_name,
                        h->ev_name, h->name, ev_data, h->handler);
                h->handler(ctx, ev_data);
    }


    key = state_event_to_key(((ow_ctx_t*)ctx)->state, event);
    h = hashmap_get((sm_entry_map_t *)sm->hash, &key);
    if (h == NULL) {
        _DEBUGF(debug, "sm_push_event< invalid next state\n");
    } else {
        _DEBUGF(debug, "%08lld sm_push_event< %s (ctx: %p) next state=> %s(%d)\n",
                get_sim_nanos(), sm->cfg->name, ctx, h->st_name, h->state);
    }
}



static void on_timer_event(void *data) {
    OW_CTX(data);
    sm_push_event(sm_sig, data, ctx->reset_fn, ctx->state, EV_TIMER_EXPIRED, 0,
                  ctx->ow_debug);
}

static void on_reset_timer_event(void *data) {
    OW_CTX(data);
    OW_DEBUGF("on_reset_timer_event: %d\n", ctx->reset_detection_timer);


    // if the timer expired, last pin change was pulled low meaning there 
    // was no other bus event and the master decided to reset us
    // for now, record this and allow sm to handle this fact
    sm_push_event(sm_sig, ctx, ((ow_ctx_t *) data)->reset_fn, ((ow_ctx_t *) data)->state, EV_RESET_TIMER_EXPIRED, 0, ctx->ow_debug);
}

static void on_pin_change(void *data, pin_t pin, uint32_t value) {
    ow_ctx_t *ctx = data;

    if (pin != ctx->pin) {
        OW_DEBUGF("called back on wrong pin\n");
        return;
    }

    ctx->reset_timer_expired = false;
    timer_stop(ctx->reset_detection_timer);
    if (value == LOW) {
        OW_DEBUGF("on_pin_change, starting reset detection timer\n");
        timer_start(ctx->reset_detection_timer, PR_DUR_FORCED_RESET, false);
    }

    sm_push_event(sm_sig, ctx, ((ow_ctx_t *) data)->reset_fn, ((ow_ctx_t *) data)->state, EV_PIN_CHG, value, ctx->ow_debug);
}

void on_not_impl(void *ctx, uint32_t data) {
    printf("not implemented\n");
}

void on_ignored(void *d, uint32_t data) {
    OW_CTX(d);
    OW_DEBUGF("on_ignored\n");
}

static void on_reset_detected(void *d, uint32_t data) {
    OW_CTX(d);
    OW_DEBUGF("on_reset_detected: %d\n", ctx->reset_detection_timer);


    // reset our and owner's context and then set the state as if we're waiting for the reset 
    // pin change from LOW to HIGH. The normal timer is not started.
    ctx->forced_reset_callback(ctx->user_data, OW_ERR_NO_ERROR, 0);
    ow_ctx_reset_state(ctx);
    ctx->state = ST_RESET_WAIT_RELEASE;
}

// ==================== Implementation =========================
static void ow_ctx_reset_cb(void *ctx) { ow_ctx_reset_state((ow_ctx_t *)ctx);}
ow_ctx_t * ow_ctx_init(ow_ctx_cfg_t *cfg)  {
    ow_ctx_t *ctx = calloc(1, sizeof(ow_ctx_t));
    ctx->user_data = cfg->data;
    ctx->reset_callback = cfg->reset_cb;
    ctx->forced_reset_callback = cfg->forced_reset_cb;
    ctx->bit_read_callback = cfg->bit_read_cb;
    ctx->bit_written_callback = cfg->bit_written_cb;

    ctx->reset_fn = ow_ctx_reset_cb;

    timer_config_t timer_cfg = {
            .user_data = ctx
    };
    timer_cfg.callback = on_timer_event;
    ctx->timer = timer_init(&timer_cfg);

    timer_cfg.callback = on_reset_timer_event;
    ctx->reset_detection_timer = timer_init(&timer_cfg);


    ctx->pin = pin_init(cfg->pin_name, INPUT_PULLUP);
    const pin_watch_config_t watch_config = {
            .edge = BOTH,
            .pin_change = on_pin_change,
            .user_data = ctx,
    };
    pin_watch(ctx->pin, &watch_config);


    // read config attributes
    uint32_t attr;

    attr = attr_init("ow_debug", false);
    ctx->ow_debug = attr_read(attr) != 0;

//    attr = attr_init("presence_wait_time", PR_DUR_WAIT_PRESENCE);
//    ctx->presence_wait_time = attr_read(attr);
//    attr = attr_init("presence_time", PR_DUR_PULL_PRESENCE);
//    ctx->presence_time = attr_read(attr);


    ow_ctx_reset_state(ctx);
    OW_DEBUGF("ow_ctx_init\n");

    return ctx;
}


void ow_ctx_reset_state(ow_ctx_t *ctx) {
    OW_DEBUGF("ow_ctx: resetting state from %s\n", ST_NAME(sm_sig_entries))
    ctx->state = ST_RESET_INIT;
    ctx->cur_sm = sm_sig;
    ctx->reset_time = 0;
    ctx->reset_timer_expired = false;

    ctx->slot_start = 0;

    pin_mode(ctx->pin, INPUT_PULLUP);

    timer_stop(ctx->timer);
    timer_stop(ctx->reset_detection_timer);
}

void ow_ctx_set_master_write_state(ow_ctx_t *ctx, bool bit) {
    ctx->bit_buf = bit;
    ctx->state = ST_MASTER_WRITE_INIT;
}

void ow_ctx_set_master_read_state(ow_ctx_t *ctx, bool bit) {
    ctx->bit_buf = bit;
    ctx->state = ST_MASTER_READ_INIT;
}



// --------------- reset state handlers -----------------------

static void on_reset_init_pin_chg(void *d, uint32_t data) {
    OW_CTX(d);
    OW_DEBUGF("on_reset_init_pin_chg - ctx: %p\n", ctx);

    // pin transition to LOW starts a RESET sequence, arm the timer
    if (data == LOW) {
        ctx->reset_time = get_sim_nanos();
        timer_start(ctx->timer, PR_DUR_RESET, false);
        ctx->state = ST_RESET_WAIT_RELEASE;
    } else {
        OW_DEBUGF("L->H transition unexpected during initialisation, ignoring\n");
    }
}

static void on_reset_wait_release_pin_chg(void *d, uint32_t data) {
    OW_CTX(d);
    // unexpected transition, ignore
    if (data == LOW) {
        OW_DEBUGF("H->L transition unexpected while waiting for reset release, ignoring\n");
        return;
    }

    timer_stop(ctx->timer);

    // if pin changed before the expected duration, log and reset
    // note: no need to notify owner, since we're still waiting for reset
    if (TOO_EARLY((ctx->reset_time), _NS(PR_DUR_RESET), PR_DUR_BUS_JITTER)) {
        OW_DEBUGF("L->H transition happened too soon, (%lld) - resetting\n", _US(OW_ELAPSED(ctx->reset_time)));
        ow_ctx_reset_state(ctx);
        return;
    }

    // move to wait for bus to 'stabilise'
    ctx->state = ST_RESET_WAIT_PRESENCE;
    timer_start(ctx->timer, PR_DUR_RESET_MASTER_RELEASE, false);
}

static void on_reset_wait_release_timer_expired(void *d, uint32_t data) {
    OW_CTX(d);
    OW_DEBUGF("on_reset_wait_release_timer_expired: wait for pin change\n");
    // ok, we're ready for the bus to be pulled. Currently, we don't do anything
}


static void on_reset_wait_presence_pin_chg(void *d, uint32_t data) {
    OW_CTX(d);
    // unexpected transition, ignore
    OW_DEBUGF("H->L transition unexpected while waiting for reset release, ignoring\n");
}

static void on_reset_wait_presence_timer_expired(void *d, uint32_t data) {
    OW_CTX(d);
    // we're ready for next phase, delay before presence
    // timer_start(ctx->timer, ctx->presence_wait_time, false);
    ctx->state = ST_RESET_PULL_PRESENCE;
    pin_mode(ctx->pin, OUTPUT_LOW);
    timer_start(ctx->timer, PR_DUR_RESET_PULL_PRESENCE, false);
}


static void on_reset_pull_presence_pin_chg(void *d, uint32_t data) {
    OW_CTX(d);
    // expected transition, ignore
    if (data == HIGH) {
        OW_DEBUGF("L->H transition expected due to pin_mode, ignoring\n");
    }
}

static void on_reset_pull_presence_timer_expired(void *d, uint32_t data) {
    OW_CTX(d);
    // release the bus and wait for master next write (bits)
    pin_mode(ctx->pin, INPUT_PULLUP);
    ctx->state = ST_RESET_DONE;
    timer_start(ctx->timer, PR_DUR_RESET_SLOT_END, false);
}


static void on_reset_done_pin_chg(void *d, uint32_t data) {
    OW_CTX(d);
    // unexpected transition, reset
    if (data == LOW) {
        OW_DEBUGF("H->L transition unexpected when in done state, resetting\n");
        ow_ctx_reset_state(ctx);
        return;
    }

    // after reset is done, the master will write the next command.
    // set the state accordingly, but allow the callback to override if desired
    ctx->state = ST_MASTER_WRITE_INIT;
    ctx->reset_callback(ctx->user_data, OW_ERR_NO_ERROR, 0);
}


static void on_reset_done_timer_expired(void *d, uint32_t data) {
    OW_CTX(d);

    // after reset is done, the master will write the next command.
    // set the state accordingly, but allow the callback to override if desired
    ctx->state = ST_MASTER_WRITE_INIT;
    ctx->reset_callback(ctx->user_data, OW_ERR_NO_ERROR, 0);
}


// --------------- write bit state handlers -----------------------


static void on_master_write_init_pin_chg(void *d, uint32_t data) {
    OW_CTX(d);

    if (data == HIGH) {
        OW_DEBUGF("L->H transition unexpected while waiting for initial pull down during write time slot, resetting\n");
        ow_ctx_reset_state(ctx);
        return;
    }
    ctx->state = ST_MASTER_WRITE_WAIT_SAMPLE;
    ctx->slot_start = get_sim_nanos();
    timer_start(ctx->timer, PR_DUR_SAMPLE_WAIT, false);
}

static void on_master_write_wait_sample_pin_chg(void *d, uint32_t data) {
    OW_CTX(d);

    // todo(bonnyr): this needs to be confirmed as unexpected
    if (data == LOW) {
        OW_DEBUGF("H->L transition unexpected while waiting for bus release during write time slot, resetting\n");
        ow_ctx_reset_state(ctx);
        return;
    }


    // ignore bit transition to HIGH  - this is the master releasing the bus. 
    // we wait for the timer to sample and then continue

    // todo(bonnyr): consider starting a protection timer
}

static void on_master_write_wait_sample_timer_expired(void *d, uint32_t data) {
    OW_CTX(d);

    ctx->bit_buf = pin_read(ctx->pin);
    timer_start(ctx->timer, PR_DUR_WRITE_SLOT_END, false);
    ctx->state = ST_MASTER_WRITE_SLOT_END;
}


static void on_master_write_wait_slot_end_pin_chg(void *d, uint32_t data) {
    OW_CTX(d);
    if (data == LOW) {
        OW_DEBUGF("H->L transition unexpected while waiting for bus release during write slot end, ignoring\n");
    }
}

static void on_master_write_wait_slot_end_timer_expired(void *d, uint32_t data) {
    OW_CTX(d);

    // we're done
    if (pin_read(ctx->pin) == LOW) {
        ctx->state = ST_MASTER_WRITE_DONE;
        return;
    }
    ctx->state = ST_MASTER_WRITE_INIT;
    ctx->bit_read_callback(ctx->user_data, OW_ERR_NO_ERROR, ctx->bit_buf);
    OW_DEBUGF("on_master_write_wait_slot_end_timer_expired: ctx: %p, ctx->state after callback %d (was set by us to %d)\n", ctx, ctx->state, ST_MASTER_WRITE_INIT);
}


static void on_master_write_done_pin_chg(void *d, uint32_t data) {
    OW_CTX(d);

    if (data == LOW) {
        OW_DEBUGF("H->L transition unexpected while waiting for bus release during write slot end, ignoring\n");
        ow_ctx_reset_state(ctx);
        return;
    }

    ctx->state = ST_MASTER_WRITE_INIT;
    ctx->bit_read_callback(ctx->user_data, OW_ERR_NO_ERROR, ctx->bit_buf);
    OW_DEBUGF("on_master_write_done_pin_chg: ctx->state after callback %d (was set by us to %d)\n", ST_MASTER_WRITE_INIT, ctx->state);

}




// --------------- read bit state handlers -----------------------


static void on_master_read_init_pin_chg(void *d, uint32_t data) {
    OW_CTX(d);

    if (data == HIGH) {
        OW_DEBUGF("L->H transition unexpected while waiting for initial pull down during write time slot, resetting\n");
        ow_ctx_reset_state(ctx);
        return;
    }

    // pin dropped low, we need to wait >1us (but since we need to start pulling while the bus is also pulling, we'll do this for 1us)
    ctx->state = ST_MASTER_READ_WAIT_SAMPLE;
    ctx->slot_start = get_sim_nanos();
    timer_start(ctx->timer, PR_DUR_READ_INIT, false);
}


static void write_next_bit(ow_ctx_t *ctx) {

    if (!ctx->bit_buf) {
        pin_mode(ctx->pin, OUTPUT_LOW);

        timer_start(ctx->timer, PR_DUR_READ_SLOT - OW_ELAPSED_US(ctx->slot_start), false);
        ctx->state = ST_MASTER_READ_SLOT_END;
    } else {
        ctx->state = ST_MASTER_READ_DONE;
    }
}

// this should not happen, but just in case, write the bit anyway
static void on_master_read_wait_sample_pin_chg(void *d, uint32_t data) {
    OW_CTX(d);

    // todo(bonnyr): this needs to be confirmed as unexpected
    if (data == LOW) {
        OW_DEBUGF("H->L transition unexpected while waiting for bus release during write time slot, resetting\n");
        ow_ctx_reset_state(ctx);
        return;
    }

    // bus has been released, we're ready to write the next bit we need to
    timer_stop(ctx->timer);

    write_next_bit(ctx);

}

static void on_master_read_wait_sample_timer_expired(void *ctx, uint32_t data) {
    write_next_bit(ctx);
}


static void on_master_read_slot_end_pin_chg(void *d, uint32_t data) {
    OW_CTX(d);

    // todo(bonnyr): this needs to be confirmed as unexpected
    if (data == HIGH && !ctx->bit_buf || data == LOW && ctx->bit_buf) {
        OW_DEBUGF("L->H or H->L transition unexpected while waiting for READ slot timer, resetting\n");
        ow_ctx_reset_state(ctx);
        return;
    }

}

static void on_master_read_slot_end_timer_expired(void *d, uint32_t data) {
    OW_CTX(d);

    ctx->state = ST_MASTER_READ_DONE;
    if (!ctx->bit_buf) {
        pin_mode(ctx->pin, INPUT_PULLUP);
    }
}


static void on_master_read_done_pin_chg(void *d, uint32_t data) {
    OW_CTX(d);

    if (data == LOW) {
        OW_DEBUGF("H->L transition unexpected while waiting for READ slot bus release, resetting\n");
        ow_ctx_reset_state(ctx);
        return;
    }

    ctx->state = ST_MASTER_READ_INIT;
    ctx->bit_written_callback(ctx->user_data, OW_ERR_NO_ERROR, ctx->bit_buf);
}
