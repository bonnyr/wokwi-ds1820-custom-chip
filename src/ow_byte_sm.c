#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>

#include "wokwi-api.h"
#include "hashmap.h"
#include "ow.h"


ow_byte_ctx_t *ow_read_byte_init(void *data, byte_cb cb, ow_ctx_t *ow_ctx);
void ow_read_byte_ctx_reset_state(ow_byte_ctx_t *ctx);
ow_byte_ctx_t *ow_write_byte_init(void *data, byte_cb cb, ow_ctx_t *ow_ctx);


static void on_read_byte_running_bit_written(void *d, uint32_t data);
static void on_write_byte_running_bit_read(void *d, uint32_t data);


sm_entry_t sm_write_byte_entries[] = { //[ST_RESET_MAX][EV_MAX]
        // ST_WRITE_RUNNING
        SM_E(ST_WRITE_RUNNING, EV_BIT_READ, on_write_byte_running_bit_read),
        SM_E(ST_WRITE_RUNNING, EV_BIT_WRITTEN, on_not_impl),

        // ST_WRITE_RUNNING
        SM_E(ST_WRITE_DONE, EV_BIT_READ, on_not_impl),
        SM_E(ST_WRITE_DONE, EV_BIT_WRITTEN, on_not_impl),
};

sm_cfg_t sm_write_byte_cfg = {
        .name = "sm_write_byte",
        .sm_entries = sm_write_byte_entries,
        .num_entries = sizeof sm_write_byte_entries/sizeof(sm_entry_t),
};

sm_entry_t sm_read_byte_entries[] = { //[ST_RESET_MAX][EV_MAX]
        // ST_READ_RUNNING
        SM_E(ST_READ_RUNNING, EV_BIT_READ, on_not_impl),
        SM_E(ST_READ_RUNNING, EV_BIT_WRITTEN, on_read_byte_running_bit_written),

        // ST_READ_DONE
        SM_E(ST_READ_DONE, EV_BIT_READ, on_not_impl),
        SM_E(ST_READ_DONE, EV_BIT_WRITTEN, on_not_impl),
};

sm_cfg_t sm_read_byte_cfg = {
        .name = "sm_read_byte",
        .sm_entries = sm_read_byte_entries,
        .num_entries = sizeof(sm_read_byte_entries)/sizeof(sm_entry_t)
};


sm_t *sm_read_byte  = &(sm_t){.cfg = &sm_read_byte_cfg};
sm_t *sm_write_byte  = &(sm_t){.cfg = &sm_write_byte_cfg};


// --------------- read/write byte state handlers -----------------------
void ow_read_byte_bit_written_cb(void *d, uint32_t err, uint32_t data) {
    ow_byte_ctx_t *ctx = d;
    OW_DEBUGF("ow_read_byte_bit_written_cb: %d\n", data);

    if (err != 0) {
        OW_DEBUGF("byte read: Error occurred while waiting for bit to be read")
        ow_read_byte_ctx_reset_state(ctx);
        return;
    }

    // update state first
    ctx->state = ST_READ_RUNNING;

    sm_push_event(sm_read_byte, ctx, ctx->reset_fn, ctx->state, EV_BIT_WRITTEN, data, ctx->owDebug);
}


static void ow_read_byte_reset_cb(void *ctx) { ow_read_byte_ctx_reset_state((ow_byte_ctx_t *) ctx);}
ow_byte_ctx_t *ow_read_byte_ctx_init(void *data, byte_cb cb, ow_ctx_t *ow_ctx) {
    ow_byte_ctx_t *ctx = calloc(1, sizeof(ow_byte_ctx_t));

    ctx->bit_callback = ow_read_byte_bit_written_cb;
    ctx->callback = cb;
    ctx->user_data = data;
    ctx->reset_fn = ow_read_byte_reset_cb;
    ctx->ow_ctx = ow_ctx;

    uint32_t attr;

    attr = attr_init("owDebug", false);
    ctx->owDebug = attr_read(attr) != 0;
    return ctx;
}
void ow_read_byte_ctx_reset_state(ow_byte_ctx_t *ctx) {
    OW_DEBUGF("read_byte: resetting state from %s\n", ST_NAME(sm_read_byte_entries))
    ctx->state = ST_READ_RUNNING;
    ctx->bit_ndx = 0;
    ctx->byte_buf = 0;
}

static void on_read_byte_running_bit_written(void *d, uint32_t data) {
    ow_byte_ctx_t *ctx = d;
    OW_DEBUGF("read_byte: on_read_byte_running_bit_written: enter\n");

    ctx->byte_buf |= (data & 0x1) << ctx->bit_ndx;
    ctx->bit_ndx++;

    OW_DEBUGF("read_byte: on_read_byte_running_bit_written: bit %d=%d => %02x\n", ctx->bit_ndx, data, ctx->byte_buf);
    // check if we're done
    if (ctx->bit_ndx == 8) {
        uint8_t  byte_buf = ctx->byte_buf;
        ow_read_byte_ctx_reset_state(ctx);
        ctx->callback(ctx->user_data, OW_ERR_NO_ERROR, byte_buf);
    }
}



void ow_write_byte_bit_read_cb(void *d, uint32_t err, uint32_t data) {
    ow_byte_ctx_t *ctx = d;
    OW_DEBUGF("ow_read_byte_bit_written_cb: %d\n", data);

    if (err != 0) {
        OW_DEBUGF("write byte: Error occurred while waiting for bit to be read")
        ow_write_byte_ctx_reset_state(ctx, 0);
        return;
    }

    // update state first
    ctx->state = ST_WRITE_RUNNING;

    sm_push_event(sm_write_byte, ctx, ctx->reset_fn, ctx->state, EV_BIT_READ, data, ctx->owDebug);
}

static void ow_write_byte_reset_cb(void *ctx) { ow_write_byte_ctx_reset_state((ow_byte_ctx_t *) ctx, 0);}
ow_byte_ctx_t *ow_write_byte_ctx_init(void *data, byte_cb cb, ow_ctx_t *ow_ctx) {
    ow_byte_ctx_t *ctx = calloc(1, sizeof(ow_byte_ctx_t));

    ctx->bit_callback = ow_write_byte_bit_read_cb;
    ctx->callback = cb;
    ctx->user_data = data;
    ctx->reset_fn = ow_write_byte_reset_cb;
    ctx->ow_ctx = ow_ctx;

    uint32_t attr;

    attr = attr_init("owDebug", false);
    ctx->owDebug = attr_read(attr) != 0;
    return ctx;
}

void ow_write_byte_ctx_reset_state(ow_byte_ctx_t *ctx, uint8_t byte_buf) {
    OW_DEBUGF("write_byte: resetting state from %s, new byte: %02x\n", ST_NAME(sm_write_byte_entries), byte_buf);
    ctx->state = ST_WRITE_RUNNING;
    ctx->bit_ndx = 0;
    ctx->byte_buf = byte_buf;
    
    ctx->ow_ctx->bit_buf = ctx->byte_buf & 0x01;
}


static void on_write_byte_running_bit_read(void *d, uint32_t data) {
    ow_byte_ctx_t *ctx = d;

    ctx->bit_ndx++;
    ctx->ow_ctx->bit_buf = ctx->byte_buf & (1 << ctx->bit_ndx);

    // check if we're done
    if (ctx->bit_ndx == 8) {
        ow_write_byte_ctx_reset_state(ctx, 0);
        ctx->callback(ctx->user_data, OW_ERR_NO_ERROR, ctx->byte_buf);
    }
}
