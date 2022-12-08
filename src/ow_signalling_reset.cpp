//
// Created by bonny on 5/12/2022.
//



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



