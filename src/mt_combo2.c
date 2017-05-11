/*The standard main function for bitcount modified to work with the new, but not newest
 * version of maker... 
 */
#include <msp430.h>
#include <stdlib.h>

//Added following libs, commented out the next block
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include <libio/log.h>
#include <libmsp/mem.h>
#include <libwispbase/wisp-base.h>
#include <libchain/chain.h>
//#include <libio/log.h>
//#include <libalpaca/alpaca.h>
//#include <libmspbuiltins/builtins.h>
//#include <libmsp/mem.h>
#include <libmsp/periph.h>
#include <libmsp/clock.h>
#include <libmsp/watchdog.h>
#include <libmsp/gpio.h>
#include <libmspmath/msp-math.h>

#ifdef CONFIG_LIBEDB_PRINTF
#include <libedb/edb.h>
#endif

//Add for threaded version
#include <libchain/thread.h>
#include <libchain/mutex.h>

#include "pins.h"
/*----------------------------------Begin blinker init stuff------------------*/


#define INIT_TASK_DURATION_ITERS  400000
#define TASK_START_DURATION_ITERS 1600000
#define BLINK_DURATION_ITERS      400000
#define WAIT_TICK_DURATION_ITERS  300000
#define NUM_BLINKS_PER_TASK       5
#define WAIT_TICKS                3

// If you link-in wisp-base, then you have to define some symbols.
uint8_t usrBank[USRBANK_SIZE];

struct msg_blinks {
    CHAN_FIELD(unsigned, blinks);
};

struct msg_tick {
    CHAN_FIELD(unsigned, tick);
};

struct msg_self_tick {
    SELF_CHAN_FIELD(unsigned, tick);
};
#define FIELD_INIT_msg_self_tick { \
    SELF_FIELD_INITIALIZER \
}

struct msg_duty_cycle {
    CHAN_FIELD(unsigned, duty_cycle);
};

TASK(24, task_1_r)
TASK(25, task_2_r)
TASK(26, task_3_r)

TASK(27, task_1_g)
TASK(28, task_2_g)
TASK(29, task_3_g)

/*CHANNEL(task_init, task_1, msg_blinks);
CHANNEL(task_init, task_3, msg_tick);
CHANNEL(task_1, task_2, msg_blinks);
CHANNEL(task_2, task_1, msg_blinks);*/
CHANNEL_WT(task_init, task_1_r, 0, msg_blinks);
CHANNEL_WT(task_init, task_3_r, 0,  msg_tick);
CHANNEL_WT(task_1_r, task_2_r, 0, msg_blinks);
CHANNEL_WT(task_2_r, task_1_r, 0, msg_blinks);

CHANNEL_WT(task_init, task_1_g, 0, msg_blinks); 
CHANNEL_WT(task_init, task_3_g, 0, msg_tick); 
CHANNEL_WT(task_1_g, task_2_g, 0, msg_blinks); 
CHANNEL_WT(task_2_g, task_1_g, 0, msg_blinks); 

SELF_CHANNEL(task_3_g, msg_self_tick); 
SELF_CHANNEL(task_3_r, msg_self_tick);

MULTICAST_CHANNEL(msg_duty_cycle, ch_duty_cycle_r, task_init, task_1_r, task_2_r);
MULTICAST_CHANNEL(msg_duty_cycle, ch_duty_cycle_g, task_init, task_1_g, task_2_g);

volatile unsigned work_x;


/*----------------------------------Now bitcount init stuff--------------------*/
#define SEED 4L
#define ITER 10
#define CHAR_BIT 8

__nv static char bits_arr[256] =
{
      0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,  /* 0   - 15  */
      1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,  /* 16  - 31  */
      1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,  /* 32  - 47  */
      2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,  /* 48  - 63  */
      1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,  /* 64  - 79  */
      2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,  /* 80  - 95  */
      2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,  /* 96  - 111 */
      3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,  /* 112 - 127 */
      1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,  /* 128 - 143 */
      2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,  /* 144 - 159 */
      2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,  /* 160 - 175 */
      3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,  /* 176 - 191 */
      2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,  /* 192 - 207 */
      3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,  /* 208 - 223 */
      3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,  /* 224 - 239 */
      4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8   /* 240 - 255 */
};

struct c0 {
	CHAN_FIELD(unsigned, func);
};
struct c1 {
	CHAN_FIELD(unsigned, n);
};
struct c2 {
	CHAN_FIELD(unsigned, iter);
	CHAN_FIELD(uint32_t, seed);
};

struct c3 {
	SELF_CHAN_FIELD(unsigned, func);
};
#define FIELD_INIT_c3 {\
	SELF_FIELD_INITIALIZER\
}

struct c4 {
	SELF_CHAN_FIELD(unsigned, iter);
	SELF_CHAN_FIELD(uint32_t, seed);
	SELF_CHAN_FIELD(unsigned, n);
};
#define FIELD_INIT_c4 {\
	SELF_FIELD_INITIALIZER,\
	SELF_FIELD_INITIALIZER,\
	SELF_FIELD_INITIALIZER\
}

struct c5 {
	CHAN_FIELD(unsigned, n);
};

struct c6 {
	CHAN_FIELD(unsigned, n);
};
struct c7 {
	CHAN_FIELD(unsigned, iter);
	CHAN_FIELD(uint32_t, seed);
};

struct c8 {
	SELF_CHAN_FIELD(unsigned, iter);
	SELF_CHAN_FIELD(uint32_t, seed);
	SELF_CHAN_FIELD(unsigned, n);
};
#define FIELD_INIT_c8 {\
	SELF_FIELD_INITIALIZER,\
	SELF_FIELD_INITIALIZER,\
	SELF_FIELD_INITIALIZER\
}

struct c9 {
	CHAN_FIELD(unsigned, n);
};

struct c10 {
	CHAN_FIELD(unsigned, n);
};
struct c11 {
	CHAN_FIELD(unsigned, iter);
	CHAN_FIELD(uint32_t, seed);
};

struct c12 {
	SELF_CHAN_FIELD(unsigned, iter);
	SELF_CHAN_FIELD(uint32_t, seed);
	SELF_CHAN_FIELD(unsigned, n);
};
#define FIELD_INIT_c12 {\
	SELF_FIELD_INITIALIZER,\
	SELF_FIELD_INITIALIZER,\
	SELF_FIELD_INITIALIZER\
}

struct c13 {
	CHAN_FIELD(unsigned, n);
};

struct c14 {
	CHAN_FIELD(unsigned, n);
};
struct c15 {
	CHAN_FIELD(unsigned, iter);
	CHAN_FIELD(uint32_t, seed);
};

struct c16 {
	SELF_CHAN_FIELD(unsigned, iter);
	SELF_CHAN_FIELD(uint32_t, seed);
	SELF_CHAN_FIELD(unsigned, n);
};
#define FIELD_INIT_c16 {\
	SELF_FIELD_INITIALIZER,\
	SELF_FIELD_INITIALIZER,\
	SELF_FIELD_INITIALIZER\
}

struct c17 {
	CHAN_FIELD(unsigned, n);
};

struct c18 {
	CHAN_FIELD(unsigned, n);
};
struct c19 {
	CHAN_FIELD(unsigned, iter);
	CHAN_FIELD(uint32_t, seed);
};

struct c20 {
	SELF_CHAN_FIELD(unsigned, iter);
	SELF_CHAN_FIELD(uint32_t, seed);
	SELF_CHAN_FIELD(unsigned, n);
};
#define FIELD_INIT_c20 {\
	SELF_FIELD_INITIALIZER,\
	SELF_FIELD_INITIALIZER,\
	SELF_FIELD_INITIALIZER\
}

struct c21 {
	CHAN_FIELD(unsigned, n);
};

struct c22 {
	CHAN_FIELD(unsigned, n);
};
struct c23 {
	CHAN_FIELD(unsigned, iter);
	CHAN_FIELD(uint32_t, seed);
};

struct c24 {
	SELF_CHAN_FIELD(unsigned, iter);
	SELF_CHAN_FIELD(uint32_t, seed);
	SELF_CHAN_FIELD(unsigned, n);
};
#define FIELD_INIT_c24 {\
	SELF_FIELD_INITIALIZER,\
	SELF_FIELD_INITIALIZER,\
	SELF_FIELD_INITIALIZER\
}

struct c25 {
	CHAN_FIELD(unsigned, n);
};

struct c26 {
	CHAN_FIELD(unsigned, n);
};
struct c27 {
	CHAN_FIELD(unsigned, iter);
	CHAN_FIELD(uint32_t, seed);
};

struct c28 {
	SELF_CHAN_FIELD(unsigned, iter);
	SELF_CHAN_FIELD(uint32_t, seed);
	SELF_CHAN_FIELD(unsigned, n);
};
#define FIELD_INIT_c28 {\
	SELF_FIELD_INITIALIZER,\
	SELF_FIELD_INITIALIZER,\
	SELF_FIELD_INITIALIZER\
}

struct c29 {
	CHAN_FIELD(unsigned, n);
};

CHANNEL(task_init, task_select_func, c0);
CHANNEL(task_init, task_bit_count, c1);
CHANNEL(task_select_func, task_bit_count, c2);
SELF_CHANNEL(task_select_func, c3);
SELF_CHANNEL(task_bit_count, c4);
CHANNEL(task_bit_count, task_end, c5);

CHANNEL(task_init, task_bitcount, c6);
CHANNEL(task_select_func, task_bitcount, c7);
SELF_CHANNEL(task_bitcount, c8);
CHANNEL(task_bitcount, task_end, c9);

CHANNEL(task_init, task_ntbl_bitcnt, c10);
CHANNEL(task_select_func, task_ntbl_bitcnt, c11);
SELF_CHANNEL(task_ntbl_bitcnt, c12);
CHANNEL(task_ntbl_bitcnt, task_end, c13);

CHANNEL(task_init, task_ntbl_bitcount, c14);
CHANNEL(task_select_func, task_ntbl_bitcount, c15);
SELF_CHANNEL(task_ntbl_bitcount, c16);
CHANNEL(task_ntbl_bitcount, task_end, c17);

CHANNEL(task_init, task_BW_btbl_bitcount, c18);
CHANNEL(task_select_func, task_BW_btbl_bitcount, c19);
SELF_CHANNEL(task_BW_btbl_bitcount, c20);
CHANNEL(task_BW_btbl_bitcount, task_end, c21);

CHANNEL(task_init, task_AR_btbl_bitcount, c22);
CHANNEL(task_select_func, task_AR_btbl_bitcount, c23);
SELF_CHANNEL(task_AR_btbl_bitcount, c24);
CHANNEL(task_AR_btbl_bitcount, task_end, c25);

CHANNEL(task_init, task_bit_shifter, c26);
CHANNEL(task_select_func, task_bit_shifter, c27);
SELF_CHANNEL(task_bit_shifter, c28);
CHANNEL(task_bit_shifter, task_end, c29);

TASK(1, task_init)
TASK(2, task_select_func)
TASK(3, task_bit_count)
TASK(4, task_bitcount)
TASK(5, task_ntbl_bitcnt)
TASK(6, task_ntbl_bitcount)
TASK(7, task_BW_btbl_bitcount)
TASK(8, task_AR_btbl_bitcount)
TASK(9, task_bit_shifter)
TASK(10, task_end)

/*Commented out for now
static void init_hw()
{
	msp_watchdog_disable();
	msp_gpio_unlock();
	msp_clock_setup();
}
*/

/*---------------------------------Cuckoo app -------------------------------------*/

/*--------------------------cuckoo defs and channels-----------------------------*/
#define NUM_INSERTS (NUM_BUCKETS / 4) // shoot for 25% occupancy
#define NUM_LOOKUPS NUM_INSERTS
#define NUM_BUCKETS 256//256 // must be a power of 2
#define MAX_RELOCATIONS 8

typedef uint16_t value_t;
typedef uint16_t hash_t;
typedef uint16_t fingerprint_t;
typedef uint16_t index_t; // bucket index

typedef struct _insert_count {
    unsigned insert_count;
    unsigned inserted_count;
} insert_count_t;

typedef struct _lookup_count {
    unsigned lookup_count;
    unsigned member_count;
} lookup_count_t;

struct msg_key {
    CHAN_FIELD(value_t, key);
};

struct msg_genkey {
    CHAN_FIELD(value_t, key);
    CHAN_FIELD(task_t*, next_task);
};

struct msg_calc_indexes {
    CHAN_FIELD(value_t, key);
    CHAN_FIELD(task_t*, next_task);
};

struct msg_self_key {
    SELF_CHAN_FIELD(value_t, key);
};
#define FIELD_INIT_msg_self_key {\
    SELF_FIELD_INITIALIZER \
}

struct msg_indexes {
    CHAN_FIELD(fingerprint_t, fingerprint);
    CHAN_FIELD(index_t, index1);
    CHAN_FIELD(index_t, index2);
};

struct msg_fingerprint {
    CHAN_FIELD(fingerprint_t, fingerprint);
};

struct msg_index1 {
    CHAN_FIELD(index_t, index1);
};

struct msg_filter {
    CHAN_FIELD_ARRAY(fingerprint_t, filter, NUM_BUCKETS);
};

struct msg_self_filter {
    SELF_CHAN_FIELD_ARRAY(fingerprint_t, filter, NUM_BUCKETS);
};
#define FIELD_INIT_msg_self_filter { \
    SELF_FIELD_ARRAY_INITIALIZER(NUM_BUCKETS) \
}

struct msg_filter_insert_done {
    CHAN_FIELD_ARRAY(fingerprint_t, filter, NUM_BUCKETS);
    CHAN_FIELD(bool, success);
};

struct msg_victim {
    CHAN_FIELD_ARRAY(fingerprint_t, filter, NUM_BUCKETS);
    CHAN_FIELD(fingerprint_t, fp_victim);
    CHAN_FIELD(index_t, index_victim);
    CHAN_FIELD(unsigned, relocation_count);
};

struct msg_self_victim {
    SELF_CHAN_FIELD_ARRAY(fingerprint_t, filter, NUM_BUCKETS);
    SELF_CHAN_FIELD(fingerprint_t, fp_victim);
    SELF_CHAN_FIELD(index_t, index_victim);
    SELF_CHAN_FIELD(unsigned, relocation_count);
};
#define FIELD_INIT_msg_self_victim { \
    SELF_FIELD_ARRAY_INITIALIZER(NUM_BUCKETS), \
    SELF_FIELD_INITIALIZER, \
    SELF_FIELD_INITIALIZER, \
    SELF_FIELD_INITIALIZER \
}

struct msg_hash_args {
    CHAN_FIELD(value_t, data);
    CHAN_FIELD(task_t*, next_task);
};

struct msg_hash {
    CHAN_FIELD(hash_t, hash);
};

struct msg_member {
    CHAN_FIELD(bool, member);
};

struct msg_lookup_result {
    CHAN_FIELD(value_t, key);
    CHAN_FIELD(bool, member);
};

struct msg_self_insert_count {
    SELF_CHAN_FIELD(unsigned, insert_count);
    SELF_CHAN_FIELD(unsigned, inserted_count);
};
#define FIELD_INIT_msg_self_insert_count {\
    SELF_FIELD_INITIALIZER, \
    SELF_FIELD_INITIALIZER \
}

struct msg_self_lookup_count {
    SELF_CHAN_FIELD(unsigned, lookup_count);
    SELF_CHAN_FIELD(unsigned, member_count);
};
#define FIELD_INIT_msg_self_lookup_count {\
    SELF_FIELD_INITIALIZER, \
    SELF_FIELD_INITIALIZER \
}

struct msg_insert_count {
    CHAN_FIELD(unsigned, insert_count);
    CHAN_FIELD(unsigned, inserted_count);
};

struct msg_lookup_count {
    CHAN_FIELD(unsigned, lookup_count);
    CHAN_FIELD(unsigned, member_count);
};

struct msg_inserted_count {
    CHAN_FIELD(unsigned, inserted_count);
};

struct msg_member_count {
    CHAN_FIELD(unsigned, member_count);
};

TASK(11,  task_generate_key)
TASK(12,  task_insert)
TASK(13,  task_calc_indexes)
TASK(14,  task_calc_indexes_index_1)
TASK(15,  task_calc_indexes_index_2)
TASK(16,  task_add) // TODO: rename: add 'insert' prefix
TASK(17,  task_relocate)
TASK(18,  task_insert_done)
TASK(19, task_lookup)
TASK(20, task_lookup_search)
TASK(21, task_lookup_done)
TASK(22, task_print_stats)
TASK(23, task_done)

CHANNEL(task_init, task_generate_key, msg_genkey);
CHANNEL(task_init, task_insert_done, msg_insert_count);
CHANNEL(task_init, task_lookup_done, msg_lookup_count);
MULTICAST_CHANNEL(msg_key, ch_key, task_generate_key, task_insert, task_lookup);
SELF_CHANNEL(task_insert, msg_self_key);
MULTICAST_CHANNEL(msg_filter, ch_filter, task_init,
                  task_add, task_relocate, task_insert_done,
                  task_lookup_search, task_print_stats);
MULTICAST_CHANNEL(msg_filter, ch_filter_add, task_add,
                  tsk_relocate, task_insert_done, task_lookup_search,
                  task_print_stats);
MULTICAST_CHANNEL(msg_filter, ch_filter_relocate, task_relocate,
                  task_add, task_insert_done, task_lookup_search,
                  task_print_stats);
CALL_CHANNEL(ch_calc_indexes, msg_calc_indexes);
RET_CHANNEL(ch_calc_indexes, msg_indexes);
CHANNEL(task_calc_indexes, task_calc_indexes_index_2, msg_fingerprint);
CHANNEL(task_calc_indexes_index_1, task_calc_indexes_index_2, msg_index1);
CHANNEL(task_add, task_relocate, msg_victim);
SELF_CHANNEL(task_add, msg_self_filter);
CHANNEL(task_add, task_insert_done, msg_filter_insert_done);
MULTICAST_CHANNEL(msg_filter, ch_reloc_filter, task_relocate,
                  task_add, task_insert_done);
SELF_CHANNEL(task_relocate, msg_self_victim);
CHANNEL(task_relocate, task_add, msg_filter);
CHANNEL(task_relocate, task_insert_done, msg_filter_insert_done);
CHANNEL(task_lookup, task_lookup_done, msg_lookup_result);
SELF_CHANNEL(task_insert_done, msg_self_insert_count);
SELF_CHANNEL(task_lookup_done, msg_self_lookup_count);
CHANNEL(task_insert_done, task_generate_key, msg_genkey);
CHANNEL(task_lookup_done, task_generate_key, msg_genkey);
CHANNEL(task_insert_done, task_print_stats, msg_inserted_count);
CHANNEL(task_lookup_done, task_print_stats, msg_member_count);
SELF_CHANNEL(task_generate_key, msg_self_key);
CHANNEL(task_lookup_search, task_lookup_done, msg_member);

/*--------------------------cuckoo inits and functions----------------------------*/

static value_t init_key = 0x0001; // seeds the pseudo-random sequence of keys

static hash_t djb_hash(uint8_t* data, unsigned len)
{
   uint32_t hash = 5381;
   unsigned int i;

   for(i = 0; i < len; data++, i++)
      hash = ((hash << 5) + hash) + (*data);

   return hash & 0xFFFF;
}

static index_t hash_to_index(fingerprint_t fp)
{
    hash_t hash = djb_hash((uint8_t *)&fp, sizeof(fingerprint_t));
    return hash & (NUM_BUCKETS - 1); // NUM_BUCKETS must be power of 2
}

static fingerprint_t hash_to_fingerprint(value_t key)
{
    return djb_hash((uint8_t *)&key, sizeof(value_t));
}

void init() {
    WISP_init();
#ifdef CONFIG_EDB
    debug_setup();
#endif
/*
    INIT_CONSOLE();
#ifndef BOARD_CAPYBARA
    GPIO(PORT_AUX, DIR)   |= BIT(PIN_AUX_1); 
    GPIO(PORT_LED_1, DIR) |= BIT(PIN_LED_1);
    GPIO(PORT_LED_2, DIR) |= BIT(PIN_LED_2);
#if defined(PORT_LED_3)
        GPIO(PORT_LED_3, DIR) |= BIT(PIN_LED_3);
#endif
#endif
  */ 
    INIT_CONSOLE(); 
    __enable_interrupt();
/*
#if defined(PORT_LED_3) // when available, this LED indicates power-on
    GPIO(PORT_LED_3, OUT) |= BIT(PIN_LED_3);
    GPIO(PORT_LED_1, OUT) &= ~BIT(PIN_LED_1); 
    GPIO(PORT_AUX, OUT)   &= ~BIT(PIN_AUX_1); 
#endif

#if defined(PORT_DEBUG)
    GPIO(PORT_DEBUG, DIR) |= BIT(PIN_DEBUG_1) ; 
    GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG_1); 
#endif
  */
    PRINTF(".%u.\r\n", curctx->task->idx);
}

void task_init() {
	LOG("init\r\n");
  task_prologue();
  unsigned i;
  /*--------------------------thread_init call!!-----------------------------*/
  thread_init(); 

  /*-----------------------Cuckoo app init start-----------------------------*/

  LOG("init\r\n");

  for (i = 0; i < NUM_BUCKETS; ++i) {
      fingerprint_t fp = 0;
      CHAN_OUT1(fingerprint_t, filter[i], fp, MC_OUT_CH(ch_filter, task_init,
                             task_add, task_relocate, task_insert_done,
                             task_lookup_search, task_print_stats));
  }

    unsigned count = 0;
    CHAN_OUT1(unsigned, insert_count, count, CH(task_init, task_insert_done));
    CHAN_OUT1(unsigned, lookup_count, count, CH(task_init, task_lookup_done));

    CHAN_OUT1(unsigned, inserted_count, count, CH(task_init, task_insert_done));
    CHAN_OUT1(unsigned, member_count, count, CH(task_init, task_lookup_done));

    CHAN_OUT1(value_t, key, init_key, CH(task_init, task_generate_key));
    task_t *next_task = TASK_REF(task_insert);
    CHAN_OUT1(task_t *, next_task, next_task, CH(task_init, task_generate_key));

  /*----------------------bitcount init start-----------------------------*/
  unsigned func = 0;
	unsigned n = 0;
  thread_init(); 
	CHAN_OUT1(unsigned, func, func, CH(task_init, task_select_func));
	CHAN_OUT1(unsigned, n, n, CH(task_init, task_bit_count));
	CHAN_OUT1(unsigned, n, n, CH(task_init, task_bitcount));
	CHAN_OUT1(unsigned, n, n, CH(task_init, task_ntbl_bitcnt));
	CHAN_OUT1(unsigned, n, n, CH(task_init, task_ntbl_bitcount));
	CHAN_OUT1(unsigned, n, n, CH(task_init, task_BW_btbl_bitcount));
	CHAN_OUT1(unsigned, n, n, CH(task_init, task_AR_btbl_bitcount));
	CHAN_OUT1(unsigned, n, n, CH(task_init, task_bit_shifter));
  /*-----------------------both blinker inits start--------------------------*/
  unsigned blinks = NUM_BLINKS_PER_TASK;
  CHAN_OUT1(unsigned, blinks, blinks, CH_TH(task_init, task_1_r, 0));
  unsigned tick = 0;
  CHAN_OUT1(unsigned, tick, tick, CH_TH(task_init, task_3_r, 0));
  unsigned duty_cycle = 75;
  CHAN_OUT1(unsigned, duty_cycle, duty_cycle,
           MC_OUT_CH(ch_duty_cycle_r, task_init, task_1_r, task_2_r));
  
  CHAN_OUT1(unsigned, blinks, blinks, CH_TH(task_init, task_1_g, 0));
  CHAN_OUT1(unsigned, tick, tick, CH_TH(task_init, task_3_g, 0));
  CHAN_OUT1(unsigned, duty_cycle, duty_cycle,
           MC_OUT_CH(ch_duty_cycle_g, task_init, task_1_g, task_2_g));


  LOG("LED\r\n");

/*-----------------------THREAD_CREATE calls to separate programs--------------------------*/
  THREAD_CREATE(task_3_g); 
  THREAD_CREATE(task_3_r); 
  THREAD_CREATE(task_generate_key); 
  THREAD_CREATE(task_select_func); 
  TRANSITION_TO_MT(task_select_func);
}

void task_select_func() {
	LOG("select func\r\n");

	unsigned func = *CHAN_IN2(unsigned, func, SELF_IN_CH(task_select_func),
			CH(task_init, task_select_func));
	uint32_t seed = (uint32_t)SEED; // for test, seed is always the same
	unsigned iter = 0;
	LOG("func: %u\r\n", func);
	if(func == 0){
		CHAN_OUT1(unsigned, iter, iter, CH(task_select_func, task_bit_count));
		CHAN_OUT1(uint32_t, seed, seed, CH(task_select_func, task_bit_count));
		func++;
		CHAN_OUT1(unsigned, func, func, SELF_CH(task_select_func));
		TRANSITION_TO_MT(task_bit_count);
	}
	else if(func == 1){
		CHAN_OUT1(unsigned, iter, iter, CH(task_select_func, task_bitcount));
		CHAN_OUT1(uint32_t, seed, seed, CH(task_select_func, task_bitcount));
		func++;
		CHAN_OUT1(unsigned, func, func, SELF_CH(task_select_func));
		TRANSITION_TO_MT(task_bitcount);
	}
	else if(func == 2){
		CHAN_OUT1(unsigned, iter, iter, CH(task_select_func, task_ntbl_bitcnt));
		CHAN_OUT1(uint32_t, seed, seed, CH(task_select_func, task_ntbl_bitcnt));
		func++;
		CHAN_OUT1(unsigned, func, func, SELF_CH(task_select_func));
		TRANSITION_TO_MT(task_ntbl_bitcnt);
	}
	else if(func == 3){
			CHAN_OUT1(unsigned, iter, iter, CH(task_select_func, task_ntbl_bitcount));
			CHAN_OUT1(uint32_t, seed, seed, CH(task_select_func, task_ntbl_bitcount));
			func++;
			CHAN_OUT1(unsigned, func, func, SELF_CH(task_select_func));
			TRANSITION_TO_MT(task_ntbl_bitcount);
	}
	else if(func == 4){
			CHAN_OUT1(unsigned, iter, iter, CH(task_select_func, task_BW_btbl_bitcount));
			CHAN_OUT1(uint32_t, seed, seed, CH(task_select_func, task_BW_btbl_bitcount));
			func++;
			CHAN_OUT1(unsigned, func, func, SELF_CH(task_select_func));
			TRANSITION_TO_MT(task_BW_btbl_bitcount);
	}
	else if(func == 5){
			CHAN_OUT1(unsigned, iter, iter, CH(task_select_func, task_AR_btbl_bitcount));
			CHAN_OUT1(uint32_t, seed, seed, CH(task_select_func, task_AR_btbl_bitcount));
			func++;
			CHAN_OUT1(unsigned, func, func, SELF_CH(task_select_func));
			TRANSITION_TO_MT(task_AR_btbl_bitcount);
	}
	else if(func == 6){
			CHAN_OUT1(unsigned, iter, iter, CH(task_select_func, task_bit_shifter));
			CHAN_OUT1(uint32_t, seed, seed, CH(task_select_func, task_bit_shifter));
			func++;
			CHAN_OUT1(unsigned, func, func, SELF_CH(task_select_func));
			TRANSITION_TO_MT(task_bit_shifter);
	}
	else{
		TRANSITION_TO_MT(task_end);

	}
#if 0
	switch(func){
		case 0:
			LOG("a\r\n");
			CHAN_OUT1(unsigned, iter, iter, CH(task_select_func, task_bit_count));
			CHAN_OUT1(uint32_t, seed, seed, CH(task_select_func, task_bit_count));
			func++;
			CHAN_OUT1(unsigned, func, func, SELF_CH(task_select_func));
			TRANSITION_TO_MT(task_bit_count);
			break;
		case 1:
			LOG("b\r\n");
			CHAN_OUT1(unsigned, iter, iter, CH(task_select_func, task_bitcount));
			CHAN_OUT1(uint32_t, seed, seed, CH(task_select_func, task_bitcount));
			func++;
			CHAN_OUT1(unsigned, func, func, SELF_CH(task_select_func));
			TRANSITION_TO_MT(task_bitcount);
			break;
		case 2:
			LOG("c\r\n");
			CHAN_OUT1(unsigned, iter, iter, CH(task_select_func, task_ntbl_bitcnt));
			CHAN_OUT1(uint32_t, seed, seed, CH(task_select_func, task_ntbl_bitcnt));
			func++;
			CHAN_OUT1(unsigned, func, func, SELF_CH(task_select_func));
			TRANSITION_TO_MT(task_ntbl_bitcnt);
			break;
		case 3:
			LOG("d\r\n");
			CHAN_OUT1(unsigned, iter, iter, CH(task_select_func, task_ntbl_bitcount));
			CHAN_OUT1(uint32_t, seed, seed, CH(task_select_func, task_ntbl_bitcount));
			func++;
			CHAN_OUT1(unsigned, func, func, SELF_CH(task_select_func));
			TRANSITION_TO_MT(task_ntbl_bitcount);
			break;
		case 4:
			CHAN_OUT1(unsigned, iter, iter, CH(task_select_func, task_BW_btbl_bitcount));
			CHAN_OUT1(unsigned, seed, seed, CH(task_select_func, task_BW_btbl_bitcount));
			func++;
			CHAN_OUT1(unsigned, func, func, SELF_CH(task_select_func));
			TRANSITION_TO_MT(task_BW_btbl_bitcount);
			break;
		case 5:
			CHAN_OUT1(unsigned, iter, iter, CH(task_select_func, task_AR_btbl_bitcount));
			CHAN_OUT1(unsigned, seed, seed, CH(task_select_func, task_AR_btbl_bitcount));
			func++;
			CHAN_OUT1(unsigned, func, func, SELF_CH(task_select_func));
			TRANSITION_TO_MT(task_AR_btbl_bitcount);
			break;
		case 6:
			CHAN_OUT1(unsigned, iter, iter, CH(task_select_func, task_bit_shifter));
			CHAN_OUT1(unsigned, seed, seed, CH(task_select_func, task_bit_shifter));
			func++;
			CHAN_OUT1(unsigned, func, func, SELF_CH(task_select_func));
			TRANSITION_TO_MT(task_bit_shifter);
			break;
		default: TRANSITION_TO_MT(task_end);
	}
#endif
}
void task_bit_count() {
	LOG("bit_count\r\n");
	unsigned n = *CHAN_IN2(unsigned, n, CH(task_init, task_bit_count), SELF_CH(task_bit_count));
	unsigned iter = *CHAN_IN2(unsigned, iter, CH(task_select_func, task_bit_count), SELF_CH(task_bit_count));
	uint32_t seed = *CHAN_IN2(uint32_t, seed, CH(task_select_func, task_bit_count), SELF_CH(task_bit_count));
	uint32_t next_seed = seed + 13;
	unsigned temp = 0;
	if(seed) do 
		temp++;
	while (0 != (seed = seed&(seed-1)));
	n += temp;
	CHAN_OUT2(unsigned, n, n, CH(task_bit_count, task_end), SELF_CH(task_bit_count));
	iter++;
	CHAN_OUT1(unsigned, iter, iter, SELF_CH(task_bit_count));
	CHAN_OUT1(uint32_t, seed, next_seed, SELF_CH(task_bit_count));
	
	if(iter < ITER){
		TRANSITION_TO_MT(task_bit_count);
	}
	else{
		TRANSITION_TO_MT(task_select_func);
	}
}
void task_bitcount() {
	LOG("bitcount\r\n");
	unsigned n = *CHAN_IN2(unsigned, n, CH(task_init, task_bitcount), SELF_CH(task_bitcount));
	unsigned iter = *CHAN_IN2(unsigned, iter, CH(task_select_func, task_bitcount), SELF_CH(task_bitcount));
	uint32_t seed = *CHAN_IN2(uint32_t, seed, CH(task_select_func, task_bitcount), SELF_CH(task_bitcount));
	uint32_t next_seed = seed + 13;
	unsigned temp = 0;
	seed = ((seed & 0xAAAAAAAAL) >>  1) + (seed & 0x55555555L);
	seed = ((seed & 0xCCCCCCCCL) >>  2) + (seed & 0x33333333L);
	seed = ((seed & 0xF0F0F0F0L) >>  4) + (seed & 0x0F0F0F0FL);
	seed = ((seed & 0xFF00FF00L) >>  8) + (seed & 0x00FF00FFL);
	seed = ((seed & 0xFFFF0000L) >> 16) + (seed & 0x0000FFFFL);
	n += (int)seed;
	CHAN_OUT2(unsigned, n, n, CH(task_bitcount, task_end), SELF_CH(task_bitcount));
	iter++;
	CHAN_OUT1(unsigned, iter, iter, SELF_CH(task_bitcount));
	CHAN_OUT1(uint32_t, seed, next_seed, SELF_CH(task_bitcount));

	if(iter < ITER){
		TRANSITION_TO_MT(task_bitcount);
	}
	else{
		TRANSITION_TO_MT(task_select_func);
	}
}
int recursive_cnt(uint32_t x){
	int cnt = bits_arr[(int)(x & 0x0000000FL)];

	if (0L != (x >>= 4))
		cnt += recursive_cnt(x);

	return cnt;
}
void task_ntbl_bitcnt() {
	LOG("ntbl_bitcnt\r\n");
	unsigned n = *CHAN_IN2(unsigned, n, CH(task_init, task_ntbl_bitcnt), SELF_CH(task_ntbl_bitcnt));
	unsigned iter = *CHAN_IN2(unsigned, iter, CH(task_select_func, task_ntbl_bitcnt), SELF_CH(task_ntbl_bitcnt));
	uint32_t seed = *CHAN_IN2(uint32_t, seed, CH(task_select_func, task_ntbl_bitcnt), SELF_CH(task_ntbl_bitcnt));
	uint32_t next_seed = seed + 13;
	unsigned temp = 0;
	n += recursive_cnt(seed);	
	CHAN_OUT2(unsigned, n, n, CH(task_ntbl_bitcnt, task_end), SELF_CH(task_ntbl_bitcnt));
	iter++;
	CHAN_OUT1(unsigned, iter, iter, SELF_CH(task_ntbl_bitcnt));
	CHAN_OUT1(uint32_t, seed, next_seed, SELF_CH(task_ntbl_bitcnt));

	if(iter < ITER){
		TRANSITION_TO_MT(task_ntbl_bitcnt);
	}
	else{
		TRANSITION_TO_MT(task_select_func);
	}
}
void task_ntbl_bitcount() {
	LOG("ntbl_bitcount\r\n");
	unsigned n = *CHAN_IN2(unsigned, n, CH(task_init, task_ntbl_bitcount), SELF_CH(task_ntbl_bitcount));
	unsigned iter = *CHAN_IN2(unsigned, iter, CH(task_select_func, task_ntbl_bitcount), SELF_CH(task_ntbl_bitcount));
	uint32_t seed = *CHAN_IN2(uint32_t, seed, CH(task_select_func, task_ntbl_bitcount), SELF_CH(task_ntbl_bitcount));
	uint32_t next_seed = seed + 13;
	unsigned temp = 0;
	n += bits_arr[ (int) (seed & 0x0000000FUL)       ] +
		bits_arr[ (int)((seed & 0x000000F0UL) >> 4) ] +
		bits_arr[ (int)((seed & 0x00000F00UL) >> 8) ] +
		bits_arr[ (int)((seed & 0x0000F000UL) >> 12)] +
		bits_arr[ (int)((seed & 0x000F0000UL) >> 16)] +
		bits_arr[ (int)((seed & 0x00F00000UL) >> 20)] +
		bits_arr[ (int)((seed & 0x0F000000UL) >> 24)] +
		bits_arr[ (int)((seed & 0xF0000000UL) >> 28)];
	CHAN_OUT2(unsigned, n, n, CH(task_ntbl_bitcount, task_end), SELF_CH(task_ntbl_bitcount));
	iter++;
	CHAN_OUT1(unsigned, iter, iter, SELF_CH(task_ntbl_bitcount));
	CHAN_OUT1(uint32_t, seed, next_seed, SELF_CH(task_ntbl_bitcount));

	if(iter < ITER){
		TRANSITION_TO_MT(task_ntbl_bitcount);
	}
	else{
		TRANSITION_TO_MT(task_select_func);
	}
}
void task_BW_btbl_bitcount() {
	LOG("BW_btbl_bitcount\r\n");
	unsigned n = *CHAN_IN2(unsigned, n, CH(task_init, task_BW_btbl_bitcount), SELF_CH(task_BW_btbl_bitcount));
	unsigned iter = *CHAN_IN2(unsigned, iter, CH(task_select_func, task_BW_btbl_bitcount), SELF_CH(task_BW_btbl_bitcount));
	uint32_t seed = *CHAN_IN2(uint32_t, seed, CH(task_select_func, task_BW_btbl_bitcount), SELF_CH(task_BW_btbl_bitcount));
	uint32_t next_seed = seed + 13;
	unsigned temp = 0;
	union 
	{ 
		unsigned char ch[4]; 
		long y; 
	} U; 

	U.y = seed; 

	n += bits_arr[ U.ch[0] ] + bits_arr[ U.ch[1] ] + 
		bits_arr[ U.ch[3] ] + bits_arr[ U.ch[2] ]; 
	CHAN_OUT2(unsigned, n, n, CH(task_BW_btbl_bitcount, task_end), SELF_CH(task_BW_btbl_bitcount));
	iter++;
	CHAN_OUT1(unsigned, iter, iter, SELF_CH(task_BW_btbl_bitcount));
	CHAN_OUT1(uint32_t, seed, next_seed, SELF_CH(task_BW_btbl_bitcount));

	if(iter < ITER){
		TRANSITION_TO_MT(task_BW_btbl_bitcount);
	}
	else{
		TRANSITION_TO_MT(task_select_func);
	}
}
void task_AR_btbl_bitcount() {
	LOG("AR_btbl_bitcount\r\n");
	unsigned n = *CHAN_IN2(unsigned, n, CH(task_init, task_AR_btbl_bitcount), SELF_CH(task_AR_btbl_bitcount));
	unsigned iter = *CHAN_IN2(unsigned, iter, CH(task_select_func, task_AR_btbl_bitcount), SELF_CH(task_AR_btbl_bitcount));
	uint32_t seed = *CHAN_IN2(uint32_t, seed, CH(task_select_func, task_AR_btbl_bitcount), SELF_CH(task_AR_btbl_bitcount));
	uint32_t next_seed = seed + 13;
	unsigned temp = 0;
	unsigned char * Ptr = (unsigned char *) &seed ;
	int Accu ;

	Accu  = bits_arr[ *Ptr++ ];
	Accu += bits_arr[ *Ptr++ ];
	Accu += bits_arr[ *Ptr++ ];
	Accu += bits_arr[ *Ptr ];
	n+= Accu;
	CHAN_OUT2(unsigned, n, n, CH(task_AR_btbl_bitcount, task_end), SELF_CH(task_AR_btbl_bitcount));
	iter++;
	CHAN_OUT1(unsigned, iter, iter, SELF_CH(task_AR_btbl_bitcount));
	CHAN_OUT1(uint32_t, seed, next_seed, SELF_CH(task_AR_btbl_bitcount));

	if(iter < ITER){
		TRANSITION_TO_MT(task_AR_btbl_bitcount);
	}
	else{
		TRANSITION_TO_MT(task_select_func);
	}
}
void task_bit_shifter() {
	LOG("bit_shifter\r\n");
	unsigned n = *CHAN_IN2(unsigned, n, CH(task_init, task_bit_shifter), SELF_CH(task_bit_shifter));
	unsigned iter = *CHAN_IN2(unsigned, iter, CH(task_select_func, task_bit_shifter), SELF_CH(task_bit_shifter));
	uint32_t seed = *CHAN_IN2(uint32_t, seed, CH(task_select_func, task_bit_shifter), SELF_CH(task_bit_shifter));
	uint32_t next_seed = seed + 13;
	unsigned temp = 0;
	
	int i, nn;

	for (i = nn = 0; seed && (i < (sizeof(long) * CHAR_BIT)); ++i, seed >>= 1)
		nn += (int)(seed & 1L);
	n += nn;

	CHAN_OUT2(unsigned, n, n, CH(task_bit_shifter, task_end), SELF_CH(task_bit_shifter));
	iter++;
	CHAN_OUT1(unsigned, iter, iter, SELF_CH(task_bit_shifter));
	CHAN_OUT1(uint32_t, seed, next_seed, SELF_CH(task_bit_shifter));

	if(iter < ITER){
		TRANSITION_TO_MT(task_bit_shifter);
	}
	else{
		TRANSITION_TO_MT(task_select_func);
	}
}

void task_end() {
	LOG("end\r\n");
	unsigned n_0 = *CHAN_IN1(unsigned, n, CH(task_bit_count, task_end));
	unsigned n_1 = *CHAN_IN1(unsigned, n, CH(task_bitcount, task_end));
	unsigned n_2 = *CHAN_IN1(unsigned, n, CH(task_ntbl_bitcnt, task_end));
	unsigned n_3 = *CHAN_IN1(unsigned, n, CH(task_ntbl_bitcount, task_end));
	unsigned n_4 = *CHAN_IN1(unsigned, n, CH(task_BW_btbl_bitcount, task_end));
	unsigned n_5 = *CHAN_IN1(unsigned, n, CH(task_AR_btbl_bitcount, task_end));
	unsigned n_6 = *CHAN_IN1(unsigned, n, CH(task_bit_shifter, task_end));
	PRINTF("%u\r\n", n_0);
	PRINTF("%u\r\n", n_1);
	PRINTF("%u\r\n", n_2);
	PRINTF("%u\r\n", n_3);
	PRINTF("%u\r\n", n_4);
	PRINTF("%u\r\n", n_5);
	PRINTF("%u\r\n", n_6);
	THREAD_END(); 
	TRANSITION_TO_MT(task_end);
}

/*-----------------------cuckoo filter tasks start--------------------------------------*/ 
void task_generate_key()
{
    task_prologue();

    value_t key = *CHAN_IN4(value_t, key, CH(task_init, task_generate_key),
                                          CH(task_insert_done, task_generate_key),
                                          CH(task_lookup_done, task_generate_key),
                                          SELF_IN_CH(task_generate_key));

    // insert pseufo-random integers, for testing
    // If we use consecutive ints, they hash to consecutive DJB hashes...
    // NOTE: we are not using rand(), to have the sequence available to verify
    // that that are no false negatives (and avoid having to save the values).
    key = (key + 1) * 17;

    LOG("generate_key: key: %x\r\n", key);

    CHAN_OUT2(value_t, key, key, MC_OUT_CH(ch_key, task_generate_key,
                                           task_fingerprint, task_lookup),
                                 SELF_OUT_CH(task_generate_key));

    task_t *next_task = *CHAN_IN2(task_t *, next_task,
                                  CH(task_init, task_generate_key),
                                  CH(task_insert_done, task_generate_key));
    transition_to_mt(next_task);
}

void task_calc_indexes()
{
    task_prologue();

    value_t key = *CHAN_IN1(value_t, key, CALL_CH(ch_calc_indexes));

    fingerprint_t fp = hash_to_fingerprint(key);
    LOG("calc indexes: fingerprint: key %04x fp %04x\r\n", key, fp);

    CHAN_OUT2(fingerprint_t, fingerprint, fp,
              CH(task_calc_indexes, task_calc_indexes_index_2),
              RET_CH(ch_calc_indexes));

    TRANSITION_TO_MT(task_calc_indexes_index_1);
}

void task_calc_indexes_index_1()
{
    task_prologue();
    LOG("CALC_INDEXES_cuckoo\r\n"); 

    value_t key = *CHAN_IN1(value_t, key, CALL_CH(ch_calc_indexes));

    index_t index1 = hash_to_index(key);
    LOG("calc indexes: index1: key %04x idx1 %u\r\n", key, index1);

    CHAN_OUT2(index_t, index1, index1,
              CH(task_calc_indexes_index_1, task_calc_indexes_index_2),
              RET_CH(ch_calc_indexes));

    TRANSITION_TO_MT(task_calc_indexes_index_2);
}

void task_calc_indexes_index_2()
{
    task_prologue();
    LOG("CALC_INDEXES_2_cuckoo\r\n"); 
    fingerprint_t fp = *CHAN_IN1(fingerprint_t, fingerprint,
                                 CH(task_calc_indexes, task_calc_indexes_index_2));
    index_t index1 = *CHAN_IN1(index_t, index1,
                               CH(task_calc_indexes_index_1, task_calc_indexes_index_2));

    index_t fp_hash = hash_to_index(fp);
    index_t index2 = index1 ^ fp_hash;

    LOG("calc indexes: index2: fp hash: %04x idx1 %u idx2 %u\r\n",
        fp_hash, index1, index2);

    CHAN_OUT1(index_t, index2, index2, RET_CH(ch_calc_indexes));

    task_t *next_task = *CHAN_IN1(task_t *, next_task,
                                  CALL_CH(ch_calc_indexes));
    transition_to_mt(next_task);
}

// This task is a somewhat redundant proxy. But it will be a callable
// task and also be responsible for making the call to calc_index.
void task_insert()
{
    task_prologue();
    LOG("TASK_INSERT_cuckoo\r\n"); 
    value_t key = *CHAN_IN1(value_t, key,
                            MC_IN_CH(ch_key, task_generate_key, task_insert));

    LOG("insert: key %04x\r\n", key);

    CHAN_OUT1(value_t, key, key, CALL_CH(ch_calc_indexes));

    task_t *next_task = TASK_REF(task_add);
    CHAN_OUT1(task_t *, next_task, next_task, CALL_CH(ch_calc_indexes));
    TRANSITION_TO_MT(task_calc_indexes);
}


void task_add()
{
    task_prologue();
    LOG("TASK_ADD_cuckoo\r\n");

    bool success = true;

    // Fingerprint being inserted
    fingerprint_t fp = *CHAN_IN1(fingerprint_t, fingerprint,
                                 RET_CH(ch_calc_indexes));
    LOG("add: fp %04x\r\n", fp);

    // index1,fp1 and index2,fp2 are the two alternative buckets

    index_t index1 = *CHAN_IN1(index_t, index1, RET_CH(ch_calc_indexes));

    fingerprint_t fp1 = *CHAN_IN3(fingerprint_t, filter[index1],
                                 MC_IN_CH(ch_filter, task_init, task_add),
                                 CH(task_relocate, task_add),
                                 SELF_IN_CH(task_add));
    LOG("add: idx1 %u fp1 %04x\r\n", index1, fp1);

    if (!fp1) {
        LOG("add: filled empty slot at idx1 %u\r\n", index1);

        CHAN_OUT2(fingerprint_t, filter[index1], fp,
                  MC_OUT_CH(ch_filter_add, task_add,
                            task_relocate, task_insert_done,
                            task_lookup_search, task_print_stats),
                  SELF_OUT_CH(task_add));

        CHAN_OUT1(bool, success, success, CH(task_add, task_insert_done));
        TRANSITION_TO_MT(task_insert_done);
    } else {
        index_t index2 = *CHAN_IN1(index_t, index2, RET_CH(ch_calc_indexes));
        fingerprint_t fp2 = *CHAN_IN3(fingerprint_t, filter[index2],
                                     MC_IN_CH(ch_filter, task_init, task_add),
                                     CH(task_relocate, task_add),
                                     SELF_IN_CH(task_add));
        LOG("add: fp2 %04x\r\n", fp2);

        if (!fp2) {
            LOG("add: filled empty slot at idx2 %u\r\n", index2);

            CHAN_OUT2(fingerprint_t, filter[index2], fp,
                      MC_OUT_CH(ch_filter_add, task_add,
                                task_relocate, task_insert_done, task_lookup_search),
                      SELF_OUT_CH(task_add));

            CHAN_OUT1(bool, success, success, CH(task_add, task_insert_done));
            TRANSITION_TO_MT(task_insert_done);
        } else { // evict one of the two entries
            fingerprint_t fp_victim;
            index_t index_victim;

            if (rand() % 2) {
                index_victim = index1;
                fp_victim = fp1;
            } else {
                index_victim = index2;
                fp_victim = fp2;
            }

            LOG("add: evict [%u] = %04x\r\n", index_victim, fp_victim);

            // Evict the victim
            CHAN_OUT2(fingerprint_t, filter[index_victim], fp,
                      MC_OUT_CH(ch_filter_add, task_add,
                                task_relocate, task_insert_done, task_lookup_search),
                      SELF_OUT_CH(task_add));

            CHAN_OUT1(index_t, index_victim, index_victim, CH(task_add, task_relocate));
            CHAN_OUT1(fingerprint_t, fp_victim, fp_victim, CH(task_add, task_relocate));
            unsigned relocation_count = 0;
            CHAN_OUT1(unsigned, relocation_count, relocation_count,
                      CH(task_add, task_relocate));

            TRANSITION_TO_MT(task_relocate);
        }
    }
}

void task_relocate()
{
    task_prologue();
    LOG("TASK_RELOCATE_cuckoo\r\n");

    fingerprint_t fp_victim = *CHAN_IN2(fingerprint_t, fp_victim,
                                        CH(task_add, task_relocate),
                                        SELF_IN_CH(task_relocate));

    index_t index1_victim = *CHAN_IN2(index_t, index_victim,
                                      CH(task_add, task_relocate),
                                      SELF_IN_CH(task_relocate));

    index_t fp_hash_victim = hash_to_index(fp_victim);
    index_t index2_victim = index1_victim ^ fp_hash_victim;

    LOG("relocate: victim fp hash %04x idx1 %u idx2 %u\r\n",
        fp_hash_victim, index1_victim, index2_victim);

    fingerprint_t fp_next_victim =
        *CHAN_IN3(fingerprint_t, filter[index2_victim],
                  MC_IN_CH(ch_filter, task_init, task_relocate),
                  MC_IN_CH(ch_filter_add, task_add, task_relocate),
                  SELF_IN_CH(task_relocate));

    LOG("relocate: next victim fp %04x\r\n", fp_next_victim);

    // Take victim's place
    CHAN_OUT2(fingerprint_t, filter[index2_victim], fp_victim,
             MC_OUT_CH(ch_filter_relocate, task_relocate,
                       task_add, task_insert_done, task_lookup_search,
                       task_print_stats),
             SELF_OUT_CH(task_relocate));

    if (!fp_next_victim) { // slot was free
        bool success = true;
        CHAN_OUT1(bool, success, success, CH(task_relocate, task_insert_done));
        TRANSITION_TO_MT(task_insert_done);
    } else { // slot was occupied, rellocate the next victim

        unsigned relocation_count = *CHAN_IN2(unsigned, relocation_count,
                                              CH(task_add, task_relocate),
                                              SELF_IN_CH(task_relocate));

        LOG("relocate: relocs %u\r\n", relocation_count);

        if (relocation_count >= MAX_RELOCATIONS) { // insert failed
            LOG("relocate: max relocs reached: %u\r\n", relocation_count);
            PRINTF("insert: lost fp %04x\r\n", fp_next_victim);
            bool success = false;
            CHAN_OUT1(bool, success, success, CH(task_relocate, task_insert_done));
            TRANSITION_TO_MT(task_insert_done);
        }

        relocation_count++;
        CHAN_OUT1(unsigned, relocation_count, relocation_count,
                 SELF_OUT_CH(task_relocate));

        CHAN_OUT1(index_t, index_victim, index2_victim, SELF_OUT_CH(task_relocate));
        CHAN_OUT1(fingerprint_t, fp_victim, fp_next_victim, SELF_OUT_CH(task_relocate));

        TRANSITION_TO_MT(task_relocate);
    }
}

void task_insert_done()
{
    task_prologue();
    LOG("TASK_INSERT_DONE_cuckoo\r\n"); 

//#if VERBOSE > 0
    unsigned i;

    LOG("insert done: filter:\r\n");
    for (i = 0; i < NUM_BUCKETS; ++i) {
        fingerprint_t fp = *CHAN_IN3(fingerprint_t, filter[i],
                 MC_IN_CH(ch_filter, task_init, task_insert_done),
                 MC_IN_CH(ch_filter_add, task_add, task_insert_done),
                 MC_IN_CH(ch_filter_relocate, task_relocate, task_insert_done));

        LOG("%04x ", fp);
        if (i > 0 && (i + 1) % 8 == 0)
            LOG("\r\n");
    }
    LOG("\r\n");
//#endif

    unsigned insert_count = *CHAN_IN2(unsigned, insert_count,
                                      CH(task_init, task_insert_done),
                                      SELF_IN_CH(task_insert_done));
    insert_count++;
    CHAN_OUT1(unsigned, insert_count, insert_count, SELF_OUT_CH(task_insert_done));

    bool success = *CHAN_IN2(bool, success,
                             CH(task_add, task_insert_done),
                             CH(task_relocate, task_insert_done));

    unsigned inserted_count = *CHAN_IN2(unsigned, inserted_count,
                                        CH(task_init, task_insert_done),
                                        SELF_IN_CH(task_insert_done));
    inserted_count += success;
    CHAN_OUT1(unsigned, inserted_count, inserted_count, SELF_OUT_CH(task_insert_done));

    LOG("insert done: insert %u inserted %u\r\n", insert_count, inserted_count);

#ifdef CONT_POWER
    volatile uint32_t delay = 0x8ffff;
    while (delay--);
#endif

    if (insert_count < NUM_INSERTS) {
        task_t *next_task = TASK_REF(task_insert);
        CHAN_OUT1(task_t *, next_task, next_task, CH(task_insert_done, task_generate_key));
        TRANSITION_TO_MT(task_generate_key);
    } else {
        CHAN_OUT1(unsigned, inserted_count, inserted_count,
                  CH(task_insert_done, task_print_stats));

        task_t *next_task = TASK_REF(task_lookup);
        CHAN_OUT1(value_t, key, init_key, CH(task_insert_done, task_generate_key));
        CHAN_OUT1(task_t *, next_task, next_task, CH(task_insert_done, task_generate_key));
        TRANSITION_TO_MT(task_generate_key);
    }
}

void task_lookup()
{
    task_prologue();
    LOG("TASK_LOOKUP_cuckoo\r\n"); 

    value_t key = *CHAN_IN1(value_t, key,
                            MC_IN_CH(ch_key, task_generate_key,task_lookup));
    LOG("lookup: key %04x\r\n", key);

    CHAN_OUT2(value_t, key, key, CALL_CH(ch_calc_indexes),
                                 CH(task_lookup, task_lookup_done));
    
    task_t *next_task = TASK_REF(task_lookup_search);
    CHAN_OUT1(task_t *, next_task, next_task, CALL_CH(ch_calc_indexes));
    TRANSITION_TO_MT(task_calc_indexes);
}

void task_lookup_search()
{
    task_prologue();
    LOG("TASK_LOOKUP_SEARCH_cuckoo\r\n"); 

    fingerprint_t fp1, fp2;
    bool member = false;

    index_t index1 = *CHAN_IN1(index_t, index1, RET_CH(ch_calc_indexes));
    index_t index2 = *CHAN_IN1(index_t, index2, RET_CH(ch_calc_indexes));
    fingerprint_t fp = *CHAN_IN1(fingerprint_t, fingerprint, RET_CH(ch_calc_indexes));

    LOG("lookup search: fp %04x idx1 %u idx2 %u\r\n", fp, index1, index2);

    fp1 = *CHAN_IN3(fingerprint_t, filter[index1],
                    MC_IN_CH(ch_filter, task_init, task_lookup_search),
                    MC_IN_CH(ch_filter_add, task_add, task_lookup_search),
                    MC_IN_CH(ch_filter_relocate, task_relocate, task_lookup_search));
    LOG("lookup search: fp1 %04x\r\n", fp1);

    if (fp1 == fp) {
        member = true;
    } else {
        fp2 = *CHAN_IN3(fingerprint_t, filter[index2],
                MC_IN_CH(ch_filter, task_init, task_lookup_search),
                MC_IN_CH(ch_filter_add, task_add, task_lookup_search),
                MC_IN_CH(ch_filter_relocate, task_relocate, task_lookup_search));
        LOG("lookup search: fp2 %04x\r\n", fp2);

        if (fp2 == fp) {
            member = true;
        }
    }

    LOG("lookup search: fp %04x member %u\r\n", fp, member);
    CHAN_OUT1(bool, member, member, CH(task_lookup_search, task_lookup_done));

    if (!member) {
        PRINTF("lookup: key %04x not member\r\n", fp);
    }

    TRANSITION_TO_MT(task_lookup_done);
}

void task_lookup_done()
{
    task_prologue();
    LOG("TASK_LOOKUP_DONE_cuckoo\r\n"); 

    bool member = *CHAN_IN1(bool, member, CH(task_lookup_search, task_lookup_done));

    unsigned lookup_count = *CHAN_IN2(unsigned, lookup_count,
                                      CH(task_init, task_lookup_done),
                                      SELF_IN_CH(task_lookup_done));


    lookup_count++;
    CHAN_OUT1(unsigned, lookup_count, lookup_count, SELF_OUT_CH(task_lookup_done));

//#if VERBOSE > 1
    value_t key = *CHAN_IN1(value_t, key, CH(task_lookup, task_lookup_done));
    LOG("lookup done [%u]: key %04x member %u\r\n", lookup_count, key, member);
//#endif

    unsigned member_count = *CHAN_IN2(bool, member_count,
                                      CH(task_init, task_lookup_done),
                                      SELF_IN_CH(task_lookup_done));


    member_count += member;
    CHAN_OUT1(unsigned, member_count, member_count, SELF_OUT_CH(task_lookup_done));

    LOG("lookup done: lookups %u members %u\r\n", lookup_count, member_count);

#ifdef CONT_POWER
    volatile uint32_t delay = 0x8ffff;
    while (delay--);
#endif

    if (lookup_count < NUM_LOOKUPS) {
        task_t *next_task = TASK_REF(task_lookup);
        CHAN_OUT1(task_t *, next_task, next_task, CH(task_lookup_done, task_generate_key));
        TRANSITION_TO_MT(task_generate_key);
    } else {
        CHAN_OUT1(unsigned, member_count, member_count,
                  CH(task_lookup_done, task_print_stats));
        TRANSITION_TO_MT(task_print_stats);
    }
}

void task_print_stats()
{
    task_prologue();
    LOG("TASK_PRINT_STATS_cuckoo\r\n"); 

    unsigned i;

    unsigned inserted_count = *CHAN_IN1(unsigned, inserted_count,
                                     CH(task_insert_done, task_print_stats));
    unsigned member_count = *CHAN_IN1(unsigned, member_count,
                                     CH(task_lookup_done, task_print_stats));

    PRINTF("stats: inserts %u members %u total %u\r\n",
           inserted_count, member_count, NUM_INSERTS);

    BLOCK_PRINTF_BEGIN();
    BLOCK_PRINTF("filter:\r\n");
    for (i = 0; i < NUM_BUCKETS; ++i) {
        fingerprint_t fp = *CHAN_IN3(fingerprint_t, filter[i],
                 MC_IN_CH(ch_filter, task_init, task_print_stats),
                 MC_IN_CH(ch_filter_add, task_add, task_print_stats),
                 MC_IN_CH(ch_filter_relocate, task_relocate, task_print_stats));

        BLOCK_PRINTF("%04x ", fp);
        if (i > 0 && (i + 1) % 8 == 0)
            BLOCK_PRINTF("\r\n");
    }
    BLOCK_PRINTF_END();

    TRANSITION_TO_MT(task_done);
}

void task_done()
{
    task_prologue();
/*
#if defined(BOARD_WISP) || defined(BOARD_MSP_TS430)
    GPIO(PORT_AUX, OUT) |= BIT(PIN_AUX_1); 
    GPIO(PORT_LED_1, OUT) |= BIT(PIN_LED_1); 
#elif defined(BOARD_CAPYBARA)
    GPIO(PORT_DEBUG, OUT) |= BIT(PIN_DEBUG_1); 
#endif
*/    
    THREAD_END(); 
    TRANSITION_TO_MT(task_done);
}
/*--------------------------blinker functions------------------------------------*/


void task_1_r()
{
    task_prologue();

    unsigned blinks;
    unsigned duty_cycle;

    LOG("task 1_r\r\n");

    // Solid flash signifying beginning of task
    /*False
    GPIO(PORT_LED_1, OUT) |= BIT(PIN_LED_1);
    burn(TASK_START_DURATION_ITERS);
    GPIO(PORT_LED_1, OUT) &= ~BIT(PIN_LED_1);
    burn(TASK_START_DURATION_ITERS);
    */

    blinks = *CHAN_IN2(unsigned, blinks, CH_TH(task_init, task_1_r, 0), CH_TH(task_2_r, task_1_r, 0));
    duty_cycle = *CHAN_IN1(unsigned, duty_cycle,
                           MC_IN_CH(ch_duty_cycle_r, task_init, task_1_r));

    LOG("task 1: blinks %u dc %u\r\n", blinks, duty_cycle);

    //blink_led1(blinks, duty_cycle);
    /*
    if(blinks > 8)
      blinks = 0;
    */
    blinks++;

    CHAN_OUT1(unsigned, blinks, blinks, CH_TH(task_1_r, task_2_r, 0));

    //THREAD_CREATE(task_2_r);
    TRANSITION_TO_MT(task_2_r);
    //TRANSITION_TO(task_2);
}

void task_2_r()
{
    task_prologue();

    unsigned blinks;
    unsigned duty_cycle;

    LOG("task 2_r\r\n");

    // Solid flash signifying beginning of task
    /*
    GPIO(PORT_LED_2, OUT) |= BIT(PIN_LED_2);
    burn(TASK_START_DURATION_ITERS);
    GPIO(PORT_LED_2, OUT) &= ~BIT(PIN_LED_2);
    burn(TASK_START_DURATION_ITERS);
    */
    blinks = *CHAN_IN1(unsigned, blinks, CH_TH(task_1_r, task_2_r, 0));
    duty_cycle = *CHAN_IN1(unsigned, duty_cycle,
                           MC_IN_CH(ch_duty_cycle_r, task_init, task_2_r));

    LOG("task 2: blinks %u dc %u\r\n", blinks, duty_cycle);

    //blink_led2(blinks, duty_cycle);
    
    if(blinks > 200)
    { PRINTF("Ending red thread!\r\n"); 
      THREAD_END(); 
    }
    
    blinks++;

    CHAN_OUT1(unsigned, blinks, blinks, CH_TH(task_2_r, task_1_r, 0));

	TRANSITION_TO_MT(task_3_r);

	//TRANSITION_TO(task_3);
}

void task_3_r()
{
    task_prologue();
    LOG("task 3 r prologue\r\n");

    unsigned wait_tick = *CHAN_IN2(unsigned, tick, CH_TH(task_init, task_3_r, 0),
                                                   SELF_IN_CH(task_3_r));

    LOG("task 3: wait tick %u\r\n", wait_tick);
    /*
    GPIO(PORT_LED_1, OUT) |= BIT(PIN_LED_1);
    GPIO(PORT_LED_2, OUT) |= BIT(PIN_LED_2);
    burn(WAIT_TICK_DURATION_ITERS);
    GPIO(PORT_LED_1, OUT) &= ~BIT(PIN_LED_1);
    GPIO(PORT_LED_2, OUT) &= ~BIT(PIN_LED_2);
    burn(WAIT_TICK_DURATION_ITERS);
    */
    if (++wait_tick < WAIT_TICKS) {
        CHAN_OUT1(unsigned, tick, wait_tick, SELF_OUT_CH(task_3_r));

	  	TRANSITION_TO_MT(task_3_r);
        //TRANSITION_TO(task_3);
    } else {
        unsigned tick = 0;
        CHAN_OUT1(unsigned, tick, tick, SELF_OUT_CH(task_3_r));

		TRANSITION_TO_MT(task_1_r);
        //TRANSITION_TO(task_1);

    }
}

void task_1_g()
{
    task_prologue();

    unsigned blinks;
    unsigned duty_cycle;

    LOG("task 1_g\r\n");

    // Solid flash signifying beginning of task
    /*
    GPIO(PORT_LED_1, OUT) |= BIT(PIN_LED_1);
    burn(TASK_START_DURATION_ITERS);
    GPIO(PORT_LED_1, OUT) &= ~BIT(PIN_LED_1);
    burn(TASK_START_DURATION_ITERS);
    */

    blinks = *CHAN_IN2(unsigned, blinks, CH_TH(task_init, task_1_g, 0), CH_TH(task_2_g, task_1_g, 0));
    duty_cycle = *CHAN_IN1(unsigned, duty_cycle,
                           MC_IN_CH(ch_duty_cycle_r, task_init, task_1_g));

    LOG("task 1: blinks %u dc %u\r\n", blinks, duty_cycle);

    //blink_led1(blinks, duty_cycle);
    /*
    if(blinks > 8)
      blinks = 0;
    */
    blinks++;

    CHAN_OUT1(unsigned, blinks, blinks, CH_TH(task_1_g, task_2_g, 0));

    THREAD_CREATE(task_2_g);
    TRANSITION_TO_MT(task_2_g);
    //TRANSITION_TO(task_2);
}

void task_2_g()
{
    task_prologue();

    unsigned blinks;
    unsigned duty_cycle;

    LOG("task 2_g\r\n");

    // Solid flash signifying beginning of task
    /*
    GPIO(PORT_LED_2, OUT) |= BIT(PIN_LED_2);
    burn(TASK_START_DURATION_ITERS);
    GPIO(PORT_LED_2, OUT) &= ~BIT(PIN_LED_2);
    burn(TASK_START_DURATION_ITERS);
    */
    blinks = *CHAN_IN1(unsigned, blinks, CH_TH(task_1_g, task_2_g, 0));
    duty_cycle = *CHAN_IN1(unsigned, duty_cycle,
                           MC_IN_CH(ch_duty_cycle_r, task_init, task_2_g));

    LOG("task 2_g: blinks %u dc %u\r\n", blinks, duty_cycle);

    //blink_led2(blinks, duty_cycle);
    if(blinks > 100)
    { PRINTF("Green thread end!\r\n"); 
      THREAD_END(); 
    }
    blinks++;

    CHAN_OUT1(unsigned, blinks, blinks, CH_TH(task_2_g, task_1_g, 0));

	TRANSITION_TO_MT(task_3_g);

	//TRANSITION_TO(task_3);
}

void task_3_g()
{
    task_prologue();
    LOG("task 3_g prologue\r\n");

    unsigned wait_tick = *CHAN_IN2(unsigned, tick, CH_TH(task_init, task_3_g, 0),
                                                   SELF_IN_CH(task_3_g));

    LOG("task 3: wait tick %u\r\n", wait_tick);
    /*
    GPIO(PORT_LED_1, OUT) |= BIT(PIN_LED_1);
    GPIO(PORT_LED_2, OUT) |= BIT(PIN_LED_2);
    burn(WAIT_TICK_DURATION_ITERS);
    GPIO(PORT_LED_1, OUT) &= ~BIT(PIN_LED_1);
    GPIO(PORT_LED_2, OUT) &= ~BIT(PIN_LED_2);
    burn(WAIT_TICK_DURATION_ITERS);
    */
    if (++wait_tick < WAIT_TICKS) {
        CHAN_OUT1(unsigned, tick, wait_tick, SELF_OUT_CH(task_3_g));

	  	TRANSITION_TO_MT(task_3_g);
        //TRANSITION_TO(task_3);
    } else {
        unsigned tick = 0;
        CHAN_OUT1(unsigned, tick, tick, SELF_OUT_CH(task_3_g));

		TRANSITION_TO_MT(task_1_g);
        //TRANSITION_TO(task_1);

    }
}

/*---------------------------shared end code-------------------------------------*/
ENTRY_TASK(task_init)
INIT_FUNC(init)
