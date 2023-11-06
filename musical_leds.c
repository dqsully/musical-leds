#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/sem.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "ws2812.pio.h"

#define LED_PIN_START 16
#define LED_STRIPS 5
#define MAX_STRIP_LENGTH 300

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return
            ((uint32_t) (r) << 16) |
            ((uint32_t) (g) << 8) |
            (uint32_t) (b);
}

typedef struct {
    // stored most-significant bit first
    uint32_t bit_planes[24];
} pixel_plane_t;

static pixel_plane_t framebuffers[2][MAX_STRIP_LENGTH];

static uint32_t pixels[LED_STRIPS][MAX_STRIP_LENGTH];

void translate_to_framebuffer(uint32_t pixels[LED_STRIPS][MAX_STRIP_LENGTH], pixel_plane_t framebuffer[MAX_STRIP_LENGTH]) {
    memset(framebuffer, 0, sizeof(pixel_plane_t) * MAX_STRIP_LENGTH);

    for (int pixel = 0; pixel < MAX_STRIP_LENGTH; pixel++) {
        for (int bit = 0; bit < 24; bit++) {
            for (int n_strip = 0; n_strip < LED_STRIPS; n_strip++) {
                if (pixels[n_strip][pixel] & (0x800000 >> bit)) framebuffer[pixel].bit_planes[bit] |= 1 << n_strip;
            }
        }
    }
}

#define MAX_ZONES_PER_STRIP 24

typedef struct {
    uint32_t effect;
    uint16_t off;
    uint16_t on;
} led_zone_t;

static led_zone_t zones[LED_STRIPS][MAX_ZONES_PER_STRIP];
static mutex_t zone_lock;
static led_zone_t zones_copy[LED_STRIPS][MAX_ZONES_PER_STRIP];

uint32_t render_pixel(int t, uint32_t offset, uint32_t *effect) {
    if ((*effect & 0xff000000) == 0) {
        return *effect;
    } else {
        return urgb_u32(0xff, 0, 0);
    }
}

void render_zones(int t) {
    memset(pixels, 0x00, sizeof(pixels));

    mutex_enter_blocking(&zone_lock);
    memcpy(zones_copy, zones, sizeof(zones));
    mutex_exit(&zone_lock);

    for (int strip = 0; strip < LED_STRIPS; strip++) {
        uint32_t pixel = 0;

        for (int zone_n = 0; zone_n < MAX_ZONES_PER_STRIP; zone_n++) {
            led_zone_t zone = zones_copy[strip][zone_n];
            pixel += zone.off;

            for (uint32_t offset = 0; offset < zone.on && pixel + offset < MAX_STRIP_LENGTH; offset++) {
                pixels[strip][pixel + offset] = render_pixel(t, offset, &zone.effect);
            }

            pixel += zone.on;
        }
    }
}

#define DMA_CHANNEL 0
#define DMA_CHANNEL_MASK (1u << DMA_CHANNEL)
#define DMA_CHANNELS_MASK (DMA_CHANNEL_MASK)

static struct semaphore reset_delay_complete_sem;
alarm_id_t reset_delay_alarm_id;

int64_t reset_delay_complete(alarm_id_t id, void *user_data) {
    reset_delay_alarm_id = 0;
    sem_release(&reset_delay_complete_sem);
    // no repeat
    return 0;
}

void __isr dma_complete_handler() {
    if (dma_hw->ints0 & DMA_CHANNEL_MASK) {
        // clear IRQ
        dma_hw->ints0 = DMA_CHANNEL_MASK;
        // when the dma is complete we start the reset delay timer
        if (reset_delay_alarm_id) cancel_alarm(reset_delay_alarm_id);
        reset_delay_alarm_id = add_alarm_in_us(400, reset_delay_complete, NULL, true);
    }
}

void dma_init(PIO pio, uint sm) {
    dma_claim_mask(DMA_CHANNELS_MASK);

    dma_channel_config channel_config = dma_channel_get_default_config(DMA_CHANNEL);
    channel_config_set_dreq(&channel_config, pio_get_dreq(pio, sm, true));
    // channel_config_set_irq_quiet(&channel_config, true);
    dma_channel_configure(
        DMA_CHANNEL,
        &channel_config,
        &pio->txf[sm],
        NULL, // set later
        MAX_STRIP_LENGTH * 24,
        false
    );

    irq_set_exclusive_handler(DMA_IRQ_0, dma_complete_handler);
    dma_channel_set_irq0_enabled(DMA_CHANNEL, true);
    irq_set_enabled(DMA_IRQ_0, true);
}

void output_strips_dma(pixel_plane_t framebuffer[MAX_STRIP_LENGTH]) {
    dma_channel_set_read_addr(DMA_CHANNEL, framebuffer, true);
}

uint32_t hex_to_uint(char *s, int len) {
    uint32_t out = 0;

    for (int i = 0; i < len; i++) {
        char c = s[i];

        if (c >= '0' && c <= '9') {
            c -= '0';
        } else if (c >= 'a' && c <= 'f') {
            c = c - 'a' + 10;
        } else if (c >= 'A' && c <= 'F') {
            c = c - 'A' + 10;
        }

        out |= (uint32_t) (c) << (4 * (len - i - 1));
    }

    return out;
}

uint8_t char_to_byte(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    } else if (c >= 'a' && c <= 'z') {
        return c - 'a' + 10;
    } else if (c >= 'A' && c <= 'z') {
        return c - 'A' + 10;
    } else {
        return 0;
    }
}

int getchar2() {
    int c = getchar();
    putchar(c);
    if (c == '\r') {
        putchar('\n');
    }
    return c;
}

int get_line(char *buf, int max) {
    char c;
    int i;

    for (i = 0; i < max; i++) {
        c = getchar2();

        if (c == '\n' || c == '\r') {
            break;
        }

        buf[i] = c;
    }

    return i;
}

void get_chars_silent(char *buf, int total) {
    for (int i = 0; i < total; i++) {
        buf[i] = getchar();
    }
}

void core1_entry() {
    char command[8];
    int i;

    while (1) {
        uint8_t strip;
        uint8_t zone;
        uint32_t value;

        command[0] = getchar();
        int value_size;
        bool text = false;
        int field;

        switch (command[0]) {
        case '\n':
        case '\r':
            putchar('\n');
            continue;

        case 'r':
            putchar(command[0]);
            putchar('\n');
        case 'R':
            mutex_enter_blocking(&zone_lock);
            memset(zones, 0, sizeof(zones));
            mutex_exit(&zone_lock);
            continue;

        case 'e':
            text = true;
        case 'E':
            value_size = 4;
            field = 2;
            break;

        case 'l':
            text = true;
        case 'L':
            value_size = 2;
            field = 1;
            break;

        case 'd':
            text = true;
        case 'D':
            value_size = 2;
            field = 0;
            break;

        default:
            puts("\nInvalid command\n");
            continue;
        }

        if (text) {
            putchar(command[0]);

            i = get_line(command, 3);

            if (i < 3) {
                puts("Command too short\n");
                continue;
            }

            strip = char_to_byte(command[0]);
            zone = hex_to_uint(command + 1, 2);

            i = get_line(command, value_size * 2);

            value = hex_to_uint(command, i);

            if (i == value_size * 2) {
                putchar('\n');
            }
        } else {
            strip = getchar();
            zone = getchar();

            get_chars_silent(command, value_size);

            value = ((uint32_t) (command[0]) << 24)
                | ((uint32_t) (command[1]) << 16)
                | ((uint32_t) (command[2]) << 8)
                | (uint32_t) (command[3]);
            value = value >> (4 - value_size) * 8;
        }

        if (strip >= LED_STRIPS) {
            if (text) {
                puts("Strip count too high\n");
            }

            continue;
        }

        if (zone >= MAX_ZONES_PER_STRIP) {
            if (text) {
                puts("Zone count too high\n");
            }

            continue;
        }

        mutex_enter_blocking(&zone_lock);

        if (field == 0) {
            zones[strip][zone].off = value;
        } else if (field == 1) {
            zones[strip][zone].on = value;
        } else if (field == 2) {
            zones[strip][zone].effect = value;
        }

        mutex_exit(&zone_lock);
    }
}

int main() {
    stdio_init_all();

    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &musical_leds_program);

    musical_leds_program_init(pio, sm, offset, LED_PIN_START, LED_STRIPS, 800000);

    sem_init(&reset_delay_complete_sem, 1, 1);
    dma_init(pio, sm);

    mutex_init(&zone_lock);

    multicore_launch_core1(core1_entry);

    uint t = 0;
    uint current_framebuffer = 0;

    while (1) {
        render_zones(t);
        translate_to_framebuffer(pixels, framebuffers[current_framebuffer]);
        sem_acquire_blocking(&reset_delay_complete_sem);
        output_strips_dma(framebuffers[current_framebuffer]);

        current_framebuffer ^= 1;
        t++;
    }
}
