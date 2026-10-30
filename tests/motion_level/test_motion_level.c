#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "motion_level.h"

#define WIDTH 160
#define HEIGHT 128
#define THEME 0xF6D34A
#define INK 0x1B1713
#define GREEN 0x55C76A
#define RAD_PER_DEG 0.0174532925f

static uint16_t draw_buffer[WIDTH * HEIGHT];
static uint16_t pixels[WIDTH * HEIGHT];

static void flush_cb(lv_display_t *display, const lv_area_t *area, uint8_t *data)
{
    const int width = area->x2 - area->x1 + 1;
    for (int y = area->y1; y <= area->y2; ++y) {
        memcpy(&pixels[y * WIDTH + area->x1], data, width * sizeof(uint16_t));
        data += width * sizeof(uint16_t);
    }
    lv_display_flush_ready(display);
}

static uint16_t color(uint32_t hex)
{
    return lv_color_to_u16(lv_color_hex(hex));
}

static void assert_background_corners(uint32_t hex)
{
    const uint16_t expected = color(hex);
    assert(pixels[0] == expected);
    assert(pixels[WIDTH - 1] == expected);
    assert(pixels[(HEIGHT - 1) * WIDTH] == expected);
    assert(pixels[WIDTH * HEIGHT - 1] == expected);
}

static bool dark(uint16_t pixel)
{
    return ((pixel >> 11) & 31) < 12 && ((pixel >> 5) & 63) < 24 && (pixel & 31) < 12;
}

static int horizon_boundary_y(void)
{
    for (int y = 26; y < HEIGHT; ++y) {
        if (pixels[y * WIDTH + 10] != color(0xFFFFFF)) {
            return y;
        }
    }
    return HEIGHT;
}

static void settle(lv_obj_t *obj, float x, float y, float z, int count)
{
    const int16_t acc[3] = {lroundf(x * 16384), lroundf(y * 16384), lroundf(z * 16384)};
    for (int i = 0; i < count; ++i) {
        lv_tick_inc(16);
        motion_level_update(obj, acc, true);
        lv_timer_handler();
    }
    lv_refr_now(NULL);
}

static void face_tilt(lv_obj_t *obj, float degrees)
{
    settle(obj, 0, sinf(degrees * RAD_PER_DEG), cosf(degrees * RAD_PER_DEG), 240);
}

static void tilt_over_top(lv_obj_t *obj, float degrees)
{
    settle(obj, sinf(degrees * RAD_PER_DEG), 0, cosf(degrees * RAD_PER_DEG), 240);
}

static void horizon_pose(lv_obj_t *obj, float pitch, float roll)
{
    const float projection = cosf(pitch * RAD_PER_DEG);
    settle(obj, projection * cosf(roll * RAD_PER_DEG), -projection * sinf(roll * RAD_PER_DEG),
           sinf(pitch * RAD_PER_DEG), 240);
}

static void edge_tilt(lv_obj_t *obj, float degrees)
{
    settle(obj, cosf(degrees * RAD_PER_DEG), -sinf(degrees * RAD_PER_DEG), 0, 240);
}

static bool label_visible(lv_obj_t *obj, int child)
{
    return !lv_obj_has_flag(lv_obj_get_child(obj, child), LV_OBJ_FLAG_HIDDEN);
}

static bool visible(lv_obj_t *obj, int child)
{
    return label_visible(obj, child);
}

static lv_opa_t label_opacity(lv_obj_t *obj, int child)
{
    return label_visible(obj, child) ? lv_obj_get_style_text_opa(lv_obj_get_child(obj, child), 0) : LV_OPA_TRANSP;
}

static lv_opa_t text_opacity(lv_obj_t *obj, int child)
{
    return label_opacity(obj, child);
}

static void assert_readout_handoff(lv_obj_t *obj)
{
    const int bubble = text_opacity(obj, 0);
    const int horizon = text_opacity(obj, 2);
    assert(bubble > 0 || horizon > 0);
    assert(bubble + horizon >= 254 && bubble + horizon <= 256);
    assert(lv_obj_get_child_count(obj) == 5);
    if (horizon > 0) {
        lv_obj_t *label = lv_obj_get_child(obj, 2);
        assert(lv_obj_get_y(label) >= 26);
        assert(lv_obj_get_y(label) + lv_obj_get_height(label) <= HEIGHT);
    }
}

static int angle_center_y(lv_obj_t *obj)
{
    lv_obj_t *label = lv_obj_get_child(obj, 2);
    return lv_obj_get_y(label) + lv_obj_get_height(label) / 2;
}

static int straight_horizon_y(float pitch)
{
    return 76 + lroundf(80.0f * pitch / 60.0f);
}

static int readout_tenths(lv_obj_t *obj, int child)
{
    const char *text = lv_label_get_text(lv_obj_get_child(obj, child));
    const char *newline = strchr(text, '\n');
    if (newline) text = newline + 1;
    char sign;
    int whole, fraction;
    assert(sscanf(text, "%c%d.%d", &sign, &whole, &fraction) == 3);
    return (sign == '-' ? -1 : 1) * (whole * 10 + fraction);
}

static void screenshot(const char *name)
{
    FILE *file = fopen(name, "wb");
    assert(file);
    fprintf(file, "P6\n%d %d\n255\n", WIDTH, HEIGHT);
    for (int i = 0; i < WIDTH * HEIGHT; ++i) {
        const uint16_t pixel = pixels[i];
        const unsigned char rgb[3] = {
            ((pixel >> 11) & 31) * 255 / 31,
            ((pixel >> 5) & 63) * 255 / 63,
            (pixel & 31) * 255 / 31,
        };
        assert(fwrite(rgb, sizeof(rgb), 1, file) == 1);
    }
    assert(fclose(file) == 0);
}

static void add_header(lv_obj_t *page)
{
    lv_obj_t *title = lv_label_create(page);
    lv_label_set_text(title, "MOTION");
    lv_obj_set_pos(title, 8, 7);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(INK), 0);
    lv_obj_t *index = lv_label_create(page);
    lv_label_set_text(index, "03/16");
    lv_obj_set_pos(index, 8, 7);
    lv_obj_set_width(index, 144);
    lv_obj_set_style_text_align(index, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_font(index, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(index, lv_color_hex(0x5C4220), 0);
}

int main(void)
{
    lv_init();
    lv_display_t *display = lv_display_create(WIDTH, HEIGHT);
    lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);
    lv_display_set_buffers(display, draw_buffer, NULL, sizeof(draw_buffer), LV_DISPLAY_RENDER_MODE_FULL);
    lv_display_set_flush_cb(display, flush_cb);
    lv_obj_t *page = lv_obj_create(lv_screen_active());
    lv_obj_remove_style_all(page);
    lv_obj_set_size(page, WIDTH, HEIGHT);
    lv_obj_clear_flag(page, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *obj = motion_level_create(page, THEME, INK);
    add_header(page);
    assert(lv_obj_get_width(obj) == WIDTH);

    const int16_t zero[3] = {0};
    motion_level_update(obj, zero, false);
    lv_refr_now(display);
    assert(visible(obj, 3) && !visible(obj, 0) && !visible(obj, 2));
    assert(pixels[120 * WIDTH + 10] == color(THEME));
    screenshot("motion-absent.ppm");
    motion_level_update(obj, zero, true);
    assert(visible(obj, 3));

    face_tilt(obj, 0);
    assert(!visible(obj, 0) && !visible(obj, 1) && !visible(obj, 2) && !visible(obj, 3));
    assert(pixels[120 * WIDTH + 10] == color(GREEN));
    assert(dark(pixels[76 * WIDTH + 80]));
    screenshot("motion-bubble-level.ppm");

    settle(obj, -0.12f, 0.08f, 0.99f, 240);
    assert(pixels[120 * WIDTH + 10] == color(0xFFFFFF));
    assert(dark(pixels[88 * WIDTH + 88]));
    assert(readout_tenths(obj, 0) >= 45 && readout_tenths(obj, 0) <= 47);
    assert(readout_tenths(obj, 1) >= -70 && readout_tenths(obj, 1) <= -68);
    screenshot("motion-bubble-tilted.ppm");

    face_tilt(obj, 40);
    assert(!visible(obj, 0) && !visible(obj, 1));
    assert(pixels[0] == color(THEME) && pixels[WIDTH - 1] == color(0xFFFFFF));
    settle(obj, 0, sinf(25 * RAD_PER_DEG), cosf(25 * RAD_PER_DEG), 30);
    const lv_opa_t bubble_after_clear = text_opacity(obj, 0);
    assert(bubble_after_clear > LV_OPA_TRANSP && bubble_after_clear < LV_OPA_COVER);
    assert_readout_handoff(obj);
    assert_background_corners(0xFFFFFF);
    screenshot("motion-transition.ppm");
    face_tilt(obj, 25);
    assert(text_opacity(obj, 0) == LV_OPA_COVER);
    for (int direction = 0; direction < 2; ++direction) {
        const float degrees = direction == 0 ? 40 : 25;
        for (int i = 0; i < 90; ++i) {
            settle(obj, 0, sinf(degrees * RAD_PER_DEG), cosf(degrees * RAD_PER_DEG), 1);
            assert_readout_handoff(obj);
        }
    }
    face_tilt(obj, 55);
    assert(!visible(obj, 0));
    face_tilt(obj, 70);
    assert(!visible(obj, 0));
    face_tilt(obj, 35);
    assert(!visible(obj, 0));
    assert(visible(obj, 2));

    horizon_pose(obj, 45, 45);
    assert(!visible(obj, 0) && !visible(obj, 1));
    assert(visible(obj, 2));
    assert(lv_obj_get_y(lv_obj_get_child(obj, 2)) >= 26);
    assert(lv_obj_get_y(lv_obj_get_child(obj, 2)) + lv_obj_get_height(lv_obj_get_child(obj, 2)) <= HEIGHT);
    assert(pixels[0] == color(0xFFFFFF));
    assert(pixels[WIDTH * HEIGHT - 1] == color(THEME));
    horizon_pose(obj, 55, 45);
    assert(visible(obj, 0) && visible(obj, 1));
    assert_background_corners(0xFFFFFF);
    horizon_pose(obj, -55, 45);
    assert(!visible(obj, 0) && !visible(obj, 1));
    assert(visible(obj, 2));
    assert(lv_obj_get_y(lv_obj_get_child(obj, 2)) >= 26);
    assert(lv_obj_get_y(lv_obj_get_child(obj, 2)) + lv_obj_get_height(lv_obj_get_child(obj, 2)) <= HEIGHT);
    assert(pixels[0] == color(0xFFFFFF));
    assert(pixels[WIDTH * HEIGHT - 1] == color(THEME));
    horizon_pose(obj, -65, 45);
    assert(visible(obj, 0) && visible(obj, 1));
    assert_background_corners(THEME);

    edge_tilt(obj, 12);
    assert(!visible(obj, 0) && visible(obj, 2));
    assert(readout_tenths(obj, 2) == 0);
    assert(pixels[30 * WIDTH + 80] == color(0xFFFFFF));
    assert(pixels[120 * WIDTH + 80] == color(THEME));
    screenshot("motion-horizon-tilted.ppm");
    assert(dark(pixels[76 * WIDTH + 80]));

    edge_tilt(obj, 5);
    const lv_opa_t text_at_5 = text_opacity(obj, 2);
    edge_tilt(obj, 2.5f);
    const lv_opa_t text_at_2_5 = text_opacity(obj, 2);
    assert(text_at_2_5 > LV_OPA_TRANSP && text_at_2_5 < text_at_5);
    assert(pixels[120 * WIDTH + 10] != color(THEME));
    assert(pixels[120 * WIDTH + 10] != color(GREEN));
    screenshot("motion-horizon-near-level.ppm");
    edge_tilt(obj, 1);
    assert(text_opacity(obj, 2) > LV_OPA_TRANSP && text_opacity(obj, 2) < text_at_2_5);
    edge_tilt(obj, 0);
    assert(!visible(obj, 2));
    assert(pixels[30 * WIDTH + 80] == color(0xFFFFFF));
    assert(pixels[120 * WIDTH + 80] == color(GREEN));
    assert(pixels[76 * WIDTH + 80] != color(INK));
    screenshot("motion-horizon-level.ppm");

    tilt_over_top(obj, 75);
    assert(visible(obj, 2));
    assert(angle_center_y(obj) < straight_horizon_y(15));
    assert(readout_tenths(obj, 2) >= 149 && readout_tenths(obj, 2) <= 151);
    assert(pixels[120 * WIDTH + 10] == color(THEME));
    tilt_over_top(obj, 105);
    assert(visible(obj, 2));
    assert(angle_center_y(obj) > straight_horizon_y(-15));
    assert(readout_tenths(obj, 2) >= -151 && readout_tenths(obj, 2) <= -149);
    assert(pixels[120 * WIDTH + 10] == color(THEME));
    edge_tilt(obj, 12);
    assert(visible(obj, 2));
    assert(readout_tenths(obj, 2) == 0);
    assert(pixels[120 * WIDTH + 10] == color(THEME));
    edge_tilt(obj, 90);
    assert(visible(obj, 2));
    assert(readout_tenths(obj, 2) == 0);
    assert(pixels[120 * WIDTH + 120] == color(THEME));
    tilt_over_top(obj, 87.5f);
    assert(visible(obj, 2));
    assert(pixels[120 * WIDTH + 10] != color(THEME));
    assert(pixels[120 * WIDTH + 10] != color(GREEN));
    tilt_over_top(obj, 90);
    assert(!visible(obj, 2));
    assert(pixels[120 * WIDTH + 10] == color(GREEN));

    horizon_pose(obj, 8, 0);
    assert(visible(obj, 2));
    assert(angle_center_y(obj) < straight_horizon_y(8));
    horizon_pose(obj, 4, 0);
    assert(angle_center_y(obj) < straight_horizon_y(4));
    horizon_pose(obj, 0, 0);
    assert(!visible(obj, 2));
    horizon_pose(obj, -4, 0);
    assert(angle_center_y(obj) < straight_horizon_y(-4));
    horizon_pose(obj, -8, 0);
    assert(angle_center_y(obj) > straight_horizon_y(-8));
    for (int i = 0; i < 4; ++i) {
        const float pitch = i % 2 ? 4.0f : -4.0f;
        horizon_pose(obj, pitch, 0);
        assert(angle_center_y(obj) > straight_horizon_y(pitch));
    }
    horizon_pose(obj, -8, 0);
    bool saw_slide = false;
    for (int i = 0; i < 64; ++i) {
        settle(obj, cosf(8 * RAD_PER_DEG), 0, sinf(8 * RAD_PER_DEG), 1);
        const int offset = angle_center_y(obj) - horizon_boundary_y();
        if (offset > -12 && offset < 12) saw_slide = true;
        assert(lv_obj_get_child_count(obj) == 5);
    }
    assert(saw_slide);
    assert(angle_center_y(obj) < straight_horizon_y(8));
    horizon_pose(obj, 0, 12);
    assert(visible(obj, 2));
    assert(lv_obj_get_child_count(obj) == 5);

    for (int degrees = -180; degrees <= 180; degrees += 15) {
        edge_tilt(obj, degrees);
        const bool level = degrees % 180 == 0;
        assert(visible(obj, 2) != level);
        for (int i = 0; i < 4; ++i) {
            lv_obj_t *label = lv_obj_get_child(obj, i);
            if (!visible(obj, i)) continue;
            assert(lv_obj_get_x(label) >= 0 && lv_obj_get_y(label) >= 26);
            assert(lv_obj_get_x(label) + lv_obj_get_width(label) <= WIDTH);
            assert(lv_obj_get_y(label) + lv_obj_get_height(label) <= HEIGHT);
        }
        assert(lv_obj_get_child_count(obj) == 5);
    }
    edge_tilt(obj, 78);
    screenshot("motion-horizon-portrait.ppm");
    int previous_boundary = -1;
    for (int degrees = 70; degrees <= 110; degrees += 10) {
        tilt_over_top(obj, degrees);
        const int boundary = horizon_boundary_y();
        if (previous_boundary >= 0) {
            const int step = previous_boundary - boundary;
            assert(step >= 12 && step <= 15);
        }
        previous_boundary = boundary;
    }
    tilt_over_top(obj, 20);
    assert(pixels[120 * WIDTH + 10] == color(0xFFFFFF));
    assert(readout_tenths(obj, 1) >= 199 && readout_tenths(obj, 1) <= 201);
    tilt_over_top(obj, 40);
    assert(pixels[90 * WIDTH + 10] == color(0xFFFFFF));
    tilt_over_top(obj, 90);
    assert(pixels[30 * WIDTH + 10] == color(0xFFFFFF));
    assert(pixels[120 * WIDTH + 10] == color(GREEN));
    tilt_over_top(obj, 120);
    assert(pixels[90 * WIDTH + 10] != color(0xFFFFFF));
    screenshot("motion-horizon-over-top.ppm");
    tilt_over_top(obj, 140);
    assert(pixels[90 * WIDTH + 10] != color(0xFFFFFF));
    screenshot("motion-transition-face-down.ppm");
    tilt_over_top(obj, 160);
    assert(visible(obj, 0) && visible(obj, 1) && !visible(obj, 2));
    assert(readout_tenths(obj, 1) >= 199 && readout_tenths(obj, 1) <= 201);
    assert(pixels[120 * WIDTH + 10] == color(THEME));
    screenshot("motion-bubble-face-down.ppm");
    tilt_over_top(obj, 180);
    assert(!visible(obj, 0) && !visible(obj, 1));
    assert(pixels[120 * WIDTH + 10] == color(GREEN));

    motion_level_update(obj, zero, false);
    lv_refr_now(display);
    assert(visible(obj, 3) && !visible(obj, 0) && !visible(obj, 2));
    assert(pixels[120 * WIDTH + 10] == color(THEME));
    settle(obj, 0, 0, 1, 1);
    assert(visible(obj, 0));
    assert(pixels[120 * WIDTH + 10] != color(GREEN));
    settle(obj, 0, 0, 1, 240);
    assert(!visible(obj, 0) && !visible(obj, 1));
    assert(pixels[120 * WIDTH + 10] == color(GREEN));

    /* A retiring page keeps its own draw state while the next page slides in. */
    lv_obj_set_x(page, -40);
    lv_refr_now(display);
    assert(pixels[76 * WIDTH + 40] == color(INK));
    lv_obj_delete(page);
    for (int i = 0; i < 100; ++i) {
        lv_obj_t *temp = motion_level_create(lv_screen_active(), THEME, INK);
        settle(temp, 0, 0, 1, 1);
        lv_obj_delete(temp);
    }

    lv_obj_t *sliding = motion_level_create(lv_screen_active(), THEME, INK);
    horizon_pose(sliding, -8, 0);
    settle(sliding, cosf(8 * RAD_PER_DEG), 0, sinf(8 * RAD_PER_DEG), 24);
    lv_obj_delete(sliding);
    lv_tick_inc(250);
    lv_timer_handler();

    lv_obj_t *calibrated = motion_level_create(lv_screen_active(), THEME, INK);
    settle(calibrated, 0.06f, -0.04f, 0.997f, 240);
    assert(readout_tenths(calibrated, 0) != 0 && readout_tenths(calibrated, 1) != 0);
    assert(motion_level_calibrate(calibrated));
    settle(calibrated, 0.06f, -0.04f, 0.997f, 240);
    assert(readout_tenths(calibrated, 0) == 0 && readout_tenths(calibrated, 1) == 0);
    assert(pixels[120 * WIDTH + 10] == color(GREEN));
    assert(dark(pixels[76 * WIDTH + 80]));
    lv_obj_delete(calibrated);

    calibrated = motion_level_create(lv_screen_active(), THEME, INK);
    settle(calibrated, 0.06f, -0.04f, 0.997f, 240);
    assert(readout_tenths(calibrated, 0) == 0 && readout_tenths(calibrated, 1) == 0);
    assert(pixels[120 * WIDTH + 10] == color(GREEN));
    motion_level_update(calibrated, zero, false);
    assert(!motion_level_calibrate(calibrated));
    lv_obj_delete(calibrated);
    lv_display_delete(display);
    lv_deinit();
    puts("Motion level: continuous fades, single readout, hysteresis/sliding, four edges, layout, reconnect, calibration and page lifetime passed.");
    return 0;
}
