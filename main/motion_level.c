#include "motion_level.h"

#include <math.h>

#define LEVEL_GREEN             0x55C76A
#define LEVEL_DEG_PER_RAD       57.2957795f
#define LEVEL_FILTER_MS         160.0f
#define LEVEL_READOUT_FILTER_MS 220.0f
#define LEVEL_TRANSITION_MS     140.0f
#define LEVEL_COLOR_FILTER_MS   180.0f
#define LEVEL_READOUT_PERIOD_MS 100
#define LEVEL_GREEN_FULL_DEG    0.35f
#define LEVEL_GREEN_START_DEG   5.0f
#define LEVEL_HORIZON_SWEEP_DEG 60.0f
#define LEVEL_HORIZON_CLEAR_PX  2.0f
#define LEVEL_CAL_SETTLE_MS     250
#define LEVEL_CAL_SAMPLE_MS     600
#define LEVEL_CAL_TIMEOUT_MS    3000
#define LEVEL_CAL_MIN_SAMPLES   20
#define LEVEL_CAL_MAX_VARIANCE  0.0004f
#define LEVEL_CAL_STATUS_MS     1400
#define LEVEL_RING_RADIUS       36
#define LEVEL_BUBBLE_RADIUS     14
#define LEVEL_BUBBLE_TRAVEL     ((float)(LEVEL_RING_RADIUS - LEVEL_BUBBLE_RADIUS - 2))
#define LEVEL_BUBBLE_RANGE_DEG  12.0f
#define LEVEL_CONTENT_TOP       26
#define LEVEL_ANGLE_WIDTH       52
#define LEVEL_ANGLE_HEIGHT      16
#define LEVEL_ANGLE_SWITCH_DEG  6.0f
#define LEVEL_ANGLE_SLIDE_MS    220
#define LEVEL_ANGLE_LINE_GAP    8.0f

typedef struct {
    lv_obj_t *x_label;
    lv_obj_t *y_label;
    lv_obj_t *angle_label;
    lv_obj_t *missing_label;
    lv_obj_t *calibration_label;
    uint32_t theme_color;
    uint32_t ink_color;
    uint32_t last_tick;
    uint32_t last_readout_tick;
    uint32_t calibration_tick;
    uint32_t calibration_window_tick;
    uint32_t calibration_sample_tick;
    uint32_t calibration_status_tick;
    uint32_t calibration_samples;
    float gravity[3];
    float calibration_sum[3];
    float calibration_squared_sum;
    float x_deg;
    float y_deg;
    float normal_x;
    float normal_y;
    float horizon_pitch_deg;
    float horizon_mix;
    float sky_mix;
    float bubble_green;
    float horizon_green;
    float angle_side;
    float readout_deg[3];
    int readout_tenths[3];
    bool ready;
    bool horizon_offscreen;
    bool angle_below;
    bool angle_position_ready;
    bool calibrating;
    bool calibration_status_active;
} motion_level_t;

/* Keep the zero across page recreation without writing flash on each calibration. */
static float s_level_alignment[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f},
};

static float level_smoothstep(float low, float high, float value)
{
    const float t = fminf(1.0f, fmaxf(0.0f, (value - low) / (high - low)));
    return t * t * (3.0f - 2.0f * t);
}

static float level_follow(float value, float target, float alpha)
{
    value += alpha * (target - value);
    return fabsf(target - value) < 0.001f ? target : value;
}

static lv_opa_t level_opacity(float amount)
{
    return (lv_opa_t)lroundf(fminf(1.0f, fmaxf(0.0f, amount)) * LV_OPA_COVER);
}

static float level_bubble_visibility(const motion_level_t *state)
{
    return 1.0f - level_smoothstep(0.0f, 0.85f, state->horizon_mix);
}

static float level_horizon_visibility(const motion_level_t *state)
{
    /* The number takes over exactly as the bubble fades; only level green suppresses it. */
    return (1.0f - level_bubble_visibility(state)) * (1.0f - state->horizon_green);
}

static void level_center(lv_obj_t *obj, int32_t *x, int32_t *y)
{
    *x = lv_obj_get_width(obj) / 2;
    *y = (lv_obj_get_height(obj) + 24) / 2;
}

static void level_horizon_center(lv_obj_t *obj, const motion_level_t *state, int32_t *x, int32_t *y)
{
    level_center(obj, x, y);
    const float side_distance = fabsf(state->normal_x) * lv_obj_get_width(obj) / 2.0f;
    const float sky_distance = side_distance +
                               fabsf(state->normal_y) * (state->normal_y >= 0 ? *y : lv_obj_get_height(obj) - *y) + 4.0f;
    const float ground_distance = side_distance +
                                  fabsf(state->normal_y) * (state->normal_y >= 0 ? lv_obj_get_height(obj) - *y : *y) + 4.0f;
    /* Complete the linear sweep before the flat pose, leaving room for the bubble fade. */
    const float slide = fmaxf(sky_distance, ground_distance) * state->horizon_pitch_deg / LEVEL_HORIZON_SWEEP_DEG;
    *x += lroundf(state->normal_x * slide);
    *y += lroundf(state->normal_y * slide);
}

static bool level_horizon_offscreen(lv_obj_t *obj, const motion_level_t *state)
{
    int32_t hx, hy;
    level_horizon_center(obj, state, &hx, &hy);
    const float left = -state->normal_x * hx;
    const float right = state->normal_x * (lv_obj_get_width(obj) - 1 - hx);
    const float top = -state->normal_y * hy;
    const float bottom = state->normal_y * (lv_obj_get_height(obj) - 1 - hy);
    const float nearest = fminf(left, right) + fminf(top, bottom);
    const float farthest = fmaxf(left, right) + fmaxf(top, bottom);
    /* Every corner must lie on the same side, clear of the line's antialiased stroke. */
    return nearest > LEVEL_HORIZON_CLEAR_PX || farthest < -LEVEL_HORIZON_CLEAR_PX;
}

static void level_angle_place(lv_obj_t *obj, const motion_level_t *state)
{
    int32_t hx, hy;
    level_horizon_center(obj, state, &hx, &hy);
    const float clearance = fabsf(state->normal_x) * LEVEL_ANGLE_WIDTH / 2.0f +
                            fabsf(state->normal_y) * LEVEL_ANGLE_HEIGHT / 2.0f + LEVEL_ANGLE_LINE_GAP;
    const float distance = state->angle_side * clearance;
    const int32_t x = LV_MAX(2, LV_MIN(hx + lroundf(state->normal_x * distance) - LEVEL_ANGLE_WIDTH / 2,
                                     lv_obj_get_width(obj) - LEVEL_ANGLE_WIDTH - 2));
    const int32_t y = LV_MAX(LEVEL_CONTENT_TOP,
                             LV_MIN(hy + lroundf(state->normal_y * distance) - LEVEL_ANGLE_HEIGHT / 2,
                                    lv_obj_get_height(obj) - LEVEL_ANGLE_HEIGHT));
    lv_obj_set_pos(state->angle_label, x, y);
    const bool overlaps_footer = level_horizon_visibility(state) > 0.0f &&
                                  y + LEVEL_ANGLE_HEIGHT > lv_obj_get_height(obj) - 14;
    lv_obj_set_pos(state->calibration_label, 8,
                   overlaps_footer ? LEVEL_CONTENT_TOP : lv_obj_get_height(obj) - 14);
}

static void level_angle_slide_cb(void *var, int32_t value)
{
    motion_level_t *state = var;
    state->angle_side = value / 1024.0f;
    level_angle_place(lv_obj_get_parent(state->angle_label), state);
}

static void level_angle_update_side(lv_obj_t *obj, motion_level_t *state)
{
    if (!state->angle_position_ready) {
        state->angle_below = state->horizon_pitch_deg < 0.0f;
        state->angle_side = state->angle_below ? 1.0f : -1.0f;
        state->angle_position_ready = true;
    }
    else {
        bool below = state->angle_below;
        if (state->horizon_pitch_deg > LEVEL_ANGLE_SWITCH_DEG) below = false;
        else if (state->horizon_pitch_deg < -LEVEL_ANGLE_SWITCH_DEG) below = true;
        if (below != state->angle_below) {
            state->angle_below = below;
            lv_anim_t animation;
            lv_anim_init(&animation);
            lv_anim_set_var(&animation, state);
            lv_anim_set_exec_cb(&animation, level_angle_slide_cb);
            lv_anim_set_values(&animation, lroundf(state->angle_side * 1024.0f), below ? 1024 : -1024);
            lv_anim_set_duration(&animation, LEVEL_ANGLE_SLIDE_MS);
            lv_anim_set_path_cb(&animation, lv_anim_path_ease_in_out);
            if (!lv_anim_start(&animation)) {
                state->angle_side = below ? 1.0f : -1.0f;
            }
        }
    }
    level_angle_place(obj, state);
}

static void level_draw_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target_obj(e);
    const motion_level_t *state = lv_event_get_user_data(e);
    lv_layer_t *layer = lv_event_get_layer(e);
    lv_area_t bounds;
    lv_obj_get_coords(obj, &bounds);

    const lv_color_t theme = lv_color_hex(state->theme_color);
    const lv_color_t green = lv_color_hex(LEVEL_GREEN);
    const lv_color_t bubble_base = lv_color_mix(lv_color_white(), theme, level_opacity(state->sky_mix));
    const lv_color_t bubble_fill = lv_color_mix(green, bubble_base, level_opacity(state->bubble_green));
    const lv_color_t horizon_fill = lv_color_mix(green, theme, level_opacity(state->horizon_green));
    const float terrain_mix = state->horizon_offscreen ? state->horizon_mix : 1.0f;
    lv_draw_rect_dsc_t rect;
    lv_draw_rect_dsc_init(&rect);
    rect.bg_color = state->ready ? lv_color_mix(lv_color_white(), bubble_fill, level_opacity(terrain_mix)) : theme;
    rect.bg_opa = LV_OPA_COVER;
    lv_draw_rect(layer, &rect, &bounds);

    if (!state->ready) {
        return;
    }

    int32_t cx, cy;
    level_center(obj, &cx, &cy);
    cx += bounds.x1;
    cy += bounds.y1;

    if (terrain_mix > 0.0f) {
        int32_t hx, hy;
        level_horizon_center(obj, state, &hx, &hy);
        hx += bounds.x1;
        hy += bounds.y1;
        const float span = 2.0f * (lv_obj_get_width(obj) + lv_obj_get_height(obj));
        const float tx = state->normal_y * span;
        const float ty = -state->normal_x * span;
        lv_draw_triangle_dsc_t ground;
        lv_draw_triangle_dsc_init(&ground);
        ground.color = lv_color_mix(horizon_fill, bubble_fill, level_opacity(terrain_mix));
        /* One oversized triangle covers the ground half-plane without a diagonal seam. */
        ground.p[0] = (lv_point_precise_t){hx - lroundf(tx), hy - lroundf(ty)};
        ground.p[1] = (lv_point_precise_t){hx + lroundf(tx), hy + lroundf(ty)};
        ground.p[2] = (lv_point_precise_t){hx + lroundf(state->normal_x * span),
                                        hy + lroundf(state->normal_y * span)};
        lv_draw_triangle(layer, &ground);

        const lv_opa_t horizon_opa = level_opacity(level_horizon_visibility(state));
        if (horizon_opa > LV_OPA_TRANSP) {
            lv_draw_line_dsc_t line;
            lv_draw_line_dsc_init(&line);
            line.p1 = ground.p[0];
            line.p2 = ground.p[1];
            line.width = 2;
            line.color = lv_color_hex(state->ink_color);
            line.opa = horizon_opa;
            lv_draw_line(layer, &line);
        }
    }

    const lv_opa_t bubble_opa = level_opacity(level_bubble_visibility(state));
    if (bubble_opa == LV_OPA_TRANSP) {
        return;
    }

    rect.radius = LV_RADIUS_CIRCLE;
    rect.bg_opa = LV_OPA_TRANSP;
    rect.border_color = lv_color_hex(state->ink_color);
    rect.border_width = 2;
    rect.border_opa = bubble_opa;
    const lv_area_t ring = {cx - LEVEL_RING_RADIUS, cy - LEVEL_RING_RADIUS,
                           cx + LEVEL_RING_RADIUS, cy + LEVEL_RING_RADIUS};
    lv_draw_rect(layer, &rect, &ring);

    float dx = state->x_deg * LEVEL_BUBBLE_TRAVEL / LEVEL_BUBBLE_RANGE_DEG;
    float dy = state->y_deg * LEVEL_BUBBLE_TRAVEL / LEVEL_BUBBLE_RANGE_DEG;
    const float distance = sqrtf(dx * dx + dy * dy);
    if (distance > LEVEL_BUBBLE_TRAVEL) {
        dx *= LEVEL_BUBBLE_TRAVEL / distance;
        dy *= LEVEL_BUBBLE_TRAVEL / distance;
    }
    cx += lroundf(dx);
    cy += lroundf(dy);
    const lv_area_t bubble = {cx - LEVEL_BUBBLE_RADIUS, cy - LEVEL_BUBBLE_RADIUS,
                             cx + LEVEL_BUBBLE_RADIUS, cy + LEVEL_BUBBLE_RADIUS};
    rect.bg_opa = bubble_opa;
    rect.bg_color = lv_color_hex(state->ink_color);
    rect.border_width = 0;
    lv_draw_rect(layer, &rect, &bubble);
}

static void level_delete_cb(lv_event_t *e)
{
    motion_level_t *state = lv_event_get_user_data(e);
    lv_anim_delete(state, level_angle_slide_cb);
    lv_free(state);
}

static lv_obj_t *level_label(lv_obj_t *parent, uint32_t color, const lv_font_t *font)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, "");
    lv_label_set_long_mode(label, LV_LABEL_LONG_MODE_CLIP);
    lv_obj_set_style_text_color(label, lv_color_hex(color), 0);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    return label;
}

static void level_label_degrees(lv_obj_t *label, const char *prefix, int tenths)
{
    /* LVGL's built-in formatter has floating-point support disabled. */
    const int magnitude = tenths < 0 ? -tenths : tenths;
    lv_label_set_text_fmt(label, "%s%c%d.%d\xC2\xB0", prefix, tenths < 0 ? '-' : '+',
                          magnitude / 10, magnitude % 10);
}

static void level_label_visibility(lv_obj_t *label, float visibility)
{
    const lv_opa_t opa = level_opacity(visibility);
    lv_obj_set_style_text_opa(label, opa, 0);
    lv_obj_set_flag(label, LV_OBJ_FLAG_HIDDEN, opa == LV_OPA_TRANSP);
}

static void level_calibration_status(motion_level_t *state, const char *text)
{
    state->calibration_status_active = true;
    state->calibration_status_tick = lv_tick_get();
    lv_label_set_text(state->calibration_label, text);
}

static void level_calibration_reset_window(motion_level_t *state)
{
    state->calibration_samples = 0;
    state->calibration_squared_sum = 0.0f;
    for (int i = 0; i < 3; ++i) {
        state->calibration_sum[i] = 0.0f;
    }
}

bool motion_level_calibrate(lv_obj_t *obj)
{
    if (!obj) {
        return false;
    }
    motion_level_t *state = lv_obj_get_user_data(obj);
    if (!state->ready) {
        return false;
    }
    if (state->calibrating) {
        return true;
    }

    state->calibrating = true;
    state->calibration_tick = lv_tick_get();
    state->calibration_sample_tick = state->calibration_tick;
    level_calibration_reset_window(state);
    level_calibration_status(state, "CALIBRATING");
    return true;
}

static void level_calibration_alignment(const float mean[3], float magnitude)
{
    const float reference[3] = {-mean[1] / magnitude, mean[0] / magnitude, mean[2] / magnitude};
    int base = 0;
    for (int i = 1; i < 3; ++i) {
        if (fabsf(reference[i]) > fabsf(reference[base])) {
            base = i;
        }
    }
    float target[3] = {0.0f};
    target[base] = reference[base] >= 0.0f ? 1.0f : -1.0f;
    const float cross[3] = {
        reference[1] * target[2] - reference[2] * target[1],
        reference[2] * target[0] - reference[0] * target[2],
        reference[0] * target[1] - reference[1] * target[0],
    };
    const float cosine = fabsf(reference[base]);
    const float scale = 1.0f / (1.0f + cosine);
    const float skew[3][3] = {
        {0.0f, -cross[2], cross[1]},
        {cross[2], 0.0f, -cross[0]},
        {-cross[1], cross[0], 0.0f},
    };
    /* The nearest signed axis avoids antiparallel vectors in Rodrigues' rotation. */
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            s_level_alignment[row][col] = (row == col ? cosine : 0.0f) +
                                          cross[row] * cross[col] * scale + skew[row][col];
        }
    }
}

static bool level_calibration_collect(motion_level_t *state, const int16_t acc[3])
{
    if (!state->calibrating) {
        return false;
    }
    const uint32_t elapsed = lv_tick_elaps(state->calibration_tick);
    if (elapsed >= LEVEL_CAL_TIMEOUT_MS) {
        state->calibrating = false;
        level_calibration_status(state, "KEEP STILL");
        return false;
    }
    if (elapsed < LEVEL_CAL_SETTLE_MS || lv_tick_elaps(state->calibration_sample_tick) < 8) {
        return false;
    }

    const uint32_t now = lv_tick_get();
    state->calibration_sample_tick = now;
    if (state->calibration_samples == 0) {
        state->calibration_window_tick = now;
    }
    for (int i = 0; i < 3; ++i) {
        const float sample = acc[i] / 16384.0f;
        state->calibration_sum[i] += sample;
        state->calibration_squared_sum += sample * sample;
    }
    state->calibration_samples++;
    if (lv_tick_elaps(state->calibration_window_tick) < LEVEL_CAL_SAMPLE_MS ||
        state->calibration_samples < LEVEL_CAL_MIN_SAMPLES) {
        return false;
    }

    float mean[3];
    float magnitude_squared = 0.0f;
    for (int i = 0; i < 3; ++i) {
        mean[i] = state->calibration_sum[i] / state->calibration_samples;
        magnitude_squared += mean[i] * mean[i];
    }
    const float variance = state->calibration_squared_sum / state->calibration_samples - magnitude_squared;
    if (variance > LEVEL_CAL_MAX_VARIANCE || magnitude_squared < 0.64f || magnitude_squared > 1.44f) {
        level_calibration_reset_window(state);
        return false;
    }

    level_calibration_alignment(mean, sqrtf(magnitude_squared));
    for (int i = 0; i < 3; ++i) {
        state->gravity[i] = mean[i];
    }
    state->calibrating = false;
    level_calibration_status(state, "ZERO SET");
    return true;
}

lv_obj_t *motion_level_create(lv_obj_t *parent, uint32_t theme_color, uint32_t ink_color)
{
    motion_level_t *state = lv_malloc_zeroed(sizeof(*state));
    LV_ASSERT_MALLOC(state);
    if (!state) {
        return NULL;
    }

    lv_obj_t *obj = lv_obj_create(parent);
    lv_obj_remove_style_all(obj);
    lv_obj_set_size(obj, LV_PCT(100), LV_PCT(100));
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_user_data(obj, state);
    state->theme_color = theme_color;
    state->ink_color = ink_color;
    state->normal_y = 1.0f;
    state->x_label = level_label(obj, ink_color, &lv_font_montserrat_12);
    state->y_label = level_label(obj, ink_color, &lv_font_montserrat_12);
    state->angle_label = level_label(obj, ink_color, &lv_font_montserrat_14);
    state->missing_label = level_label(obj, ink_color, &lv_font_montserrat_12);
    state->calibration_label = level_label(obj, ink_color, &lv_font_montserrat_10);
    lv_obj_add_flag(state->calibration_label, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(obj, level_draw_cb, LV_EVENT_DRAW_MAIN, state);
    lv_obj_add_event_cb(obj, level_delete_cb, LV_EVENT_DELETE, state);
    lv_obj_update_layout(obj);
    return obj;
}

static void level_update_angles(lv_obj_t *obj, motion_level_t *state, const int16_t acc[3])
{
    bool first = !state->ready;
    const uint32_t now = lv_tick_get();
    const uint32_t elapsed = LV_MIN(lv_tick_elaps(state->last_tick), 200);
    const float alpha = first ? 1.0f : elapsed / (elapsed + LEVEL_FILTER_MS);
    for (int i = 0; i < 3; ++i) {
        const float sample = acc[i] / 16384.0f;
        state->gravity[i] += alpha * (sample - state->gravity[i]);
    }
    state->last_tick = now;
    if (level_calibration_collect(state, acc)) {
        first = true;
    }

    /* Rotate the mounted sensor's screen-plane axes clockwise by 90 degrees. */
    const float mounted[3] = {-state->gravity[1], state->gravity[0], state->gravity[2]};
    float aligned[3] = {0.0f};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            aligned[row] += s_level_alignment[row][col] * mounted[col];
        }
    }
    const float x = aligned[0];
    const float y = aligned[1];
    const float z = aligned[2];
    const float xy = sqrtf(x * x + y * y);
    const float magnitude = sqrtf(x * x + y * y + z * z);
    if (magnitude < 0.1f) {
        state->ready = false;
        state->calibrating = false;
        return;
    }

    state->ready = true;
    state->x_deg = atan2f(-x, sqrtf(y * y + z * z)) * LEVEL_DEG_PER_RAD;
    state->y_deg = atan2f(-y, sqrtf(x * x + z * z)) * LEVEL_DEG_PER_RAD;

    /* Near flat, the projected gravity direction is undefined; keep its last value. */
    if (xy > 0.05f * magnitude) {
        state->normal_x = x / xy;
        state->normal_y = y / xy;
    }

    /* Mounted positive Z reveals sky, without changing X/Y motion or angle signs. */
    state->sky_mix = level_smoothstep(-0.25f, 0.25f, z / magnitude);
    state->horizon_pitch_deg = atan2f(z, xy) * LEVEL_DEG_PER_RAD;
    state->horizon_offscreen = level_horizon_offscreen(obj, state);
    const float face_degrees = atan2f(xy, fabsf(z)) * LEVEL_DEG_PER_RAD;
    const float horizon_target = state->horizon_offscreen ? 0.0f : 1.0f;
    const float bubble_green = state->horizon_offscreen ?
                               1.0f - level_smoothstep(LEVEL_GREEN_FULL_DEG, LEVEL_GREEN_START_DEG, face_degrees) : 0.0f;
    const float horizon_roll_deg = atan2f(fabsf(state->normal_x), fabsf(state->normal_y)) * LEVEL_DEG_PER_RAD;
    const float horizontal = 1.0f - level_smoothstep(LEVEL_GREEN_FULL_DEG, LEVEL_GREEN_START_DEG, horizon_roll_deg);
    const float centered = 1.0f - level_smoothstep(LEVEL_GREEN_FULL_DEG, LEVEL_GREEN_START_DEG,
                                                 fabsf(state->horizon_pitch_deg));
    /* A horizontal line is level only when it also passes near the center. */
    const float horizon_green = horizontal * centered;
    const float transition_alpha = elapsed / (elapsed + LEVEL_TRANSITION_MS);
    const float color_alpha = elapsed / (elapsed + LEVEL_COLOR_FILTER_MS);
    state->horizon_mix = first ? horizon_target : level_follow(state->horizon_mix, horizon_target, transition_alpha);
    /* Upward pitch reads positive; screen-space bubble motion keeps its original sign. */
    const float pitch_readout = -state->y_deg;
    if (first) {
        state->bubble_green = 0.0f;
        state->horizon_green = 0.0f;
        state->readout_deg[0] = state->x_deg;
        state->readout_deg[1] = pitch_readout;
        state->readout_deg[2] = state->horizon_pitch_deg;
    }
    else {
        const float readout_alpha = elapsed / (elapsed + LEVEL_READOUT_FILTER_MS);
        state->readout_deg[0] += readout_alpha * (state->x_deg - state->readout_deg[0]);
        state->readout_deg[1] += readout_alpha * (pitch_readout - state->readout_deg[1]);
        state->readout_deg[2] += readout_alpha * (state->horizon_pitch_deg - state->readout_deg[2]);
    }
    state->bubble_green = level_follow(state->bubble_green, bubble_green, color_alpha);
    state->horizon_green = level_follow(state->horizon_green, horizon_green, color_alpha);

    if (first || lv_tick_elaps(state->last_readout_tick) >= LEVEL_READOUT_PERIOD_MS) {
        lv_obj_t *labels[] = {state->x_label, state->y_label, state->angle_label};
        const char *prefixes[] = {"X\n", "Y\n", ""};
        for (int i = 0; i < 3; ++i) {
            const float tenths = state->readout_deg[i] * 10.0f;
            /* A 0.03-degree margin prevents flickering at rounding boundaries. */
            if (first || fabsf(tenths - state->readout_tenths[i]) >= 0.8f) {
                const int rounded = (int)lroundf(tenths);
                if (first || rounded != state->readout_tenths[i]) {
                    state->readout_tenths[i] = rounded;
                    level_label_degrees(labels[i], prefixes[i], rounded);
                }
            }
        }
        state->last_readout_tick = now;
    }
}

void motion_level_update(lv_obj_t *obj, const int16_t acc[3], bool present)
{
    motion_level_t *state = lv_obj_get_user_data(obj);
    if (present) {
        level_update_angles(obj, state, acc);
    }
    else {
        state->ready = false;
        state->calibrating = false;
    }

    int32_t cx, cy;
    level_center(obj, &cx, &cy);
    lv_obj_set_flag(state->missing_label, LV_OBJ_FLAG_HIDDEN, state->ready);
    if (!state->ready || (!state->calibrating &&
        lv_tick_elaps(state->calibration_status_tick) >= LEVEL_CAL_STATUS_MS)) {
        state->calibration_status_active = false;
    }
    lv_obj_set_flag(state->calibration_label, LV_OBJ_FLAG_HIDDEN, !state->calibration_status_active);
    lv_obj_set_size(state->calibration_label, lv_obj_get_width(obj) - 16, 12);
    lv_obj_set_pos(state->calibration_label, 8, lv_obj_get_height(obj) - 14);

    if (!state->ready) {
        lv_anim_delete(state, level_angle_slide_cb);
        state->angle_position_ready = false;
        level_label_visibility(state->x_label, 0.0f);
        level_label_visibility(state->y_label, 0.0f);
        level_label_visibility(state->angle_label, 0.0f);
        lv_label_set_text(state->missing_label, present ? "MPU6050\nNO DATA" : "MPU6050\nNOT CONNECTED");
        lv_obj_set_size(state->missing_label, lv_obj_get_width(obj) - 16, 38);
        lv_obj_set_pos(state->missing_label, 8, cy - 19);
    }
    else {
        const float bubble_text = level_bubble_visibility(state) * (1.0f - state->bubble_green);
        level_label_visibility(state->x_label, bubble_text);
        level_label_visibility(state->y_label, bubble_text);
        lv_obj_set_size(state->x_label, 42, 34);
        lv_obj_set_size(state->y_label, 42, 34);
        lv_obj_set_pos(state->x_label, 0, cy - 17);
        lv_obj_set_pos(state->y_label, lv_obj_get_width(obj) - 42, cy - 17);

        level_label_visibility(state->angle_label, level_horizon_visibility(state));
        lv_obj_set_size(state->angle_label, LEVEL_ANGLE_WIDTH, LEVEL_ANGLE_HEIGHT);
        level_angle_update_side(obj, state);
    }
    lv_obj_invalidate(obj);
}
