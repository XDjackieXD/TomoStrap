#include <pebble.h>
#include "main_window.h"
#include "common.h"

static Window *s_main_window = NULL;
static Layer *s_main_layer = NULL;

static int *main_window_pulse;
static int *main_window_oxygen;

static char text_buffer[10];

Window* get_main_window() {
  return s_main_window;
}

void main_layer_update_proc(Layer *layer, GContext *ctx) {
  GRect bounds = layer_get_bounds(layer);

  GRect pulse_bg_bounds;
  pulse_bg_bounds.origin.x = bounds.origin.x + 12;
  pulse_bg_bounds.origin.y = bounds.origin.y + 12;
  pulse_bg_bounds.size.w = bounds.size.w - 12 - 12;
  pulse_bg_bounds.size.h = bounds.size.h/2 - 12 - 6;
  graphics_context_set_fill_color(ctx, PBL_IF_COLOR_ELSE(COLOR_PULSE_BG_COLOR, COLOR_PULSE_BG_BW));
  graphics_fill_rect(ctx, pulse_bg_bounds, 4, GCornersAll);
  
  GRect oxigen_bg_bounds;
  oxigen_bg_bounds.origin.x = bounds.origin.x + 12;
  oxigen_bg_bounds.origin.y = bounds.origin.y + (bounds.size.h/2) + 6;
  oxigen_bg_bounds.size.w = bounds.size.w - 12 - 12;
  oxigen_bg_bounds.size.h = (bounds.size.h/2) - 12 - 6;
  graphics_context_set_fill_color(ctx, PBL_IF_COLOR_ELSE(COLOR_OXIGEN_BG_COLOR, COLOR_OXIGEN_BG_BW));
  graphics_fill_rect(ctx, oxigen_bg_bounds, 4, GCornersAll);

  GRect text_bounds;
  GFont id_font = fonts_get_system_font(FONT_KEY_GOTHIC_28_BOLD);
  
  text_bounds.origin.x = pulse_bg_bounds.origin.x + 6;
  text_bounds.origin.y = pulse_bg_bounds.origin.y + 6;
  text_bounds.size.w = pulse_bg_bounds.size.w - 12;
  text_bounds.size.h = pulse_bg_bounds.size.h - 24;
  graphics_context_set_text_color(ctx, PBL_IF_COLOR_ELSE(COLOR_PULSE_TEXT_COLOR, COLOR_PULSE_TEXT_BW));
  if((*main_window_pulse) >= 0){
    snprintf(text_buffer, 10, "%d.%1d BPM", (int)((*main_window_pulse)/10.0), (*main_window_pulse)%10);
    graphics_draw_text(ctx, text_buffer, id_font, text_bounds, GTextOverflowModeFill, GTextAlignmentCenter, NULL);
  } else {
    graphics_draw_text(ctx, "--", id_font, text_bounds, GTextOverflowModeFill, GTextAlignmentCenter, NULL);
  }
  text_bounds.origin.y = pulse_bg_bounds.origin.y + 6 + pulse_bg_bounds.size.h - 32;
  text_bounds.size.h = pulse_bg_bounds.origin.y + pulse_bg_bounds.size.h - text_bounds.origin.y - 6;
  graphics_draw_text(ctx, "Pulse", fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD), text_bounds, GTextOverflowModeFill, GTextAlignmentCenter, NULL);
  text_bounds.origin.y += 48;

  text_bounds.origin.x = oxigen_bg_bounds.origin.x + 6;
  text_bounds.origin.y = oxigen_bg_bounds.origin.y + 6;
  text_bounds.size.w = oxigen_bg_bounds.size.w - 12;
  text_bounds.size.h = oxigen_bg_bounds.size.h - 24;
  graphics_context_set_text_color(ctx, PBL_IF_COLOR_ELSE(COLOR_OXIGEN_TEXT_COLOR, COLOR_OXIGEN_TEXT_BW));
  if((*main_window_oxygen) >= 0) {
    snprintf(text_buffer, 10, "%d.%1d %%", (int)((*main_window_oxygen)/10.0), (*main_window_oxygen)%10);
    graphics_draw_text(ctx, text_buffer, id_font, text_bounds, GTextOverflowModeFill, GTextAlignmentCenter, NULL);
  } else {
    graphics_draw_text(ctx, "--", id_font, text_bounds, GTextOverflowModeFill, GTextAlignmentCenter, NULL);
  }
  text_bounds.origin.y = oxigen_bg_bounds.origin.y + 6 + oxigen_bg_bounds.size.h - 32;
  text_bounds.size.h = oxigen_bg_bounds.origin.y + oxigen_bg_bounds.size.h - text_bounds.origin.y - 6;
  graphics_draw_text(ctx, "SpO2", fonts_get_system_font(FONT_KEY_GOTHIC_18_BOLD), text_bounds, GTextOverflowModeFill, GTextAlignmentCenter, NULL);
  text_bounds.origin.y += 48;
}

void main_window_load(Window *window) {
  Layer *main_window_layer = window_get_root_layer(window);
  GRect bounds = layer_get_bounds(main_window_layer);

  s_main_layer = layer_create(bounds);
  layer_set_update_proc(s_main_layer, main_layer_update_proc);
  layer_add_child(main_window_layer, s_main_layer);
}

void main_window_update() {
  layer_mark_dirty(s_main_layer);
}

void main_window_unload(Window *window) {
  layer_destroy(s_main_layer);
}

void main_window_init(int *pulse, int *oxygen) {
  main_window_pulse = pulse;
  main_window_oxygen = oxygen;
  s_main_window = window_create();
  window_set_background_color(s_main_window, PBL_IF_COLOR_ELSE(COLOR_BG_COLOR, COLOR_BG_BW));
  window_set_window_handlers(s_main_window, (WindowHandlers) {
    .load = main_window_load,
    .unload = main_window_unload,
  });
  window_stack_push(s_main_window, true);
}

void main_window_deinit() {
  window_destroy(s_main_window);
}