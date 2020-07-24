#pragma once
#include <pebble.h>
void main_layer_update_proc(Layer *layer, GContext *ctx);
void main_window_load(Window *window);
void main_window_unload(Window *window);

void main_window_init();
void main_window_deinit();

void main_window_update();

Window* get_main_window();