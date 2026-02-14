// Minimal Arduino wrapper: ensure types.h is included before prototype generation
#include <Arduino.h>
#include "types.h"

extern void app_setup();
extern void app_loop();

void setup() { app_setup(); }
void loop() { app_loop(); }
