#include <SDL2/SDL.h>
#include <lvgl.h>
#include <cmath>

#ifdef _WIN32
#include <windows.h>
#endif

#include "dashboard_layout.h"
#include "motor_metrics.h"

namespace {
constexpr int DISPLAY_WIDTH = 800;
constexpr int DISPLAY_HEIGHT = 480;
constexpr int BUFFER_LINES = 40;

SDL_Window *window = nullptr;
SDL_Renderer *renderer = nullptr;
SDL_Texture *texture = nullptr;
uint8_t draw_buffer[DISPLAY_WIDTH * BUFFER_LINES * 2];
lv_point_t mouse_position{0, 0};
bool mouse_pressed = false;

void read_mouse(lv_indev_t *, lv_indev_data_t *data) {
    data->point = mouse_position;
    data->state = mouse_pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

void flush_display(lv_display_t *display, const lv_area_t *area, uint8_t *pixel_map) {
    const int width = lv_area_get_width(area);
    const int height = lv_area_get_height(area);

    SDL_Rect destination{area->x1, area->y1, width, height};
    SDL_UpdateTexture(texture, &destination, pixel_map, width * 2);
    SDL_RenderClear(renderer);
    SDL_RenderCopy(renderer, texture, nullptr, nullptr);
    SDL_RenderPresent(renderer);
    lv_display_flush_ready(display);
}

// Generates fake but plausible motor telemetry so the simulator UI has something to show.
MotorMetrics simulate_metrics(uint32_t elapsedMs) {
    MotorMetrics metrics;
    float t = elapsedMs / 1000.0f;

    metrics.speedKmh = 15.0f + 10.0f * sinf(t * 0.3f);
    metrics.cadenceRpm = static_cast<uint16_t>(70 + 20 * sinf(t * 0.4f));
    metrics.powerWatts = static_cast<uint16_t>(150 + 100 * sinf(t * 0.25f));
    metrics.assistLevel = static_cast<uint8_t>((static_cast<int>(t / 5) % 5));
    metrics.motorTempC = static_cast<int8_t>(35 + 5 * sinf(t * 0.05f));
    metrics.batteryVoltage = 48.0f + 1.5f * sinf(t * 0.1f);
    metrics.batteryCurrent = 3.0f + 2.0f * sinf(t * 0.25f);
    metrics.batterySocPercent = static_cast<uint8_t>(80 - static_cast<int>(t / 10) % 30);
    metrics.errorCode = 0;
    metrics.canActive = true;
    return metrics;
}
} // namespace

int main(int, char **) {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        return 1;
    }

    window = SDL_CreateWindow(
        "BafangBLE Emulator",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        DISPLAY_WIDTH,
        DISPLAY_HEIGHT,
        SDL_WINDOW_SHOWN);
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    texture = SDL_CreateTexture(
        renderer,
        SDL_PIXELFORMAT_RGB565,
        SDL_TEXTUREACCESS_STREAMING,
        DISPLAY_WIDTH,
        DISPLAY_HEIGHT);

    if (window == nullptr || renderer == nullptr || texture == nullptr) {
        SDL_Quit();
        return 1;
    }

    lv_init();
    lv_display_t *display = lv_display_create(DISPLAY_WIDTH, DISPLAY_HEIGHT);
    lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);
    lv_display_set_buffers(
        display,
        draw_buffer,
        nullptr,
        sizeof(draw_buffer),
        LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(display, flush_display);

    lv_indev_t *mouse = lv_indev_create();
    lv_indev_set_type(mouse, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(mouse, read_mouse);

    create_dashboard();

    bool running = true;
    uint32_t last_tick = SDL_GetTicks();
    uint32_t last_update = last_tick;
    uint32_t start_tick = last_tick;

    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event) != 0) {
            if (event.type == SDL_QUIT) {
                running = false;
            } else if (event.type == SDL_MOUSEMOTION) {
                mouse_position.x = event.motion.x;
                mouse_position.y = event.motion.y;
            } else if (event.type == SDL_MOUSEBUTTONDOWN && event.button.button == SDL_BUTTON_LEFT) {
                mouse_position.x = event.button.x;
                mouse_position.y = event.button.y;
                mouse_pressed = true;
            } else if (event.type == SDL_MOUSEBUTTONUP && event.button.button == SDL_BUTTON_LEFT) {
                mouse_position.x = event.button.x;
                mouse_position.y = event.button.y;
                mouse_pressed = false;
            }
        }

        uint32_t now = SDL_GetTicks();
        lv_tick_inc(now - last_tick);
        last_tick = now;

        if (now - last_update >= 100) {
            update_dashboard(simulate_metrics(now - start_tick));
            last_update = now;
        }

        lv_timer_handler();
        SDL_Delay(5);
    }

    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}

#ifdef _WIN32
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {
    return main(__argc, __argv);
}
#endif
