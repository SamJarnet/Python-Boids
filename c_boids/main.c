#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <stdio.h>
#include <stdlib.h>
#include "boids.h"



int main(int argc, char* argv[]) {

    SDL_Window *window;                    // Declare a pointer

    int width = 640;
    int height = 480;

    SDL_Init(SDL_INIT_VIDEO);              // Initialize SDL3

    // Create an application window with the following settings:
    window = SDL_CreateWindow(
        "An SDL3 window",                  // window title
        640,                               // width, in pixels
        480,                               // height, in pixels
        SDL_WINDOW_OPENGL                  // flags - see below
    );

    // Check that the window was successfully created
    if (window == NULL) {
        // In the case that the window could not be made...
        SDL_LogError(SDL_LOG_CATEGORY_ERROR, "Could not create window: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Renderer *renderer = SDL_CreateRenderer(window, NULL);

    if (renderer == NULL)
    {
        SDL_Log("Renderer creation failed: %s", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }



    bool done = false;
    float speed = 5.0f;

    int boid_count = 10;

    

    Boid boid_arr[boid_count];

    for(int i = 0; i<boid_count; i++){
        boid_random_init(&boid_arr[i], width, height);
    }

    Uint64 last = SDL_GetPerformanceCounter();
    while (!done) {
        SDL_Event event;

        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_EVENT_QUIT) {
                done = true;
            }
        }

        // Do game logic, present a frame, etc.

        Uint64 now = SDL_GetPerformanceCounter();
        float dt = 1 * (float)(now - last) / SDL_GetPerformanceFrequency();
        last = now;
        float max_speed = 150.0f; // Maximum pixels per second
        // Group *groups = find_groups(boid_arr, boid_count);


        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

        for(int i = 0; i<boid_count; i++){
            
            check_boundaries(&boid_arr[i], width, height);

            Vector2 avg_pos = find_average_pos(boid_arr, boid_count);
            Vector2 avg_vel = find_average_vel(boid_arr, boid_count);

            Vector2 separation_force = separation(boid_arr, boid_count, i);            
            Vector2 cohesion_force = cohesion(boid_arr, i, avg_pos);
            Vector2 alignment_force = alignment(boid_arr, i, avg_vel);

            Vector2 total_force = vector_add(vector_add(separation_force, cohesion_force), alignment_force);

            
            boid_update(&boid_arr[i], total_force, dt);
            float current_speed = vector_length(boid_arr[i].velocity);
            if (current_speed > max_speed) {
                boid_arr[i].velocity = vector_multiply(vector_normalize(boid_arr[i].velocity), max_speed);
}
            

            SDL_RenderPoint(renderer, (int)boid_arr[i].position.x, (int)boid_arr[i].position.y);
        }


        SDL_RenderPresent(renderer);
    }

    // Close and destroy the window
    SDL_DestroyWindow(window);

    // Clean up
    SDL_Quit();
    return 0;


}
