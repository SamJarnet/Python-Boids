#include "boids.h"


void boid_random_init(Boid *boid, int width, int height)
{
    boid->position = (Vector2){rand() % width,rand() % height};
    boid->velocity = (Vector2){10.0f, 10.0f};
}

void boid_update(Boid *boid, Vector2 change, float dt)
{
    boid->velocity = vector_add(boid->velocity, change);

    boid->position = vector_add(boid->position,vector_multiply(boid->velocity, dt));
}


void check_boundaries(Boid *boid, int width, int height)
{
    if (boid->position.x < 0)
        boid->position.x = width;

    if (boid->position.x > width)
        boid->position.x = 0;

    if (boid->position.y < 0)
        boid->position.y = height;

    if (boid->position.y > height)
        boid->position.y = 0;
}

Vector2 separation(Boid *boids, int boid_count, int index){
    Vector2 vec = {0.0f, 0.0f};

    for (int i = 0; i<boid_count; i++){
        if (i == index) continue;

        Vector2 displacement = vector_subtract(boids[index].position, boids[i].position);
        float mod_r = vector_length(displacement);

        if (mod_r > 0.0f && mod_r < 40.0f)
            vec = vector_add(vec, vector_normalize(displacement));
    }
    return vector_multiply(vec, 0.5f);
}


Vector2 cohesion(Boid *boids, int index, Vector2 avg_pos){

    Vector2 difference = vector_subtract(avg_pos, boids[index].position);

    return vector_multiply(difference, 0.005f);
}

Vector2 alignment(Boid *boids, int index, Vector2 avg_vel){

    Vector2 difference = vector_subtract(avg_vel, boids[index].velocity);

    return vector_multiply(difference, 0.05f);

}

Vector2 find_average_pos(Boid *boids, int boid_count){
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    for (int i = 0; i<boid_count; i++){
        sum_x += boids[i].position.x;
        sum_y += boids[i].position.y;
    }
        
    float avg_x = sum_x / boid_count;
    float avg_y = sum_y / boid_count;

    Vector2 avg_pos = (Vector2){avg_x, avg_y};
    return avg_pos;
}

Vector2 find_average_vel(Boid *boids, int boid_count){
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    for (int i = 0; i<boid_count; i++){
        sum_x += boids[i].velocity.x;
        sum_y += boids[i].velocity.y;
    }
        
    float avg_x = sum_x / boid_count;
    float avg_y = sum_y / boid_count;

    Vector2 avg_vel = (Vector2){avg_x, avg_y};
    return avg_vel;
}