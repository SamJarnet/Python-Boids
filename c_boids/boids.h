#ifndef BOID_H
#define BOID_H
#define MAX_BOIDS 100
#include <stdio.h>
#include <stdlib.h>
#include "vector.h"

typedef struct {
    int members[MAX_BOIDS];
    int count;
} Group;

typedef struct
{
    Vector2 position;
    Vector2 velocity;
} Boid;

void boid_random_init(Boid *boid, int width, int height);
void boid_update(Boid *boid, Vector2 change, float dt);
void check_boundaries(Boid *boid, int width, int height);

Vector2 separation(Boid *boids, int boid_count, int index);
Vector2 cohesion(Boid *boids, int index, Vector2 avg_pos);
Vector2 alignment(Boid *boids, int index, Vector2 avg_pos);

Vector2 find_average_pos(Boid *boids, int boid_count);
Vector2 find_average_vel(Boid *boids, int boid_count);

#endif