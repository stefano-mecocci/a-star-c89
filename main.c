#include "astar.h"
#include <stdio.h>
#include <stdlib.h>

#define HEIGHT 5
#define WIDTH 5

#define START { 3, 0 }
#define GOAL { 0, 3 }
#define WALLS {0, 2}, {1, 2}, {1, 3}

void set_walls_map(int **map) {
  int i, j, k, l;
  int walls[3][2] = {WALLS};

  for (i = 0; i < 3; i++) {
    k = walls[i][1];
    l = walls[i][0];

    map[k][l] = 1;
  }
}

int **create_walls_map() {
  int i, j;
  int **walls = malloc(sizeof(int) * WIDTH * HEIGHT);

  for (i = 0; i < HEIGHT; i++) {
    walls[i] = malloc(sizeof(int) * WIDTH);

    for (j = 0; j < WIDTH; j++) {
      walls[i][j] = 0;
    }
  }

  return walls;
}

void print_map(int **map) {
  int i, j;

  for (i = 0; i < HEIGHT; i++) {
    for (j = 0; j < WIDTH; j++) {
      if (map[i][j] == 0) {
        printf(". ");
      } else if (map[i][j] == 1) {
        printf("X ");
      } else {
        printf("@ ");
      }
    }
    printf("\n");
  }
  printf("\n");
}

void print_path(int **map, AStarSet path) {
  int i, j;

  while (path != NULL) {
    j = path->point.x;
    i = path->point.y;
    map[i][j] = 2;

    path = path->next;
  }

  print_map(map);
}

int main(int argc, char const *argv[]) {
  AStarConfig config = {NULL, WIDTH, HEIGHT, START, GOAL};

  config.walls_map = create_walls_map();
  set_walls_map(config.walls_map);
  AStarSet path;

  path = astar(&config);

  if (path == NULL) {
    printf("The algorithm failed\n");
  } else {
    print_path(config.walls_map, path);
  }

  return 0;
}
