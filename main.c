#include "astar.h"
#include <stdio.h>
#include <stdlib.h>

#define HEIGHT 2
#define WIDTH 2

int **generate_walls_map() {
  int i, j;
  int **walls = malloc(sizeof(int) * WIDTH * HEIGHT);

  for (i = 0; i < HEIGHT; i++) {
    walls[i] = malloc(sizeof(int) * WIDTH);

    for (j = 0; j < WIDTH; j++) {
      walls[i][j] = 0;
    }
  }

  // let's add some walls

  return walls;
}

int main(int argc, char const *argv[]) {
  AStarConfig config = {NULL, WIDTH, HEIGHT, {0, 0}, {1, 1}};

  config.walls_map = generate_walls_map();
  AStarSet path;

  path = astar(&config);

  if (path == NULL) {
    printf("L'algoritmo ha fallito\n");
  } else {
    as_set_print(path);
  }

  return 0;
}
