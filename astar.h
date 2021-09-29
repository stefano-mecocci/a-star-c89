#ifndef _ASTAR_H
#define _ASTAR_H

/*
==============================================================================
Constants
==============================================================================
*/

#define TRUE 1
#define FALSE 0

#define DIRECTIONS 4

/*
==============================================================================
Data Structures
==============================================================================
*/

typedef struct astarconfig {
  int **walls_map;
  int width;
  int height;
  int start[2];
  int goal[2];
} AStarConfig;

typedef struct astarpoint {
  int x;
  int y;
  double f_score;
  double g_score;
} AStarPoint;

typedef struct astarnode {
  AStarPoint point;
  struct astarnode *next;
} AStarNode;

typedef AStarNode * AStarSet;

typedef struct astarvertex {
  AStarPoint point;
  struct astarvertex *children[DIRECTIONS];
  struct astarvertex *parent;
} AStarVertex;

typedef AStarVertex * AStarGraph;

/*
==============================================================================
Functions
==============================================================================
*/

void as_point_print(AStarPoint p);

void as_set_print(AStarSet l);

AStarSet astar(AStarConfig *config);

#endif
