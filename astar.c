#include "astar.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

int as_set_is_empty(AStarSet l);
AStarSet as_set_add(AStarSet l, AStarPoint *p);
AStarPoint as_set_find_lowest_fscore(AStarSet l);
int as_set_contains(AStarSet l, AStarPoint *p);
AStarSet as_set_remove(AStarSet l, AStarPoint *p);
void as_set_free(AStarSet l);

AStarGraph as_graph_init(AStarPoint *point);
AStarGraph as_graph_add_edge(AStarGraph p, AStarVertex *q, AStarVertex *r);
void as_graph_free(AStarGraph g);

int as_point_equal(AStarPoint *p, AStarPoint *q);
AStarPoint as_point_from(int p[]);
double heuristic(AStarPoint *p1, AStarPoint *p2);
int is_valid_point(int x, int y, AStarConfig *c);
AStarPoint *create_point_neighbors();
void set_point_neighbors(AStarPoint *neighbors, AStarPoint *p,
                         AStarConfig *config);
void astar_free(AStarSet open, AStarSet closed, AStarGraph came_from,
                AStarPoint *neighbors);
AStarSet reconstruct_path(AStarGraph p);

/* ======================= SET ============================= */

int as_set_is_empty(AStarSet l) { return l == NULL; }

AStarSet as_set_add(AStarSet l, AStarPoint *p) {
  AStarNode *new = malloc(sizeof(AStarNode));

  new->point = *p;
  new->next = l;

  return new;
}

AStarPoint as_set_find_lowest_fscore(AStarSet l) {
  double f = 9999; /*  random big num */
  AStarNode *p = l;
  AStarPoint r;

  while (p != NULL) {
    if (p->point.f_score < f) {
      f = p->point.f_score;
      r = p->point;
    }

    p = p->next;
  }

  return r;
}

int as_set_contains(AStarSet l, AStarPoint *p) {
  int found = FALSE;

  while (!found && l != NULL) {
    if (as_point_equal(p, &l->point)) {
      found = TRUE;
    }

    l = l->next;
  }

  return found;
}

AStarSet as_set_remove(AStarSet l, AStarPoint *p) {
  AStarNode *q = NULL;
  AStarNode *r = l;
  int found = FALSE;

  while (!found && r != NULL) {
    if (as_point_equal(p, &r->point)) {
      found = TRUE;

      if (q == NULL) {
        l = r->next;
      } else {
        q->next = r->next;
      }
    } else {
      q = r;
      r = r->next;
    }
  }

  return l;
}

void as_point_print(AStarPoint p) {
  printf("[(%d, %d), ", p.x, p.y);
  printf("f: %f, ", p.f_score);
  printf("g: %f]\n", p.g_score);
}

void as_set_print(AStarSet l) {
  while (l != NULL) {
    as_point_print(l->point);
    l = l->next;
  }

  printf("(end of list)\n\n");
}

void as_set_free(AStarSet l) {
  if (l != NULL) {
    as_set_free(l->next);
    free(l);
  }
}

/* ======================= GRAPH ============================= */

AStarGraph as_graph_init(AStarPoint *point) {
  AStarVertex *t = malloc(sizeof(AStarVertex));
  int i;

  t->point = *point;
  for (i = 0; i < DIRECTIONS; i++) {
    t->children[i] = NULL;
  }
  t->parent = NULL;

  return t;
}

AStarGraph as_graph_add_edge(AStarGraph p, AStarVertex *q, AStarVertex *r) {
  int i, set = FALSE;

  if (p == NULL) {
    return p;
  }

  if (as_point_equal(&p->point, &q->point)) {
    for (i = 0; !set && i < DIRECTIONS; i++) {
      if (p->children[i] == NULL) {
        p->children[i] = r;
        r->parent = p;
        set = TRUE;
      }
    }
  } else {
    for (i = 0; i < DIRECTIONS; i++) {
      if (p->children[i] != NULL) {
        as_graph_add_edge(p->children[i], q, r);
      }
    }
  }

  return p;
}

void as_graph_free(AStarGraph g) {
  int i;

  if (g != NULL) {
    for (i = 0; i < DIRECTIONS; i++) {
      as_graph_free(g->children[i]);
    }

    free(g);
  }
}

/* ======================= ALGORITHM ============================= */

int as_point_equal(AStarPoint *p, AStarPoint *q) {
  return (p->x == q->x) && (p->y == q->y);
}

AStarPoint as_point_from(int p[]) {
  AStarPoint tmp = {0, 0, 0, 0};
  tmp.x = p[0];
  tmp.y = p[1];

  return tmp;
}

double heuristic(AStarPoint *p1, AStarPoint *p2) {
  double first = pow(p1->x - p2->x, 2);
  double second = pow(p1->y - p2->y, 2);

  return sqrt(first + second);
}

int is_valid_point(int x, int y, AStarConfig *c) {
  int r = x >= 0 && x < c->width;
  r = r && (y >= 0 && y < c->height);
  r = r && c->walls_map[y][x] == 0; /* not a wall */

  return r;
}

AStarPoint *create_point_neighbors() {
  AStarPoint *neighbors = malloc(sizeof(AStarPoint) * DIRECTIONS);
  int i;

  for (i = 0; i < DIRECTIONS; i++) {
    neighbors[i].x = -1;
    neighbors[i].y = -1;
  }

  return neighbors;
}

void set_point_neighbors(AStarPoint *neighbors, AStarPoint *p,
                         AStarConfig *config) {
  int w[DIRECTIONS][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  int i, x, y;

  for (i = 0; i < DIRECTIONS; i++) {
    x = p->x + w[i][0];
    y = p->y + w[i][1];

    if (is_valid_point(x, y, config)) {
      neighbors[i].x = x;
      neighbors[i].y = y;
    }
  }
}

AStarSet reconstruct_path(AStarGraph p) {
  AStarSet r = NULL;

  while (p != NULL) {
    r = as_set_add(r, &p->point);
    p = p->parent;
  }

  return r;
}

AStarSet astar(AStarConfig *config) {
  /* preparing variables */
  AStarSet open = NULL;
  AStarSet closed = NULL;
  AStarPoint start = as_point_from(config->start);
  AStarPoint goal = as_point_from(config->goal);
  AStarGraph came_from;

  AStarPoint x;
  AStarSet result = NULL;
  AStarGraph last;
  AStarPoint *neighbors = create_point_neighbors();
  int failed = TRUE;
  int i, tentative_g_score, tentative_is_better;

  start.f_score = heuristic(&start, &goal);
  start.g_score = 0;
  came_from = as_graph_init(&start);
  open = as_set_add(open, &start);

  /* body of the algorithm */

  while (!as_set_is_empty(open)) {
    x = as_set_find_lowest_fscore(open);

    if (as_point_equal(&x, &goal)) {
      failed = FALSE;
      as_set_free(open);
      as_set_free(closed);
      free(neighbors);
      result = reconstruct_path(last);
      as_graph_free(came_from);

      return result;
    }

    open = as_set_remove(open, &x);
    closed = as_set_add(closed, &x);

    neighbors = create_point_neighbors(&x);
    set_point_neighbors(neighbors, &x, config);

    for (i = 0; i < DIRECTIONS; i++) {
      if (neighbors[i].x == -1 && neighbors[i].y == -1)
        continue;
      if (as_set_contains(closed, &neighbors[i]))
        continue;

      tentative_g_score = x.g_score + 1;

      if (!as_set_contains(open, &neighbors[i])) {
        open = as_set_add(open, &neighbors[i]);
        tentative_is_better = TRUE;
      } else if (tentative_g_score < neighbors[i].g_score) {
        tentative_is_better = TRUE;
      } else {
        tentative_is_better = FALSE;
      }

      if (tentative_is_better) {
        neighbors[i].g_score = tentative_g_score;
        neighbors[i].f_score =
            neighbors[i].g_score + heuristic(&neighbors[i], &goal);

        last = as_graph_init(&neighbors[i]);
        came_from = as_graph_add_edge(came_from, as_graph_init(&x), last);
      }
    }
  }

  if (failed) {
    as_set_free(open);
    as_set_free(closed);
    free(neighbors);
    as_graph_free(came_from);
    return NULL;
  }
}
