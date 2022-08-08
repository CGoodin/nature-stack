#ifndef ASTAR_CELL_H
#define ASTAR_CELL_H

/// represents a single pixel in A* grid
class AStarCell {
  public:
    int idx;     // index in the flattened grid
    float cost;  // cost of traversing this pixel

    /** Create a cell with index i and cost c
     * \param i The index of the cell
     * \param c The cost associated with the cell
     */
    AStarCell(int i, float c) : idx(i),cost(c) {}
};

// the top of the priority queue is the greatest element by default,
// but we want the smallest, so flip the sign
bool operator<(const AStarCell &n1, const AStarCell &n2) {
  return n1.cost > n2.cost;
}

bool operator==(const AStarCell &n1, const AStarCell &n2) {
  return n1.idx == n2.idx;
}

#endif

