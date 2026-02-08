package PathFinderVisualizer;

import java.util.List;

/**
 * A GridGraph is a graph where vertices can be arranged in a grid
 * and no pair of edges will cross one another (it is planar).
 *
 * We can use a GridGraph to represent a maze either by representing
 * the corridors of the maze or the walls of the maze.  The
 * GridGraph representing the walls of the maze should be one row
 * taller and one row wider than the GridGraph representing the
 * corridors of the maze.
 *
 * Given a grid with height h and width w, there are w*h vertices.
 * A vertex may be connected with edges to the vertices to the
 * north, south, east and/or west (assuming such vertices exist).
 * Edges are undirected, so if vertex 0 is adjacent to vertex 1,
 * the graph should also report that vertex 1 is adjacent to vertex 0.
 */
public interface GridGraph {

    /**
     @return the height of the grid
     */
    public int getHeight();

    /**
     * @return the width of the grid
     */
    public int getWidth();

    /**
     * @param vertex a vertex in the graph
     * @return a non-null list of adjacent verticies,
     * if there are no adjacent vertices, the list should have 0 size,
     * this method should NOT return null
     */
    public List<Integer> adjacent(int vertex);

    /**
     * @return the number of vertices in this graph i.e., (height*width)
     */
    public int nVertices();
}
