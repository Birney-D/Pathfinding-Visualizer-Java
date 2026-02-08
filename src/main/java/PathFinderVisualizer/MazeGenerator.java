package PathFinderVisualizer;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.*;
import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * MazeGenerator
 *
 *
 * Given a graph in which vertices are arranged in
 * a grid, we can describe a maze in the following fashion:
 *
 *  (1) create a graph of the grid with a given height and width
 *  (2) build a Spanning Tree (ST) on this graph representing corridors in the maze
 *  (3) select vertices on the ST for the maze start and end points
 *       !! for this project the start point must be on the left side of the ST graph
 *          and the end point must be on the right side of the ST graph !!
 *
 *  We can then use the ST to build a corresponding graph that represents
 *  the walls of the maze.  This graph is the dual of the ST we just built.
 *  The 'maze walls' graph has one more row and one more column than the 'corridors' graph
 *  If we lay out the 'maze walls' graph in a grid, the 'corridors' graph vertices can be
 *  placed in-between rows and columns of the 'maze walls' graph.  See the handout
 *  for details.  To do this we:
 *
 *  (1) begin with a graph whose height and width are one larger than the ST graph
 *  (2) remove edges in the wall graph that cross an edge in the corridors graph
 *  (3) remove an edge on the left side to allow an "entrance"
 *  (4) remove an edge on the right side to allow an "exit"
 *
 *  In this project, you'll build both the spanning tree for a maze and the corresponding
 *  graph to represent the walls of the maze.
 */
public class MazeGenerator {

    /**
     * Part 1: Implement this method.
     *
     * Given a height and width, generate a graph in which vertices
     * are arranged in a rectangular grid in row-major order (vertices
     * labeled 0 ... (height*width-1) starting at the upper left
     * corner and moving left to right and top to bottom).
     *
     * Return a spanning tree on this graph which can be thought
     * of as the corridors of a maze.
     *
     * IMPORTANT: The graph/spanning tree that is returned MUST be sure
     * to define vertex on the left edge of the graph as a
     * starting location, and a vertex on the right edge of
     * the graph as an end location.
     *
     * @param width   the width of the maze
     * @param height  the height of the maze
     * @return        a spanning tree over the grid
     */
    public static LabeledGridGraph generateST(int width, int height) {
        // Uses Wilson's Algorithm to generate a uniform spanning tree (corridor)
        // create a fully connected graph to run our search on and our empty st
        Corridor c = new Corridor(width, height);
        Corridor grid = new Corridor(width, height);
        grid.connectTheDots( ); //adds all edges
        int numVerts = c.nVertices( );

        // keeps track of vertices in our ST
        boolean[] inTree = new boolean[numVerts];
        inTree[c.getEnd( )] = true; // add end point to ST
        Random rando = new Random( );

        // Main loop keeps adding edges until all vertices are reached
        while (c.getNumEdges( ) < numVerts - 1) {
            // picks random start point (not in tree)
            int curVert = rando.nextInt(numVerts);
            while (inTree[curVert]) {
                curVert = rando.nextInt(numVerts);
            }

            // keeps track of a path that is our "random walk";
            int[] prev = new int[numVerts];
            Arrays.fill(prev, -1); // lets us know which verts have been visited
            prev[curVert] = -2; // marks our start point.

            // Keeps randomly walking until it connects with vertex already in tree
            while (!inTree[curVert]) {
                // walks to random adj vertices
                // If a loop appears, it is erased and it continues
                List<Integer> adjacentVs = grid.adjacent(curVert);
                int nxtVert = adjacentVs.get(rando.nextInt(adjacentVs.size( )));
                if (prev[nxtVert] == -1) { // not visited yet
                    prev[nxtVert] = curVert;

                    // It ran into itself, erase loop!
                } else eraseLoop(curVert, nxtVert, prev);

                curVert = nxtVert; // walk to next vertex
            }
            // Walk has connected to a part of the ST, let's add it
            addWalkTo(c, prev, inTree, curVert);
        }
        return c;
    }

    private static void eraseLoop(int strt, int stop, int[] prev) {
        // walks backwards over path and
        // erases (resets values) the loop.
        int idx = strt;
        int currV = prev[strt];
        int endOfLoop = prev[stop];

        while (currV != endOfLoop) {
            prev[idx] = -1;
            idx = currV;
            currV = prev[currV];
        }
    }

    private static void addWalkTo(Corridor c, int[] prev, boolean[] inTree, int curV) {
        // walks back through walk/path and adds edges to our ST
        while (prev[curV] != -2) {
            int v = prev[curV];
            int u = curV;
            c.addEdge(v, u);

            inTree[v] = true;
            inTree[u] = true;
            // increment down path
            curV = prev[curV];
        }
    }

    public static LabeledGridGraph depthFirstST(int width, int height) {
        // Uses Depth First Search to create ST
        Corridor c = new Corridor(width, height);
        Corridor grid = new Corridor(width, height);
        grid.connectTheDots( ); //adds all edges

        // Essentially our spanning tree
        int[] prev = new int[c.nVertices( )];
        Arrays.fill(prev, -1);
        int s = grid.getStart( ); // starting point
        prev[s] = -2;

        spanningDFS(grid, s, prev);
        addEdgesTo(c, prev);
        // return the ST.
        return c;
    }

    private static void spanningDFS(LabeledGridGraph g, int s, int[] prev) {
        // uses depth first search to visit all vertices and record the path
        for (Integer vertex : g.adjacent(s)) {
            if (prev[vertex] == -1) { // hasn't been visited
                prev[vertex] = s;
                spanningDFS(g, vertex, prev);
            }
        }
    }

    private static void addEdgesTo(Corridor c, int[] prev) {
        // walks over prev and adds edges, creating the ST
        for (int i = 0; i < prev.length; i++) {
            if (prev[i] == -2) continue; // the starting vertex (no prev value)
            c.addEdge(prev[i], i);
        }
    }

    public static LabeledGridGraph breadthFirstST(int width, int height) {
        // Uses Breadth First Search to create a ST
        Corridor c = new Corridor(width, height);
        Corridor g = new Corridor(width, height);
        g.connectTheDots( );

        Queue<Integer> Q = new ConcurrentLinkedQueue<>( );
        int[] prev = new int[g.nVertices( )];
        Arrays.fill(prev, -1); // marks where we've been (prevents loops)

        int currVert = g.getStart( );
        prev[currVert] = -2; // marks the beginning;
        Q.add(currVert);

        while (!Q.isEmpty( )) {
            currVert = Q.remove( );
            for (Integer adj : g.adjacent(currVert)) {
                // If not visited, set prev to cur and enqueue
                if (prev[adj] == -1) {
                    prev[adj] = currVert;
                    Q.add(adj);
                }
            }
        }
        // Q is now empty: lets add the edges
        addEdgesTo(c, prev);
        return c;
    }

    /**
     * Part 2: Implement this method
     *
     *  Given a GridGraph representing the corridors of a maze,
     *  build the corresponding graph representing the walls of the
     *  maze.
     *
     *  If the input graph is defined on a grid of size
     *  height x width, the output graph is defined on a grid
     *  of size (height + 1) x (width + 1).  That is, the
     *  graph of 'walls' has one more row and one more column
     *  than the graph of 'corridors'. See handout for details.
     *
     *  Note further, that we assume the input GridGraph has
     *  defined start and end vertices which can be accessed through
     *  the GridGraph's getStartVertex() and getEndVertex() methods,
     *  and further that the start vertex is a vertex on the spanning
     *  tree's left edge, and the end vertex is a vertex on the
     *  spanning tree's right edge.
     *
     *  The graph representing the maze should have all exterior
     *  walls intact except for one along the left edge (allowing
     *  an entrance) and one along the right edge (allowing an
     *  exit). For the maze (graph of edges)
     *  there is no starting or ending vertex, so the instance
     *  of GridGraph returned by this method should return -1
     *  when either the getStartVertex() or getEndVertex() methods
     *  are called.
     *
     * @param st
     * @return
     */
    public static GridGraph generateMaze(LabeledGridGraph st) {
        Walls maze = new Walls(st.getWidth( )+1, st.getHeight( )+1);
        maze.connectTheDots( );
        int stWidth = st.getWidth( );
        int mzWidth = maze.getWidth( );

        // locate coordinates of all edges and delete any edges that cross
        // First get row and col of corridor edges
        for (int i = 0; i < st.nVertices( ); i++) {
            int[] vertA = getCoordinates(i, stWidth);

            for (Integer adj : st.adjacent(i)) {
                // Translate these coordinates to edges to delete in maze graph
                int[] vertB = getCoordinates(adj, stWidth);
                int v = findVertAt(vertB[0], vertB[1], mzWidth);

                if (isVerticalEdge(vertA, vertB)) {
                    // The edge to delete will be horizontal (col + 1)
                    int u = findVertAt(vertB[0], vertB[1] + 1, mzWidth);
                    maze.removeEdge(v, u);

                } else if (isHorizontalEdge(vertA, vertB)) {
                    // The edge to delete will be vertical (row + 1)
                    int u = findVertAt(vertB[0] + 1, vertB[1], mzWidth);
                    maze.removeEdge(v, u);
                }
            }
        }
        // need to remove edges for entrance and exit
        createEntranceNExit((Corridor) st, maze);
        return maze;
    }

    private static void createEntranceNExit(Corridor st, Walls maze) {
        // Locates labeled vertices and removes edges in outer graph (entrance/exit)
        int entrance = st.getStart( );
        int exit = st.getEnd( );

        // create entrance:
        int[] vertLoc = getCoordinates(entrance, st.getWidth( ));
        int v = findVertAt(vertLoc[0], vertLoc[1], maze.getWidth( ));
        int u = findVertAt(vertLoc[0] + 1, vertLoc[1], maze.getWidth()); // down 1 row
        maze.removeEdge(v, u);

        // create exit
        vertLoc = getCoordinates(exit, st.getWidth( ));
        // go right one col and down one row
        v = findVertAt(vertLoc[0], vertLoc[1] + 1, maze.getWidth( ));
        u = findVertAt(vertLoc[0] + 1, vertLoc[1] + 1, maze.getWidth( ));
        maze.removeEdge(v, u);
    }

    public static int[] getCoordinates(int v, int graphWidth) {
        // Gets row and colum info for a graph
        int[] coordinates = new int[2];
        coordinates[0] = v / graphWidth; // Row
        coordinates[1] = v % graphWidth; // Column
        return coordinates;
    }

    public static boolean isVerticalEdge(int[] vertA, int[] vertB) {
        // Used to determine what edges to delete when making walls of maze
        return vertB[0] == (vertA[0] + 1);
    }

    public static boolean isHorizontalEdge(int[] vertA, int[]vertB) {
        // Also used to determine which edges to delete to make walls
        return vertB[1] == (vertA[1] + 1);
    }

    public static int findVertAt(int row, int col, int width) {
        // finds vertex determined by row, col, and graph width
        return row * width + col;
    }

    /**
     * Saves a maze as a text file. The format is as follows;
     * The first line contains two integers indicating the height and width
     * Vertices are labeled 0 ... (height*width-1) starting at the top
     * left corner and proceeding to the right and down.
     *
     * The remaining lines of the file indicate edges
     * each line contains two integers indicating an edge between
     * those two vertices.
     *
     * @param fname - the name of the text file to save typically ending in '-maze.txt'
     * @param maze - a graph representing the maze
     */
    public static void saveMaze(String fname, GridGraph maze) {
        try {
            FileWriter out = new FileWriter(new File(fname));
            out.write(maze.getWidth() + " " + maze.getHeight() + "\n");
            System.out.println("writing graph...");
            int nVertices = maze.nVertices();
            for(int v1 = 0; v1 < nVertices; v1++) {
                // maze.adjacent() should NOT return null, per the
                // interface description...it may return an empty list.
                for(int v2 : maze.adjacent(v1)) {
                    out.write(v1 + " " + v2 + "\n");
                }
            }
            out.close();
        } catch (IOException e) {
            System.err.println(e.getMessage());
        }
    }

    /**
     * Saves a ST as a text file. The format is as follows;
     * The first line contains two integers indicating the height and width
     * Vertices are labeled 0 ... (height*width-1) starting at the top
     * left corner and proceeding to the right and down
     *
     * The following lines of the file indicate edges
     * each line contains two integers indicating an edge between
     * those two vertices.
     *
     * The final two lines indicate the start and end locations on the ST of the
     * maze.  The start location must be a vertex along the left edge of the graph
     * and the end location must be a vertex along the right edge of the graph.
     *
     * @param fname - the name of the text file to save typically ending in '-maze.txt'
     * @param st - a spanning tree of the maze
     */
    public static void saveST(String fname, LabeledGridGraph st) {
        try {
            FileWriter out = new FileWriter(new File(fname));
            out.write(st.getWidth() + " " + st.getHeight() + "\n");
            System.out.println("writing graph...");
            List<Integer> labeledV;
            labeledV = st.getVertices("start");
            if (labeledV == null || labeledV.size() != 1) {
                System.err.println("'start' vertices look problematic. This graph should have exactly one vertex labeled 'start'");
            }
            labeledV = st.getVertices("end");
            if (labeledV == null || labeledV.size() != 1) {
                System.err.println("'end' vertices look problematic. This graph should have exactly one vertex labeled 'end'");
            }

            int nVertices = st.nVertices();
            for(int v1 = 0; v1 < nVertices; v1++) {
                for(int v2 : st.adjacent(v1)) {
                    out.write(v1 + " " + v2 + "\n");
                }
            }
            out.write( -1 + " " + (st.getVertices("start").get(0)) + " gray\n");
            out.write( -1 + " " + (st.getVertices("end").get(0)) + " gray\n");
            out.close();
        } catch (IOException e) {
            System.err.println(e.getMessage());
        }
    }


    static class Walls implements GridGraph {
        protected final Set<Integer>[] adjacencySet;
        protected final int height;
        protected final int width;
        private int numOfEdges;

        @SuppressWarnings("unchecked")
        public Walls(int w, int h) {
            this.width = w;
            this.height = h;
            numOfEdges = 0;
            adjacencySet = new Set[h * w];
            initiateAdjSet( );
        }

        private void initiateAdjSet( ) {
            // initially fill array with empty hashSets (no edges)
            for (int i = 0; i < adjacencySet.length; i++) {
                adjacencySet[i] = new HashSet<>();
            }
        }

        public void connectTheDots() {
            // Connects vertices in grid-like fashion
            for (int row = 0; row < height; row++) {
                for (int col = 0; col < width; col++) {
                    int v = MazeGenerator.findVertAt(row, col, width);

                    // neighbors to west and south
                    int w = MazeGenerator.findVertAt(row, col + 1, width);
                    int s = MazeGenerator.findVertAt(row + 1, col, width);

                    // connects edges to south and west. Accounts for edge cases.
                    if (row < height - 1) addEdge(v, s);
                    if (col < width - 1) addEdge(v, w);
                }
            }
        }

        public void connectTheDots_8Point() {
            // Connects vertices in grid-like fashion
            for (int row = 0; row < height; row++) {
                for (int col = 0; col < width; col++) {
                    int v = MazeGenerator.findVertAt(row, col, width);

                    // neighbors to west and south, sw and se
                    boolean canGoSouth = row < height -1;
                    boolean canGoWest = col < width - 1;
                    boolean canGoEast = col > 0;

                    if (canGoSouth) {
                        int s = MazeGenerator.findVertAt(row + 1, col, width);
                        addEdge(v,s);

                        if (canGoWest) {
                            int w = MazeGenerator.findVertAt(row, col + 1, width);
                            int sw = MazeGenerator.findVertAt(row+1, col+1, width);
                            addEdge(v,w);
                            addEdge(v,sw);
                        }

                        if (canGoEast) {
                            int se = MazeGenerator.findVertAt(row+1, col-1, width);
                            addEdge(v,se);
                        }

                    } else if (canGoWest) {
                        int w = MazeGenerator.findVertAt(row, col + 1, width);
                        addEdge(v,w);
                    }
                }
            }
        }

        public void addEdge(int v, int u) {
            // Adds edges to graph.
            if (v == u) throw new IllegalArgumentException("loops not allowed!");
            if (vertexDNE(v) || vertexDNE(u)) {
                throw new IndexOutOfBoundsException("vertex DNE!");
            }
            // If edge does not exist yet, add it! (undirected or both ways)
            if (!adjacent(v).contains(u) && !adjacent(u).contains(v)) {
                adjacencySet[v].add(u);
                adjacencySet[u].add(v);
                numOfEdges++;
            }
        }

        public void removeEdge(int v, int u) {
            // Removes edges from graph
            if (vertexDNE(v) || vertexDNE(u)) {
                throw new IndexOutOfBoundsException("vertex DNE!");
            }
            if (adjacent(v).contains(u)) {
                adjacencySet[v].remove(u);
                adjacencySet[u].remove(v);
                numOfEdges--;
            }
        }

        @Override
        public int getHeight( ) {
            return height;
        }

        @Override
        public int getWidth( ) {
            return width;
        }

        @Override
        public List<Integer> adjacent(int vertex) {
            // Returns adjacent vertices
            if (vertexDNE(vertex)) {
                throw new IndexOutOfBoundsException("vertex does not exist!");
            }
            return new ArrayList<>(adjacencySet[vertex]);
        }

        @Override
        public int nVertices( ) {
            // Size of graph
            return height * width;
        }

        public int getNumEdges( ) {
            return numOfEdges;
        }

        private boolean vertexDNE(int v) {
            return v < 0 || v >= nVertices();
        }

        public boolean edgeIsDiagonal(int vert1, int vert2) {
            int[] vert1YX = getCoordinates(vert1, getWidth());
            int[] vert2YX = getCoordinates(vert2, getWidth());
            return vert1YX[0] != vert2YX[0] && vert1YX[1] != vert2YX[1];
        }

        public String toString( ) {
            // used for testing. prints each vertex and adjacent edges
            StringBuilder grid = new StringBuilder( );
            for (int i = 0; i < nVertices(); i++) {
                grid.append(i).append("->").append(adjacent(i)).append("\n");
            }
            return new String(grid);
        }
    }


    static class Corridor extends Walls implements LabeledGridGraph {

        int start;
        int end;

        public Corridor(int w, int h) {
            super(w, h);
            pickEntryNExit();
        }

        private void pickEntryNExit( ) {
            // Picks random starting point
            Random rando = new Random( );
            int col = 0;
            int row = rando.nextInt(height);
            start = MazeGenerator.findVertAt(row, col, width);

            // pick random finish point
            col = width - 1;
            row = rando.nextInt(height);
            end = MazeGenerator.findVertAt(row, col, width);
        }

        // manual entrance & exit points
        public void setStartNEnd(int startRow, int endRow) {
            if (startRow < 0 || startRow >= height || endRow < 0 || endRow >= height) {
                throw new IndexOutOfBoundsException("Row out of bounds");
            }
            this.start = MazeGenerator.findVertAt(startRow, 0, width);
            this.end = MazeGenerator.findVertAt(endRow, width-1, width);
        }

        public int getStart( ) {
            // Returns entry point of maze
            return start;
        }

        public int getEnd( ) {
            // Returns exit point of maze
            return end;
        }

        @Override
        public List<Integer> getVertices(String label) {
            // Returns list of vertices with specified label
            List<Integer> labeledVert = new ArrayList<>( );
            if (label.equalsIgnoreCase("start")) {
                labeledVert.add(start);

            } else if (label.equalsIgnoreCase("end")) {
                labeledVert.add(end);

            } else return null;

            return labeledVert;
        }

        @Override
        public String getLabel(int v) {
            // Returns label of specified vertex
            if (v == start) return "start";
            if (v == end) return "end";
            return null;
        }
    }

    /**
     *
     * @param args <height> <width> <maze file name>
     */
    public static void main(String[] args) {

        int stW = Integer.parseInt(args[0]);
        int stH = Integer.parseInt(args[1]);

        LabeledGridGraph wilsonsST = MazeGenerator.generateST(stW, stH);
        LabeledGridGraph DFST = MazeGenerator.depthFirstST(stW, stH);
        LabeledGridGraph BFST = MazeGenerator.breadthFirstST(stW, stH);

        GridGraph wilsonsMaze = MazeGenerator.generateMaze(wilsonsST);
        GridGraph dfMaze = MazeGenerator.generateMaze(DFST);
        GridGraph bfMaze = MazeGenerator.generateMaze(BFST);

        MazeGenerator.saveST(args[2] + "-st.txt", wilsonsST);
        MazeGenerator.saveMaze(args[2] + "-maze.txt", wilsonsMaze);



        // tests:

//        MazeGenerator.Corridor grid = new MazeGenerator.Corridor(2, 2);
//        grid.generateGrid(); // connects all edges
//        System.out.println("grid = " + grid + "\n" + "graph size: " + grid.nVertices());
//        // Sees if it correctly chooses start and finishing vertices
//        System.out.println("starting point = " + grid.getStart() + "\n" +
//                "finish point = " + grid.getEnd());

//        LabeledGridGraph st = MazeGenerator.generateST(5, 5);
//        System.out.println("start: " + st.getVertices("start"));
//        System.out.println("end: " + st.getVertices("end"));
//        System.out.println("st = " + st);
    }
}

