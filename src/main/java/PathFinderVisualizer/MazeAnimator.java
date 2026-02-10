package PathFinderVisualizer;


import javax.swing.*;
import java.awt.*;
import java.util.*;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.TimeUnit;

enum Strategy{A_STAR, BFS, DFS,}

public class MazeAnimator extends JPanel implements Runnable {

    private final Screen screen;
    private MazeGenerator.Corridor C, navGraph;
    boolean[] inST, vertsToDraw, roadBlocks;
    private List<Integer> lightningPath;
    private final MazeGenerator.Walls W;
    private final int tileSize;
    private int[] stXY0;
    private int[] wallXY0;
    private final int[] opacity;
    private int stOpacity;
    // Time delay stuff:
    private final int medDelay;
    private final int shrtDelay;
    private final int lngDelay;
    private final int delay;
    private final int fadeSpeed;

    private boolean ready2Solve, allDone, autoMaze;
    private final Strategy searchMethod;

    // constructor
    public MazeAnimator(int w, int h, boolean auto_maze, Strategy search_method, int speed) {
        JFrame frame = new JFrame("Maze Animator");
        screen = new Screen();
        frame.add(screen);

        C = new MazeGenerator.Corridor(w, h);
        navGraph = new MazeGenerator.Corridor(w, h);
        W = new MazeGenerator.Walls(w+1, h+1);

        autoMaze = auto_maze;
        searchMethod = search_method;

        vertsToDraw = new boolean[C.nVertices()];
        roadBlocks = new boolean[C.nVertices()];
        inST = new boolean[C.nVertices()];
        lightningPath = new ArrayList<Integer>();
        opacity = new int[C.nVertices()];
        Arrays.fill(opacity, 0);

        tileSize = 1200/(w + h);
        int screenWidth = tileSize * (w + 4);
        int screenHeight = tileSize * (h + 4);
        stOpacity = 255;

        medDelay = 1000/(w+h);
        shrtDelay = 100/(w+h);
        lngDelay = 5000/(w+h);
        if (speed == 1)
            delay = lngDelay;
        else if (speed == 3)
            delay = shrtDelay;
        else delay = medDelay;

        fadeSpeed = 15;
        ready2Solve = false;
        allDone = false;

        setStartPoints();

        frame.setSize(screenWidth, screenHeight);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLocationRelativeTo(null); // Centers the frame
        frame.setVisible(true);
    }

    public void go() {
        new Thread(this).start();
    }

    @Override
    public void run() {
        
        if (autoMaze) {
            navGraph.connectTheDots();
            createST();
            generateMaze();
            solveMaze();
        } else {
            vertsToDraw[C.getStart()] = true;
            vertsToDraw[C.getEnd()] = true;
            navGraph.connectTheDots_8Point();
            generateGrid();
        }
    }

    private void setStartPoints() {
        stXY0 = new int[2];
        wallXY0 = new int[2];
        stXY0[0] = stXY0[1] = tileSize + (tileSize / 2);
        wallXY0[0] = wallXY0[1] = tileSize;
    }

    // creates road blocks
    private void handleClick(int x, int y, boolean makeBlocks) {
        // don't allow any more editing once algo has started
        int v = vertexAt(x, y);
        if (ready2Solve || v == -1) return;
        roadBlocks[v] = makeBlocks;
        reFresh(medDelay);
    }

    private void createST() {
        int numVerts = navGraph.nVertices( );

        // keeps track of vertices in our ST
        inST[navGraph.getEnd( )] = true; // add end point to ST
        vertsToDraw[navGraph.getEnd( )] = true;
        Random rando = new Random( );

        // Main loop keeps adding edges until all vertices are reached
        while (C.getNumEdges( ) < numVerts - 1) {
            // picks random start point (not in tree)
            int curVert = rando.nextInt(numVerts);
            while (inST[curVert]) {
                curVert = rando.nextInt(numVerts);
            }

            vertsToDraw[curVert] = true;
            // keeps track of a path that is our "random walk";
            int[] prev = new int[numVerts];
            Arrays.fill(prev, -1); // lets us know which verts have been visited
            prev[curVert] = -2; // marks our start point.

            // Keeps randomly walking until it connects with vertex already in tree
            while (!inST[curVert]) {
                // walks to random adj vertices
                // If a loop appears, it is erased and it continues
                List<Integer> adjacentVs = navGraph.adjacent(curVert);
                int nxtVert = adjacentVs.get(rando.nextInt(adjacentVs.size( )));
                if (prev[nxtVert] == -1) { // not visited yet
                    prev[nxtVert] = curVert;
                    C.addEdge(curVert, nxtVert);

                    vertsToDraw[curVert] = true;
                    vertsToDraw[nxtVert] = true;

                    // It ran into itself, erase loop!
                } else eraseLoop(curVert, nxtVert, prev);

                curVert = nxtVert; // walk to next vertex
                reFresh(shrtDelay);
            }
            addWalkTo(inST, prev, curVert);
        }
    }

    public void generateMaze() {
        generateGrid();
        int stWidth = C.getWidth( );
        int mzWidth = W.getWidth( );

        // locate coordinates of all edges and delete any edges that cross
        // First get row and col of corridor edges
        for (int i = 0; i < C.nVertices( ); i++) {
            int[] vertA = MazeGenerator.getCoordinates(i, stWidth);

            for (Integer adj : C.adjacent(i)) {
                // Translate these coordinates to edges to delete in maze graph
                int[] vertB = MazeGenerator.getCoordinates(adj, stWidth);
                int v = MazeGenerator.findVertAt(vertB[0], vertB[1], mzWidth);

                if (MazeGenerator.isVerticalEdge(vertA, vertB)) {
                    // The edge to delete will be horizontal (col + 1)
                    int u = MazeGenerator.findVertAt(vertB[0], vertB[1] + 1, mzWidth);
                    W.removeEdge(v, u);

                } else if (MazeGenerator.isHorizontalEdge(vertA, vertB)) {
                    // The edge to delete will be vertical (row + 1)
                    int u = MazeGenerator.findVertAt(vertB[0] + 1, vertB[1], mzWidth);
                    W.removeEdge(v, u);
                }
                reFresh(shrtDelay);
            }
        }
        createEntranceNExit();
    }

    private void generateGrid() {
        int width = W.getWidth();
        int  height = W.getHeight();

        for (int row = 0; row < height; row++) {
            for (int col = 0; col < width; col++) {
                int v = MazeGenerator.findVertAt(row, col, width);

                // neighbors to west and south
                int w = MazeGenerator.findVertAt(row, col + 1, width);
                int s = MazeGenerator.findVertAt(row + 1, col, width);

                // connects edges to south and west. Accounts for edge cases.
                if (row < height - 1) W.addEdge(v, s);
                if (col < width - 1) W.addEdge(v, w);

                reFresh(shrtDelay);
            }
        }
    }

    private void createEntranceNExit() {
        // Locates labeled vertices and removes edges in outer graph (entrance/exit)
        int entrance = C.getStart( );
        int exit = C.getEnd( );

        // create entrance:
        int[] vertLoc = MazeGenerator.getCoordinates(entrance, C.getWidth( ));
        int v = MazeGenerator.findVertAt(vertLoc[0], vertLoc[1], W.getWidth( ));
        int u = MazeGenerator.findVertAt(vertLoc[0] + 1, vertLoc[1], W.getWidth()); // down 1 row
        W.removeEdge(v, u);
        reFresh(lngDelay);

        // create exit
        vertLoc = MazeGenerator.getCoordinates(exit, C.getWidth( ));
        // go right one col and down one row
        v = MazeGenerator.findVertAt(vertLoc[0], vertLoc[1] + 1, W.getWidth( ));
        u = MazeGenerator.findVertAt(vertLoc[0] + 1, vertLoc[1] + 1, W.getWidth( ));
        W.removeEdge(v, u);
        reFresh(lngDelay);

        fadeOutST();
    }

    private void fadeOutST() {
        while (stOpacity != 0) {
            stOpacity -= 5;
            reFresh(medDelay);
        }
    }

    private void solveMaze() {
        if (autoMaze) {
            navGraph = C; // copy the maze path

            // delete old ST, create new empty graph (reset values)
            int s = C.getStart();
            int f = C.getEnd();
            int[] sXY = MazeGenerator.getCoordinates(s, C.getWidth());
            int[] fXY = MazeGenerator.getCoordinates(f, C.getWidth());
            C = new MazeGenerator.Corridor(C.getWidth(), C.getHeight());
            C.setStartNEnd(sXY[0], fXY[0]);
            Arrays.fill(vertsToDraw, false); // reset to empty
            Arrays.fill(inST, false);
            stOpacity = 255;

            // show start and end points
            vertsToDraw[C.getStart()] = true;
            vertsToDraw[C.getEnd()] = true;
        }

        Stack<Integer> path2Goal;
        if (searchMethod == Strategy.A_STAR)
            path2Goal = A_Star();
        else if (searchMethod == Strategy.BFS)
            path2Goal = BFS();
        else path2Goal = DFS();

        if (path2Goal == null || path2Goal.isEmpty())
            throw new IllegalStateException("didn't find a solution to maze");

        allDone = true;
        fadeOutST();
        followBreadCrumbs(path2Goal);
    }

    // search algos
    private Stack<Integer> DFS() {
        int s = C.getStart();
        int f = C.getEnd();
        int numV = navGraph.nVertices();

        Deque<Integer> nxt = new ArrayDeque<>();
        boolean[] prevVisited = new boolean[numV];
        int[] parentOf = new int[numV];

        parentOf[s] = -1;
//        prevVisited[s] = true;
        nxt.push(s);
        while (!nxt.isEmpty()) {
            int curVertex = nxt.pop();
            if (prevVisited[curVertex]) continue;
            prevVisited[curVertex] = true;
            inST[curVertex] = true;

            add2LightningStrike(curVertex, -1, parentOf);
            reFresh(delay);

            if (curVertex == f)
                return findPathBack(parentOf, curVertex);

            List<Integer> neighbors = navGraph.adjacent(curVertex);
            if (neighbors.isEmpty() && !nxt.isEmpty()) { // dead end -> backtracking visual
                add2LightningStrike(curVertex, parentOf[nxt.peek()], parentOf);
                reFresh(delay);
            } else {
                Collections.shuffle(neighbors, new Random());
                for (int adj : neighbors) {
                    if (!prevVisited[adj] && !roadBlocks[adj]) {
                        nxt.push(adj);
//                        prevVisited[adj] = true;
                        vertsToDraw[adj] = true;
                        parentOf[adj] = curVertex;
                        reFresh(delay);
                    }
                }
            }
        }

        // no path found
        return null;
    }

    private Stack<Integer> A_Star() {
        int start = C.getStart(), finish = C.getEnd();
        int totVerts = navGraph.nVertices();

        String dist_metric;
        if (autoMaze) dist_metric = "manhattan";
        else dist_metric = "octile";

        // g-score = tot distance so far
        // f-score = g-score + distance to finish
        double[] g_cost = new double[totVerts], f_cost = new double[totVerts], h_cost = new double[totVerts];
        int[] prevVertOf = new int[totVerts]; // used to find path back
        boolean[] visited = new boolean[totVerts]; // prevents going in circles

        Arrays.fill(prevVertOf, -1);
        Arrays.fill(g_cost, Double.POSITIVE_INFINITY);
        Arrays.fill(f_cost, Double.POSITIVE_INFINITY);
        Arrays.fill(h_cost, Double.POSITIVE_INFINITY);
        g_cost[start] = 0.0;

        // priority based on f-cost first, then h-cost 2nd
        PriorityQueue<Integer> pq = new PriorityQueue<>((a,b) -> {
            if (f_cost[a] != f_cost[b])
                return Double.compare(f_cost[a], f_cost[b]);
            else return Double.compare(h_cost[a], h_cost[b]);
        });

        pq.add(start);
        while (!pq.isEmpty()) {
            int currVert = pq.poll();

            // update visual stuff
            inST[currVert] = true;
            visited[currVert] = true;
            // follow path back to start each time... looks like a lightening strike
            add2LightningStrike(currVert, -1, prevVertOf);
            reFresh(delay);

            // check for winner
            if (currVert == finish)
                return findPathBack(prevVertOf, finish);


            // else spread out
            for (Integer adj : navGraph.adjacent(currVert)) {
                // calc new g-cost
                double edgeWeight = 1.0;
                if (C.edgeIsDiagonal(currVert, adj))
                    edgeWeight = Math.sqrt(2.0);
                double updated_Gcost = g_cost[currVert] + edgeWeight;

                // only update if new g_cost is better and never been there before
                if (!visited[adj] && !roadBlocks[adj] && updated_Gcost < g_cost[adj]) {
                    // update f_cost and push to Queue
                    g_cost[adj] = updated_Gcost;
                    h_cost[adj] = dist_heuristic(adj, finish, dist_metric);
                    f_cost[adj] = updated_Gcost + h_cost[adj];
                    prevVertOf[adj] = currVert;
                    pq.add(adj);
                    vertsToDraw[adj] = true;
                    reFresh(delay);
                }
            }
        }

        // couldn't find a path
        return null;
    }

    private Stack<Integer> BFS() {
        int s = C.getStart();
        int f = C.getEnd();
        int numV = navGraph.nVertices();

        Queue<Integer> nxt = new ArrayDeque<>();
        boolean[] prevVisited = new boolean[numV];
        int[] wayBack = new int[numV];
        wayBack[s] = -1;

        prevVisited[s] = true;
        nxt.add(s);
        while (!nxt.isEmpty()) {
            int currVert = nxt.poll();
            inST[currVert] = true;
            add2LightningStrike(currVert, -1, wayBack);
            reFresh(delay);

            if (currVert == f)
                return findPathBack(wayBack, f);

            for (int vertex : navGraph.adjacent(currVert)) {
                if (!prevVisited[vertex] && !roadBlocks[vertex]) {
                    nxt.add(vertex);
                    prevVisited[vertex] = true;
                    vertsToDraw[vertex] = true;
                    wayBack[vertex] = currVert;
                    reFresh(delay);
                }
            }
        }

        // didn't find target
        return null;
    }

    private void add2LightningStrike(int v, int u, int[] pathHome) {
        lightningPath.clear();
        while(v != u) {
            lightningPath.add(v);
            v = pathHome[v];
        }
    }

    private void followBreadCrumbs(Stack<Integer> wayBack) {
        int v = wayBack.pop();
        while(!wayBack.isEmpty()) {
            int u = wayBack.pop();
            C.addEdge(v, u);
            inST[v] = inST[u] = false;
            v = u;
            reFresh(delay);
        }
    }

    private double dist_heuristic(int currVert, int finish, String type) {
        int[] currYX = MazeGenerator.getCoordinates(currVert, C.getWidth());
        int[] finishYX = MazeGenerator.getCoordinates(finish, C.getWidth());
        float dx = Math.abs(currYX[1] - finishYX[1]);
        float dy = Math.abs(currYX[0] - finishYX[0]);
        return switch (type) {
            case "octile" -> dx + dy + (Math.sqrt(2.0) - 1) * Math.min(dx, dy);
            case "manhattan" -> dx + dy;
            case "Dikstra's" -> 0;
            default -> Math.sqrt((dx*dx) + (dy*dy));
        };
    }

    private static Stack<Integer> findPathBack(int[] prev, int f) {
        Stack<Integer> wayBack = new Stack<>();
        wayBack.push(f);
        int nxt = prev[f];
        while(nxt != -1) {
            wayBack.push(nxt);
            nxt = prev[nxt];
        }
        return wayBack;
    }

    private void eraseLoop(int strt, int stop, int[] prev) {
        // walks backwards over path and
        // erases (resets values) the loop.
        C.addEdge(strt, stop);
        reFresh(medDelay);
        C.removeEdge(strt, stop);
        reFresh(medDelay);

        int idx = strt;
        int currV = prev[strt];
        int endOfLoop = prev[stop];

        while (currV != endOfLoop) {
            prev[idx] = -1;
            C.removeEdge(currV, idx);
            vertsToDraw[idx] = false;
            vertsToDraw[currV] = false;
            idx = currV;
            currV = prev[currV];

            reFresh(medDelay);
        }
    }

    private void addWalkTo(boolean[] inTree, int[] prev, int curV) {
        // walks back through walk/path and adds edges to our ST
        while (prev[curV] != -2) {
            int v = prev[curV];
            int u = curV;

            inTree[v] = true;
            inTree[u] = true;
            // increment down path
            curV = prev[curV];
        }
    }

    // Drawing stuff:
    private void reFresh(int sleepTime) {
        screen.repaint();
        try {
            TimeUnit.MILLISECONDS.sleep(sleepTime);
        } catch (InterruptedException e) {
            System.err.println(e.getMessage());
        }
    }

    private int[] findXY(int v, int[] xy0, MazeGenerator.Walls w) {
        int[] xy = new int[2];
        int[] rowCol = MazeGenerator.getCoordinates(v, w.getWidth());
        xy[0] = xy0[0] + (rowCol[1] * tileSize);
        xy[1] = xy0[1] + (rowCol[0] * tileSize);
        return xy;
    }

    // inverse - used for road blocks
    private int vertexAt(int mouseX, int mouseY) {
        int col = mouseX / tileSize - 1;
        int row = mouseY / tileSize - 1;

        if (row < 0 || row >= C.getHeight()) return -1;
        if (col < 0 || col >= C.getWidth()) return -1;

        return MazeGenerator.findVertAt(row, col, C.getWidth());
    }

    private void drawBricks(Graphics2D g2d) {
        for (int i = 0; i < roadBlocks.length; i++) {
            int[] coord = findXY(i, stXY0, C);
            int sqSize = tileSize - 8; // slightly smaller than full tile
            if (roadBlocks[i]) {
                g2d.setColor(Color.DARK_GRAY); // wall
                g2d.fillRect(coord[0] - sqSize/2, coord[1] - sqSize/2, sqSize, sqSize);
            }
        }
    }

    private void drawVerts(Graphics2D g2d) {
        for (int i = 0; i < vertsToDraw.length; i++) {
            if (vertsToDraw[i]) {

                opacity[i] = Math.min(opacity[i] + fadeSpeed, 255);
                int finalOpacity = Math.min(opacity[i], stOpacity);
                int[] coord = findXY(i, stXY0, C);
                int vertSize = tileSize / 3;

                int jX = 0, jY = 0;

                Color c;
                if (i == C.getStart() || i == C.getEnd()) {
                    c = new Color(0, 255, 0, finalOpacity);
                } else if (inST[i]) {
                    c = new Color(255, 255, 255, finalOpacity);
                } else {
                    c = new Color(255, 12, 0, finalOpacity);
                    jX = (int)(Math.random() * 4 - 2); // -2..2
                    jY = (int)(Math.random() * 4 - 2);
                }

                g2d.setColor(c);
                g2d.fillOval(coord[0] - (vertSize/2) + jX, coord[1] - (vertSize/2) + jY, vertSize, vertSize);
            }
        }
    }

    private void drawInnerEdges(Graphics2D g2d) {
        // for each vertex in adjacencyList, if set isn't empty
        // get coord of vertA and adjVert, then convert to x,y
        // then draw line from point a -> b
        for (int i = 0; i < C.nVertices( ); i++) {
            int[] ptA = findXY(i, stXY0, C);
            for (Integer adj : C.adjacent(i)) {
                int[] ptB = findXY(adj, stXY0, C);
                g2d.setStroke(new BasicStroke(tileSize / 10f));
                Color c = new Color(182, 180, 179, stOpacity);
                if (!inST[i] && !inST[adj])
                    c = Color.RED;
                g2d.setColor(c);
                g2d.drawLine(ptA[0], ptA[1], ptB[0], ptB[1]);
            }
        }
    }

    private void drawOuterEdges(Graphics2D g2d) {
        for (int i = 0; i < W.nVertices( ); i++) {
            int[] ptA = findXY(i, wallXY0, W);
            for (Integer adj : W.adjacent(i)) {
                int[] ptB = findXY(adj, wallXY0, W);
                g2d.setStroke(new BasicStroke(tileSize / 10f));
                g2d.setColor(Color.blue);
                g2d.drawLine(ptA[0], ptA[1], ptB[0], ptB[1]);
            }
        }
    }

    private void drawLightning(Graphics2D g2d) {
        if (lightningPath.isEmpty()) return;

        final float MAX_STROKE = tileSize / 6f;   // ≈ 4 visually
        final float MIN_STROKE = tileSize / 16f;  // ≈ 1 visually

        // optional: reverse path so lightning starts thick at start node
        List<Integer> path = new ArrayList<>(lightningPath);
        Collections.reverse(path);

        for (int i = 0; i < path.size() - 1; i++) {
            int aV = path.get(i);
            int bV = path.get(i + 1);

            int[] a = findXY(aV, stXY0, C);
            int[] b = findXY(bV, stXY0, C);

            // taper stroke: thicker near start, thinner near tip
            float t = (float)i / (path.size() - 1);
            float stroke = MAX_STROKE - t * (MAX_STROKE - MIN_STROKE);

            // subtle pulse, just enough to animate without strobe
            if (!allDone) {
                float pulse = 0.95f + 0.05f * (float)Math.sin(System.nanoTime() * 1e-9 * 6);
                stroke *= pulse;
            }

            // add a faint glow / halo
            g2d.setStroke(new BasicStroke(stroke + 1.5f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
            g2d.setColor(new Color(0, 255, 255, 80)); // cyan glow
            int jx = (int)(Math.random() * 4 - 2);
            int jy = (int)(Math.random() * 4 - 2);
            g2d.drawLine(a[0], a[1], b[0] + jx, b[1] + jy);

            // main lightning line
            g2d.setStroke(new BasicStroke(stroke, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
            int alpha = allDone ? 255 : 200;
            g2d.setColor(allDone ? new Color(0, 255, 0, alpha) : new Color(0, 255, 255, alpha));
            jx = (int)(Math.random() * 4 - 2);
            jy = (int)(Math.random() * 4 - 2);
            g2d.drawLine(a[0], a[1], b[0] + jx, b[1] + jy);
        }
    }


    private class Screen extends JPanel {

        public Screen() {
            this.setDoubleBuffered(true);

            // for mouse stuff - creating road blocks
            addMouseListener(new java.awt.event.MouseAdapter() {
                @Override
                public void mousePressed(java.awt.event.MouseEvent e) {
                    boolean makeBlocks = SwingUtilities.isLeftMouseButton(e);
                    handleClick(e.getX(), e.getY(), makeBlocks);
                }
            });

            addMouseMotionListener(new java.awt.event.MouseMotionAdapter() {
                @Override
                public void mouseDragged(java.awt.event.MouseEvent e) {
                    boolean makeBlocks = SwingUtilities.isLeftMouseButton(e);
                    handleClick(e.getX(), e.getY(), makeBlocks);
                }
            });

            getInputMap(WHEN_IN_FOCUSED_WINDOW)
                    .put(KeyStroke.getKeyStroke("ENTER"), "startAStar");

            getActionMap().put("startAStar", new AbstractAction() {
                @Override
                public void actionPerformed(java.awt.event.ActionEvent e) {
                    ready2Solve = true;
                    // start A* on a new thread so the GUI doesn't freeze
                    new Thread(MazeAnimator.this::solveMaze).start();
                }
            });
        }

        public void paintComponent(Graphics g) {
            super.paintComponent(g);
            setBackground(Color.BLACK);
            Graphics2D g2d = (Graphics2D) g;
            drawVerts(g2d);
            drawBricks(g2d);
            drawInnerEdges(g2d);
            drawOuterEdges(g2d);
            drawLightning(g2d);
        }
    }

    // Helper for parsing user input with error handling and default values
    private static int parseIntArg(String arg, int def, int min, int max, String name) {
        try {
            int value = Integer.parseInt(arg);
            if (value < min || value > max) {
                System.err.printf("Invalid %s '%s' (range %d-%d). Using default %d.%n",
                        name, arg, min, max, def);
                return def;
            }
            return value;
        } catch (NumberFormatException e) {
            System.err.printf("Invalid %s '%s'. Using default %d.%n", name, arg, def);
            return def;
        }
    }


    public static void main(String[] args) {
        final int MAX_GRID = 100;
        final int DEFAULT_SIZE = 25;
        final int DEFAULT_SPEED = 2;

        int width = DEFAULT_SIZE;
        int height = DEFAULT_SIZE;
        boolean autoMaze = false;
        Strategy strategy = Strategy.A_STAR;
        int speed = DEFAULT_SPEED;

        // uses command line args if present, else defaults used
        if (args.length >= 1) 
            width = parseIntArg(args[0], DEFAULT_SIZE, 1, MAX_GRID, "width");
        
        if (args.length >= 2) 
            height = parseIntArg(args[1], DEFAULT_SIZE, 1, MAX_GRID, "height");
        
        if (args.length >= 3) 
            autoMaze = args[2].equalsIgnoreCase("auto");
        
        if (args.length >= 4) {
            try {
                strategy = Strategy.valueOf(args[3].toUpperCase());
            } catch (IllegalArgumentException e) {
                System.err.printf("Unknown strategy '%s'. Using A_STAR.%n", args[3]);
                strategy = Strategy.A_STAR;
            }
        }
            
        if (args.length >= 5) 
            speed = parseIntArg(args[4], DEFAULT_SPEED, 1, 3, "speed");
        
        new MazeAnimator(width, height, autoMaze, strategy, speed).go();
    }
}
