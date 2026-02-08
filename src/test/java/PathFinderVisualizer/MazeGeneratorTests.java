package PathFinderVisualizer;


import org.junit.Test;

import java.util.List;

import static org.junit.Assert.*;

public class MazeGeneratorTests {

    @Test
    public void generateST() {
        LabeledGridGraph st = MazeGenerator.generateST(5, 6);
        assertEquals("ST width isn't set correctly", 5, st.getWidth());
        assertEquals("ST height isn't set correctly", 6, st.getHeight());

        assertNotNull("The ST should have exactly one start vertex.",
                st.getVertices("start"));
        assertNotNull("The ST should have exactly one end vertex.",
                st.getVertices("end"));
        assertEquals("The ST should have exactly one start vertex.",
                1, st.getVertices("start").size());
        assertEquals("The ST should have exactly one end vertex.",
                1, st.getVertices("end").size());

        assertEquals("The ST start vertex should be on the left edge of the grid.",
                0, st.getVertices("start").get(0) % st.getWidth());
        assertEquals("The ST end vertex should be on the right edge of the grid.",
                st.getWidth()-1, st.getVertices("end").get(0) % st.getWidth());
    }

    @Test
    public void checkSTLabelsIsAView() {
        LabeledGridGraph st = MazeGenerator.generateST(5, 6);
        assertEquals("The ST should have exactly one start vertex.",
                1, st.getVertices("start").size());
        // this should have no impact on the actual labels.
        st.getVertices("start").remove(0);
        assertEquals("Modifying the list of labels returned by getVertices should not impact the graph.",
                1, st.getVertices("start").size());
        st.getVertices("start").add(3);
        assertEquals("Modifying the list of labels returned by getVertices should not impact the graph.",
                1, st.getVertices("start").size());
    }

    @Test
    public void generateMaze() {
        LabeledGridGraph st = MazeGenerator.generateST(6, 5);
        GridGraph maze = MazeGenerator.generateMaze(st);
        assertEquals("Maze width isn't set correctly", 7, maze.getWidth());
        assertEquals("Maze height isn't set correctly", 6, maze.getHeight());
    }

    @Test
    public void createGrid() {
        MazeGenerator.Corridor maze = new MazeGenerator.Corridor(2, 2);
        maze.connectTheDots();
        assertEquals(4, maze.nVertices());
        List<Integer> adj0 = maze.adjacent(0);
        assertEquals(2, adj0.size());
        assertTrue(adj0.contains(1) && adj0.contains(2));
        List<Integer> adj1 = maze.adjacent(1);
        assertEquals(2, adj1.size());
        assertTrue(adj1.contains(0) && adj1.contains(3));
        List<Integer> adj2 = maze.adjacent(2);
        assertEquals(2, adj2.size());
        assertTrue(adj2.contains(0) && adj2.contains(3));
        List<Integer> adj3 = maze.adjacent(3);
        assertEquals(2, adj3.size());
        assertTrue(adj3.contains(1) && adj3.contains(2));
        System.out.println(maze);
    }
}