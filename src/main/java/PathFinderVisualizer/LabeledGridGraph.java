package PathFinderVisualizer;

import java.util.List;

/**
 * Provides a mapping between labels and vertex numbers
 * for a grid based graph.
 */
public interface LabeledGridGraph extends GridGraph {

    /**
     * Returns a list of vertices with the given label
     * or null if no vertices with that label exist. This
     * method should return a view (or copy) of the list of vertices with
     * the specified label so that modifying the returned list
     * WILL NOT impact the actual labels on the graph.
     *
     * @param label the label to search for
     * @return a view of the list of vertices with the specified label
     */
    public List<Integer> getVertices(String label);

    /**
     * Returns the label of a given vertex, or null if
     * the vertex is unlabeled.
     *
     * @param v the vertex id
     * @return a label, or null
     */
    public String getLabel(int v);
}
