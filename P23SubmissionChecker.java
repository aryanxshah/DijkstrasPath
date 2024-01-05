import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Assertions;

import java.util.ArrayList;
import java.util.NoSuchElementException;

/**
 * This class extends the DijsktraGraph class to run submission checks on it.
 */
public class P23SubmissionChecker extends DijkstraGraph<Integer, Integer> {

    /**
     * Default constructor so that JUnit test runner can instantiate test class.
     */
    public P23SubmissionChecker() {
        super(new PlaceholderMap<>());
    }

    /**
     * Creates a graph with two nodes and a single edge between them. Then
     * checks that the cost of a path starting and ending at the same node
     * is 0.
     */
    @Test
    public void submissionCheckerSameNode() {
        DijkstraGraph<String, Integer> graph = new DijkstraGraph<>(new PlaceholderMap<>());
        graph.insertNode("A");
        graph.insertNode("B");
        graph.insertEdge("A", "B", 34);

        Assertions.assertEquals(0, graph.shortestPathCost("B", "B"));
    }

    /**
     * Creates a graph with two nodes and a single edge between them. Then
     * checks that the cost of the path between those nodes equals the weight
     * of the edge.
     */
    @Test
    public void submissionCheckerSingleEdge() {
        System.out.println("single edge");
        DijkstraGraph<String, Integer> graph = new DijkstraGraph<>(new PlaceholderMap<>());
        graph.insertNode("A");
        graph.insertNode("B");
        graph.insertEdge("A", "B", 34);

        Assertions.assertEquals(34, graph.shortestPathCost("A", "B"));
        System.out.println("single edge end");
    }

    /**
     * Creates a graph with 3 nodes and 7 edges and checks the path that
     * returned between two of the nodes based on the SearchNode created
     * by the Djikstra method.
     */
    @Test
    public void submissionCheckerSearchNodePath() {
        DijkstraGraph<Integer, Integer> graph = new DijkstraGraph<>(new PlaceholderMap<>());
        graph.insertNode(0);
        graph.insertNode(1);
        graph.insertNode(2);

        graph.insertEdge(0, 1, 9);
        graph.insertEdge(0, 0, 1);
        graph.insertEdge(0, 1, 4);
        graph.insertEdge(0, 2, 5);
        graph.insertEdge(2, 0, 2);
        graph.insertEdge(2, 0, 4);
        graph.insertEdge(0, 2, 3);

        ArrayList<Integer> pathData = new ArrayList<>();
        SearchNode current = graph.computeShortestPath(2, 1);

        Assertions.assertEquals(8, current.cost);

        while (current != null) {
            pathData.add(0, current.node.data);
            current = current.predecessor;
        }

        Assertions.assertEquals("[2, 0, 1]", pathData.toString());
    }

    /**
     * Creates a graph with 3 nodes and 7 edges and checks the path that
     * returned between two of the nodes based on the shortestPathData
     * and shortestPathData methods.
     */
    @Test
    public void submissionCheckerSmallGraph() {
        DijkstraGraph<Integer, Integer> graph = new DijkstraGraph<>(new PlaceholderMap<>());
        graph.insertNode(1);
        graph.insertNode(6);
        graph.insertNode(11);

        graph.insertEdge(11, 1, 3);
        graph.insertEdge(6, 11, 3);
        graph.insertEdge(1, 11, 5);
        graph.insertEdge(6, 6, 1);
        graph.insertEdge(1, 6, 4);
        graph.insertEdge(6, 1, 6);
        graph.insertEdge(6, 11, 7);

        Assertions.assertEquals(7, graph.shortestPathCost(6, 11));
        Assertions.assertEquals("[6, 11]", graph.shortestPathData(6, 11).toString());
    }

    /**
     * Check that a NoSuchElementException is thrown if a path from start to end
     * node does not exist.
     */
    @Test
    public void submissionCheckerNoPath() {
        DijkstraGraph<String, Integer> graph = new DijkstraGraph<>(new PlaceholderMap<>());
        graph.insertNode("A");
        graph.insertNode("B");
        graph.insertNode("C");

        graph.insertEdge("A", "B", 2);
        graph.insertEdge("B", "C", 3);

        Assertions.assertThrows(NoSuchElementException.class, () -> graph.computeShortestPath("B", "A"));
    }

    /**
     * Check that a NoSuchElementException is thrown if start node does not exist
     * in graph.
     */
    @Test
    public void submissionCheckerNoNode() {
        DijkstraGraph<String, Integer> graph = new DijkstraGraph<>(new PlaceholderMap<>());
        graph.insertNode("A");
        graph.insertNode("B");
        graph.insertNode("C");

        graph.insertEdge("A", "B", 2);
        graph.insertEdge("B", "C", 3);

        Assertions.assertThrows(NoSuchElementException.class, () -> graph.computeShortestPath("X", "A"));
    }

    /**
     * Checks data values of SearchNodes along a short path.
     */
    @Test
    public void submissionCheckerSearchNodeData() {
        DijkstraGraph<Integer, Integer> graph = new DijkstraGraph<>(new PlaceholderMap<>());
        graph.insertNode(13);
        graph.insertNode(12);
        graph.insertNode(14);
        graph.insertNode(16);

        graph.insertEdge(13, 12, 2);
        graph.insertEdge(12, 14, 4);
        graph.insertEdge(14, 16, 6);
        graph.insertEdge(16, 13, 3);
        graph.insertEdge(14, 12, 1);
        graph.insertEdge(12, 16, 15);

        SearchNode current = graph.computeShortestPath(13, 16);
        ArrayList<Integer> pathData = new ArrayList<>();
        while (current != null) {
            pathData.add(0, current.node.data);
            current = current.predecessor;
        }
        Assertions.assertEquals("[13, 12, 14, 16]", pathData.toString());
    }

    /**
     * Checks cost of SearchNodes along a short path.
     */
    @Test
    public void submissionCheckerSearchNodeCost() {
        DijkstraGraph<Integer, Integer> graph = new DijkstraGraph<>(new PlaceholderMap<>());
        graph.insertNode(13);
        graph.insertNode(12);
        graph.insertNode(14);
        graph.insertNode(16);

        graph.insertEdge(13, 12, 2);
        graph.insertEdge(12, 14, 4);
        graph.insertEdge(14, 16, 6);
        graph.insertEdge(16, 13, 3);
        graph.insertEdge(14, 12, 1);
        graph.insertEdge(12, 16, 15);

        SearchNode current = graph.computeShortestPath(13, 16);
        ArrayList<Double> pathCost = new ArrayList<>();
        while (current != null) {
            pathCost.add(0, current.cost);
            current = current.predecessor;
        }
        Assertions.assertEquals("[0.0, 2.0, 6.0, 12.0]", pathCost.toString());
    }

}
