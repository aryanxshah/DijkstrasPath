// --== CS400 File Header Information ==--
// Name: Aryan Shah
// Email: atshah2@wisc.edu
// Group and Team: E32
// Group TA: Caset Ford
// Lecturer: Garl Dahl
// Notes to Grader: <optional extra notes>

import java.util.*;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;


/**
 * This class extends the BaseGraph data structure with additional methods for
 * computing the total cost and list of node data along the shortest path
 * connecting a provided starting to ending nodes. This class makes use of
 * Dijkstra's shortest path algorithm.
 */
public class DijkstraGraph<NodeType, EdgeType extends Number>
        extends BaseGraph<NodeType, EdgeType>
        implements GraphADT<NodeType, EdgeType> {

    /**
     * While searching for the shortest path between two nodes, a SearchNode
     * contains data about one specific path between the start node and another
     * node in the graph. The final node in this path is stored in its node
     * field. The total cost of this path is stored in its cost field. And the
     * predecessor SearchNode within this path is referened by the predecessor
     * field (this field is null within the SearchNode containing the starting
     * node in its node field).
     *
     * SearchNodes are Comparable and are sorted by cost so that the lowest cost
     * SearchNode has the highest priority within a java.util.PriorityQueue.
     */
    protected class SearchNode implements Comparable<SearchNode> {
        public Node node;
        public double cost;
        public SearchNode predecessor;

        public SearchNode(Node node, double cost, SearchNode predecessor) {
            this.node = node;
            this.cost = cost;
            this.predecessor = predecessor;
        }

        public int compareTo(SearchNode other) {
            if (cost > other.cost)
                return +1;
            if (cost < other.cost)
                return -1;
            return 0;
        }
    }

    /**
     * Constructor that sets the map that the graph uses.
     * @param map the map that the graph uses to map a data object to the node
     *        object it is stored in
     */
    public DijkstraGraph(MapADT<NodeType, Node> map) {
        super(map);
    }

    /**
     * This helper method creates a network of SearchNodes while computing the
     * shortest path between the provided start and end locations. The
     * SearchNode that is returned by this method is represents the end of the
     * shortest path that is found: it's cost is the cost of that shortest path,
     * and the nodes linked together through predecessor references represent
     * all of the nodes along that shortest path (ordered from end to start).
     *
     * @param start the data item in the starting node for the path
     * @param end   the data item in the destination node for the path
     * @return SearchNode for the final end node within the shortest path
     * @throws NoSuchElementException when no path from start to end is found
     *                                or when either start or end data do not
     *                                correspond to a graph node
     */
    protected SearchNode computeShortestPath(NodeType start, NodeType end) {
        // implement in step 5.3
        if (!containsNode(start)) {
            throw new NoSuchElementException("The start node was not found in the graph");
        }

        if (!containsNode(end)) {
            throw new NoSuchElementException("The end node was not found in the graph");
        }


        PriorityQueue<SearchNode> needsVisit = new PriorityQueue<>();
        MapADT<NodeType, SearchNode> alreadyVisited = new PlaceholderMap<>();
        SearchNode startingNode = new SearchNode(nodes.get(start),0, null);
        needsVisit.offer(startingNode);
        alreadyVisited.put(start, startingNode);

        while (!needsVisit.isEmpty()) {
            SearchNode currentSearchNode = needsVisit.poll();

            if (currentSearchNode.node.data.equals(end)) {
                return currentSearchNode;
            }

            for (Edge edge : currentSearchNode.node.edgesLeaving) {
                NodeType nextData = edge.successor.data;
                double costOfPath = currentSearchNode.cost + edge.data.doubleValue();

                if (!alreadyVisited.containsKey(nextData) || alreadyVisited.get(nextData).cost > costOfPath) {
                    SearchNode nextNode = new SearchNode(edge.successor, costOfPath, currentSearchNode);
                    needsVisit.offer(nextNode);
                    try {
                        alreadyVisited.put(nextData, nextNode);
                    } catch (IllegalArgumentException e) {

                    }

                }
            }

        }
        throw new NoSuchElementException("There is no path that exists from the start to end");
    }


    /**
     * Returns the list of data values from nodes along the shortest path
     * from the node with the provided start value through the node with the
     * provided end value. This list of data values starts with the start
     * value, ends with the end value, and contains intermediary values in the
     * order they are encountered while traversing this shorteset path. This
     * method uses Dijkstra's shortest path algorithm to find this solution.
     *
     * @param start the data item in the starting node for the path
     * @param end   the data item in the destination node for the path
     * @return list of data item from node along this shortest path
     */
    public List<NodeType> shortestPathData(NodeType start, NodeType end) {
        // implement in step 5.4
        if(nodes.get(start) == null||nodes.get(end) ==  null){
            throw new NoSuchElementException("There is not a path found from the start to the end node.");
        }

        SearchNode endNode = computeShortestPath(start, end);
        List<NodeType> shortestPath = new ArrayList<>();

        while (endNode != null) {
            shortestPath.add(endNode.node.data);
            endNode = endNode.predecessor;
        }

        Collections.reverse(shortestPath);
        return shortestPath;
    }




    /**
     * Returns the cost of the path (sum over edge weights) of the shortest
     * path freom the node containing the start data to the node containing the
     * end data. This method uses Dijkstra's shortest path algorithm to find
     * this solution.
     *
     * @param start the data item in the starting node for the path
     * @param end   the data item in the destination node for the path
     * @return the cost of the shortest path between these nodes
     */
    public double shortestPathCost(NodeType start, NodeType end) {
        // implement in step 5.4
        return computeShortestPath(start, end).cost;
    }

    // TODO: implement 3+ tests in step 4.1

@Test
void test1() {
  // Create a map for node storage
        MapADT<Character, BaseGraph<Character, Integer>.Node> nodeMap = new PlaceholderMap<>();
        
        // Create an instance of DijkstraGraph
        DijkstraGraph<Character, Integer> example = new DijkstraGraph<>(nodeMap);

        Character A = 'A';
        Character B = 'B';
        Character C = 'C';
        Character D = 'D';
        Character E = 'E';


        // Add nodes to the graph
        example.insertNode('A');
        example.insertNode('B');
        example.insertNode('C');
        example.insertNode('D');
        example.insertNode('E');

        // Add edges with weights
        example.insertEdge('A', 'B', 5);
        example.insertEdge('A', 'C', 3);
        example.insertEdge('B', 'D', 4);
        example.insertEdge('C', 'D', 7);
        example.insertEdge('C', 'E', 6);
        example.insertEdge('D', 'E', 2);

        // Call Dijkstra's algorithm on specific start and end nodes
        char startNode = 'A';
        char endNode = 'E';

        double actualCost = example.shortestPathCost(startNode, endNode);
        List<Character> actualPath = example.shortestPathData(startNode, endNode);

        // Define the expected shortest path cost and sequence based on your hand computation.
        double expectedCost = 9;  // Replace with the expected cost.
        List<Character> expectedPath = List.of('A', 'C', 'E');  // Replace with the expected sequence of nodes.

        // Assert that the actual result matches the expected result.
        assertEquals(expectedCost, actualCost);
        assertIterableEquals(expectedPath, actualPath);
    }
 
@Test
void testPathCannotExist() {
// Create a map for node storage
        MapADT<Character, BaseGraph<Character, Integer>.Node> nodeMap = new PlaceholderMap<>();

        // Create an instance of DijkstraGraph
        DijkstraGraph<Character, Integer> graph = new DijkstraGraph<>(nodeMap);

        // Insert nodes to the graph
        graph.insertNode('A');
        graph.insertNode('B');
        graph.insertNode('C');
        graph.insertNode('D');
        graph.insertNode('E');

        // Insert edges with weights
        graph.insertEdge('A', 'B', 10);
        graph.insertEdge('A', 'C', 6);
        //graph.insertEdge('B', 'D', 8);
        //graph.insertEdge('C', 'D', 14);
        //graph.insertEdge('C', 'E', 12);
        graph.insertEdge('D', 'E', 4);

        // Call Dijkstra's algorithm on nodes that are not connected
        char startNode = 'A';
        char endNode = 'E';  // Replace with a node that is not connected to 'A'

        try {
            double actualCost = graph.shortestPathCost(startNode, endNode);
        }
        catch(NoSuchElementException e) {
            assertTrue(true);
            return;
        }

    assertTrue(false);
    }

@Test
void testShortestPath() {
MapADT<Character, BaseGraph<Character, Integer>.Node> nodeMap = new PlaceholderMap<>();

        // Create an instance of DijkstraGraph
        DijkstraGraph<Character, Integer> graph = new DijkstraGraph<>(nodeMap);

        // Insert nodes to the graph
        graph.insertNode('A');
        graph.insertNode('B');
        graph.insertNode('C');
        graph.insertNode('D');
        graph.insertNode('E');

        // Insert edges with weights
        graph.insertEdge('A', 'B', 15);
        graph.insertEdge('A', 'C', 9);
        graph.insertEdge('B', 'D', 12);
        graph.insertEdge('C', 'D', 21);
        graph.insertEdge('C', 'E', 18);
        graph.insertEdge('D', 'E', 6);

        // Call Dijkstra's algorithm on nodes
        char startNode = 'A';
        char endNode = 'E';

        double actualCost = graph.shortestPathCost(startNode, endNode);
        List<Character> actualPath = graph.shortestPathData(startNode, endNode);

        // Define the expected shortest path cost and sequence based on your hand computation.
        double expectedCost = 27;  // Replace with the expected cost.
        List<Character> expectedPath = List.of('A', 'C', 'E');  // Replace with the expected sequence of nodes.

        // Assert that the actual result matches the expected result.
        assertEquals(expectedCost, actualCost);
        assertIterableEquals(expectedPath, actualPath);
	    }


}

