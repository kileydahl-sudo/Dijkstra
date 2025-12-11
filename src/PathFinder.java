import java.util.*;

/**
 * PathFinder class containing Dijkstra's shortest path algorithm.
 * Students must implement the algorithm using a PriorityQueue.
 */
public class PathFinder {

    private Graph graph;

    public PathFinder(Graph graph) {
        this.graph = graph;
    }

    /**
     * Implements Dijkstra's shortest path algorithm using a PriorityQueue.
     *
     * @param startIndex the index of the starting node
     * @param endIndex   the index of the destination node
     * @return a DijkstraResult containing all steps and the final path
     */
    public DijkstraResult findShortestPath(int startIndex, int endIndex) {
        DijkstraResult result = new DijkstraResult();

        // 1. INITIALIZATION
        // TODO: Initialize distances array with Integer.MAX_VALUE for all nodes
        int distances[] = new int[graph.NUM_NODES];
        for (int i = 0; i < distances.length; i++) {
            distances[i] = Integer.MAX_VALUE;
        }

        // TODO: Initialize previous array with -1 for all nodes
        int prev[] = new int[graph.NUM_NODES];
        for (int i = 0; i < prev.length; i++) {
            prev[i] = -1;
        }

        // TODO: Initialize visited set

       // boolean visited[] = new boolean[graph.NUM_NODES];
        Set<Integer> visited = new HashSet<Integer>();


        // TODO: Set distance to start node as 0
        distances[startIndex] = 0;

        // TODO: Create a PriorityQueue of NodeDistance objects
        // The PriorityQueue should order by distance (smallest first)
        PriorityQueue<NodeDistance> myPQ = new PriorityQueue<>(new Comparator<NodeDistance>() {
            @Override
            public int compare(NodeDistance o1, NodeDistance o2) {
                return ((Integer) o1.distance).compareTo((Integer) o2.distance);
            }
        });
        // Hint: Use a Comparator or make NodeDistance implement Comparable

        // TODO: Add the starting node to the priority queue with distance 0
        NodeDistance first = new NodeDistance(0, 0);
        myPQ.add(first);

        // 2. MAIN LOOP
        // TODO: While the priority queue is not empty...
        while (myPQ.size() > 0) {

            // a. Poll the node with smallest distance from the priority queue
            NodeDistance cur = myPQ.poll();

            // b. If this node has already been visited, skip it (continue)
            // c. If this node is the endIndex, we've found the shortest path - stop.


            // d. Mark current node as visited

            // e. VITAL: Record the step for the visualizer!


            result.addStep(cur.nodeIndex, visited, distances, prev);
            if (visited.contains(cur.nodeIndex)) {
//                continue;
            } else {
                visited.add(cur.nodeIndex);
            }

            if (cur.nodeIndex == endIndex) {
               // result.addStep(cur.nodeIndex, visited, distances, prev);
                break;
            }



          List<Integer> neigh = graph.getNeighbors(cur.nodeIndex);
            for(int i = 0; i<neigh.size(); i++) {
                int distanceNew = cur.distance + graph.getEdgeWeight(cur.nodeIndex, neigh.get(i));
                if (distanceNew < distances[neigh.get(i)]) {
                   distances[neigh.get(i)] = distanceNew;
                   //this point maybe problem
                   prev[neigh.get(i)] = cur.nodeIndex;
                   myPQ.add(new NodeDistance(neigh.get(i), distanceNew));
                }
            }
        }



            // f. Iterate through all neighbors of the current node

            // For each neighbor:
            // - Calculate new distance: distances[current] + edge weight
            // - If new distance < distances[neighbor]:
            //     * Update distances[neighbor]
            //     * Update previous[neighbor] = current
            //     * Add neighbor to priority queue with new distance

            // 3. RECONSTRUCT PATH
            // TODO: Call helper method to get the path list
            // TODO: result.setFinalPath(path, distances[endIndex]);
        result.setFinalPath(reconstructPath(prev, startIndex, endIndex), distances[endIndex]);

        System.out.println(result.getFinalPath());
        System.out.println();
        for(Integer x: distances) {
            System.out.println(x);
        }
            return result;

    }

    /**
     * Helper method to reconstruct the path from start to end.
     *
     * @param previous   array where previous[i] is the node before i
     * @param startIndex the starting node index
     * @param endIndex   the ending node index
     * @return list of node indices representing the path from start to end
     */
    private List reconstructPath(int[] previous, int startIndex, int endIndex) {
        List path = new ArrayList<>();
        int pointer = endIndex;
        // TODO: Trace backwards from endIndex using the previous[] array
        while(pointer!=-1) {
            System.out.println("Current path: " + path);
            path.add(pointer);
            pointer = previous[pointer];
        }
        // TODO: Don't forget to reverse the list so it goes Start -> End!
        Collections.reverse(path);
        return path;
    }

    /**
     * Inner class to represent a node and its distance in the priority queue.
     * Students must implement Comparable to allow PriorityQueue ordering.
     */
    private static class NodeDistance implements Comparator<NodeDistance> {
        int nodeIndex;
        int distance;

        public NodeDistance(int nodeIndex, int distance) {
            this.nodeIndex = nodeIndex;
            this.distance = distance;
        }
        @Override
        public int compare(NodeDistance o1, NodeDistance o2) {
            Integer temp = o1.distance;
            Integer temp2 = o2.distance;
            return temp.compareTo(temp2);
        }

    }
}