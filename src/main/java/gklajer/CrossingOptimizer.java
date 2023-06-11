package gklajer;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Queue;
import java.util.Set;
import java.util.Stack;

public class CrossingOptimizer {
    public static void minimizeCrossings(Graph graph, List<Position> points, Optional<App.AppDrawer> appDrawer,
            Optional<Integer> numIterations) {
        minimizeCrossings(graph, points, appDrawer, numIterations, 1, 1);
    }

    public static void minimizeCrossings(Graph graph, List<Position> points, Optional<App.AppDrawer> appDrawer,
            Optional<Integer> numIterations, double initialTemperature, double coolingRate) {
        int iterations = numIterations.orElse(Integer.MAX_VALUE);
        double temperature = initialTemperature;

        int numCrossings = graph.countCrossings();
        int bestnumCrossings = numCrossings;

        List<Position> bestConfiguration = new ArrayList<>(graph.getNodes().size());
        for (Node node : graph.getNodes())
            bestConfiguration.add(node.getId(), new Position(node.getX(), node.getY()));

        List<Node> nodes = graph.getNodes();
        Set<Edge> crossingEdges = graph.getSetOfCrossingEdges();

        Boolean nodeChanged;
        do {
            nodeChanged = false;

            for (Node node : graph.getSetOfCoverNodes(crossingEdges)) {
                Position nodePosition = node.getPosition();

                // Move node around
                List<Position> unoccupiedPositions = new ArrayList<>(points);
                unoccupiedPositions.removeAll(graph.getOccupiedPositions());
                for (Position newPosition : unoccupiedPositions) {
                    // Try new position
                    node.setPosition(newPosition);

                    int newCrossings = graph.countCrossings();

                    if (Helpers.random.nextFloat() > acceptanceProbability(numCrossings, newCrossings, temperature)) { // Revert the
                                                                                                          // position
                                                                                                          // and
                                                                                                          // continue
                        node.setPosition(nodePosition);
                        continue;
                    }

                    // Keep the position
                    appDrawer.ifPresent(drawer -> drawer.draw());

                    numCrossings = newCrossings;

                    if (numCrossings == 0)
                        return; // no more crossing so stop the algo

                    if (numCrossings < bestnumCrossings) { // save the very best nodes configuration so it can be
                                                           // restore
                        saveNodesConfiguration(bestConfiguration, nodes);
                        bestnumCrossings = numCrossings;
                    }

                    if (iterations-- < 0) { // no more iterations so put the nodes in the best configuration then stop
                        setNodesConfiguration(bestConfiguration, nodes);
                        return;

                    }

                    // Update the variables
                    nodeChanged = true;

                    nodePosition = newPosition;

                    crossingEdges = graph.getSetOfCrossingEdges();

                    temperature = coolTemperature(coolingRate, temperature);
                }

                // Swap node with other nodes
                for (Node otherNode : nodes) {
                    if (otherNode.equals(node))
                        continue;

                    Position otherPosition = otherNode.getPosition();

                    // Swap positions
                    node.setPosition(otherPosition);
                    otherNode.setPosition(nodePosition);

                    int newCrossings = graph.countCrossings();

                    if (Helpers.random.nextFloat() > acceptanceProbability(numCrossings, newCrossings, temperature)) { // Revert the
                                                                                                          // position
                                                                                                          // swap
                        node.setPosition(nodePosition);
                        otherNode.setPosition(otherPosition);
                        continue;
                    }

                    // Keep the position
                    appDrawer.ifPresent(drawer -> drawer.draw());

                    numCrossings = newCrossings;

                    if (numCrossings == 0)
                        return; // no more crossing so stop the algo

                    if (numCrossings < bestnumCrossings) { // save the very best nodes configuration so it can be
                                                           // restore
                        saveNodesConfiguration(bestConfiguration, nodes);
                        bestnumCrossings = numCrossings;
                    }

                    if (iterations-- < 0) { // no more iterations so put the nodes in the best configuration then stop
                        setNodesConfiguration(bestConfiguration, nodes);
                        return;

                    }

                    // Update the variables
                    nodeChanged = true;

                    nodePosition = otherPosition;

                    crossingEdges = graph.getSetOfCrossingEdges();

                    temperature = coolTemperature(coolingRate, temperature);
                }
            }
        } while (nodeChanged);

        // Set the graph to the best configuration
        setNodesConfiguration(bestConfiguration, nodes);
    }

    private static double coolTemperature(double coolingRate, double temperature) {
        temperature *= coolingRate;
        return temperature;
    }

    private static void setNodesConfiguration(List<Position> configuration, List<Node> nodes) {
        for (Node node : nodes) {
            Position bestPosition = configuration.get(node.getId());
            node.setPosition(bestPosition);
        }
    }

    private static void saveNodesConfiguration(List<Position> configuration, List<Node> nodes) {
        for (Node node : nodes) {
            int nodeId = node.getId();
            Position currentPosition = node.getPosition();
            Position bestPosition = configuration.get(nodeId);
            bestPosition.setX(currentPosition.getX());
            bestPosition.setY(currentPosition.getY());
        }
    }

    private static double acceptanceProbability(int currentCrossings, int newCrossings, double temperature) {
        double delta = newCrossings - currentCrossings;
        return Math.min(Math.exp(-(1 + delta) / temperature), 1);
    }

    public static HashMap<Integer, Integer> labelPropagationClustering(Graph graph) {
        HashMap<Integer, Integer> labels = new HashMap<>();
        List<Node> nodes = graph.getNodes();
        int maxIterations = 1000;
        for (Node node : nodes) {
            labels.put(node.getId(), node.getId()); // Assign a unique label to each node
        }

        boolean changed;
        HashMap<Integer, Integer> newLabels = new HashMap<>(labels);
        do {
            changed = false;
            for (Node node : nodes) {
                maxIterations--;
                int currentLabel = labels.get(node.getId());
                HashMap<Integer, Integer> labelCounts = new HashMap<>();

                // Count the occurrences of neighboring labels
                List<Node> neighbors = graph.getNeighbors(node);
                for (Node neighbor : neighbors) {
                    int neighborLabel = labels.get(neighbor.getId());
                    labelCounts.put(neighborLabel, labelCounts.getOrDefault(neighborLabel, 0) + 1);

                }

                // Find the label with the maximum occurrence
                int maxCount = 0;
                List<Integer> maxLabels = new ArrayList<>();
                for (int label : labelCounts.keySet()) {
                    int count = labelCounts.get(label);

                    if (count < maxCount)
                        continue;

                    if (count > maxCount) {
                        maxCount = count;
                        maxLabels.clear();
                    }

                    maxLabels.add(label);
                }

                // Update the node's label
                if (maxLabels.contains(currentLabel))
                    continue;
                Collections.shuffle(maxLabels, Helpers.random);
                int maxLabel = maxLabels.get(0);

                newLabels.put(node.getId(), maxLabel);
                changed = true;
            }
            labels = newLabels;
        } while (changed && maxIterations > 0);

        return labels;
    }

    public static void forceDirectedSpatialization(Graph graph, List<Position> points,
            Optional<App.AppDrawer> appDrawer, Integer numIterations, boolean withClustering, boolean withHierarchy) {
        double initialTemperature = Math.sqrt(points.size()) / 6;
        double temperature = initialTemperature;
        double coolingRate = 0.95;

        List<Node> nodes = graph.getNodes();

        double k = Math.sqrt(points.size() / nodes.size());

        List<HashMap<Integer, Integer>> clustersLabels;
        List<HashMap<Integer, Set<Integer>>> clusters = new ArrayList<HashMap<Integer, Set<Integer>>>();
        List<HashMap<Integer, Integer>> clusterSizes = new ArrayList<HashMap<Integer, Integer>>();
        List<HashMap<Integer, Position>> clustersCentroidsPositions = new ArrayList<HashMap<Integer, Position>>();
        if (withClustering) {
            clustersLabels = (withHierarchy) ? edgeBetweennessClustering(graph)
                    : Collections.singletonList(labelPropagationClustering(graph));

            int level = 0;
            for (HashMap<Integer, Integer> clustersLabelsLevel : clustersLabels) {
                if (level++ % 10 != 0) continue;

                clusters.add(getClusterNodes(clustersLabelsLevel));

                HashMap<Integer, Integer> clusterSizesLevel = calculateClusterSizes(clustersLabelsLevel);
                clusterSizes.add(clusterSizesLevel);

                clustersCentroidsPositions.add(positionClustersCentroidsInGrid(points, clusterSizesLevel));
            }
        }

        while (numIterations-- > 0) {
            if (withClustering && clusters.get(0).size() > 1) {
                for (int level = 0; level < clusters.size(); level++) {
                    computeAttractiveForcesToClustersCentroids(graph, clusters.get(level), clustersCentroidsPositions.get(level),
                            k);
                }
            }
            else {    
                computeAttractiveForces(graph, k);
                computeRepulsiveForces(nodes, k);
            }

            updatePositions(graph, points, temperature);

            temperature = (temperature > initialTemperature / 2)? coolTemperature(coolingRate, temperature) : initialTemperature;

            appDrawer.ifPresent(drawer -> drawer.draw());
        }
    }

    private static void computeAttractiveForces(Graph graph, double k) {
        List<Edge> edges = graph.getEdges();
        for (Edge edge : edges) {
            Node sourceNode = graph.getSource(edge);
            Node targetNode = graph.getTarget(edge);

            double dx = targetNode.getX() - sourceNode.getX();
            double dy = targetNode.getY() - sourceNode.getY();

            double distance = Math.sqrt(dx * dx + dy * dy);

            double force = distance * distance / k;

            double displacementX = (dx / distance) * force;
            double displacementY = (dy / distance) * force;

            sourceNode.displaceX += displacementX;
            sourceNode.displaceY += displacementY;
            targetNode.displaceX -= displacementX;
            targetNode.displaceY -= displacementY;
        }
    }

    private static void computeRepulsiveForces(List<Node> nodes, double k) {
        for (Node node : nodes) {
            node.displaceX = 0;
            node.displaceY = 0;

            for (Node otherNode : nodes) {
                if (otherNode.equals(node))
                    continue;

                double dx = otherNode.getX() - node.getX();
                double dy = otherNode.getY() - node.getY();

                double distance = Math.sqrt(dx * dx + dy * dy);

                double force = k * k / distance;

                node.displaceX -= (dx / distance) * force;
                node.displaceY -= (dy / distance) * force;
            }
        }
    }

    private static HashMap<Integer, Position> positionClustersCentroidsInGrid(List<Position> points, HashMap<Integer, Integer> clusterSizes) {
    HashMap<Integer, Position> clustersCentroidsPositions = new HashMap<>();

        int totalCentroids = clusterSizes.size();
        int totalPoints = points.size();
        int maxCellSize = totalPoints / totalCentroids;
        int clusterGridWidth = (int) Math.ceil(Math.sqrt(totalCentroids));
        
        int maxClusterSize = getMaxClusterSize(clusterSizes);

        int centroidIndex = 0;
        for (HashMap.Entry<Integer, Integer> entry : clusterSizes.entrySet()) {
            int centroidLabel = entry.getKey();
            int clusterSize = entry.getValue();

            int cellSize = calculateCellSize(clusterSize, maxClusterSize, maxCellSize);

            int row = centroidIndex / clusterGridWidth;
            int col = centroidIndex % clusterGridWidth;

            int centroidX = col * maxCellSize + cellSize / 2;
            int centroidY = row * maxCellSize + cellSize / 2;

            Position centroidPosition = new Position(centroidX, centroidY);
            clustersCentroidsPositions.put(centroidLabel, centroidPosition);

            centroidIndex++;
        }

        return clustersCentroidsPositions;
    }

    private static int getMaxClusterSize(HashMap<Integer, Integer> clusterSizes) {
        int maxClusterSize = Integer.MIN_VALUE;
        for (int size : clusterSizes.values()) {
            maxClusterSize = Math.max(maxClusterSize, size);
        }
        return maxClusterSize;
    }

    private static int calculateCellSize(int clusterSize, int maxClusterSize, int maxCellSize) {
        int minCellSize = clusterSize; // Minimum desired cell size

        double scaleFactor = (double) clusterSize / maxClusterSize;
        int cellSize = (int) (minCellSize + scaleFactor * (maxCellSize - minCellSize));

        return cellSize;
    }

    private static void computeAttractiveForcesToClustersCentroids(Graph graph,
            HashMap<Integer, Set<Integer>> clusters, HashMap<Integer, Position> clustersCentroidsPositions, double k) {
        for (HashMap.Entry<Integer, Set<Integer>> entry : clusters.entrySet()) {
            int clusterLabel = entry.getKey();
            Set<Integer> cluster = entry.getValue();

            Position centroidPosition = clustersCentroidsPositions.get(clusterLabel);
            for (Integer nodeId : cluster) {
                Node node = graph.getNode(nodeId);

                double dx = centroidPosition.getX() - node.getX();
                double dy = centroidPosition.getY() - node.getY();
                
                double distance = Math.sqrt(dx * dx + dy * dy);
                
                double force = distance * distance / k;

                
                double displacementX = (dx / distance) * force;
                double displacementY = (dy / distance) * force;

                node.displaceX += displacementX;
                node.displaceY += displacementY;
            }
        }
    }

    private static HashMap<Integer, Set<Integer>> getClusterNodes(HashMap<Integer, Integer> clustersLabels) {
        HashMap<Integer, Set<Integer>> cluster = new HashMap<Integer, Set<Integer>>();

        for (HashMap.Entry<Integer, Integer> entry : clustersLabels.entrySet()) {
            int nodeId = entry.getKey();
            int label = entry.getValue();

            Set<Integer> clusterNodes = cluster.getOrDefault(label, new HashSet<>());

            clusterNodes.add(nodeId);

            cluster.put(label, clusterNodes);
        }
        return cluster;
    }

    private static HashMap<Integer, Integer> calculateClusterSizes(HashMap<Integer, Integer> clustersLabels) {
        HashMap<Integer, Integer> clusterSizes = new HashMap<>();
        for (int clusterLabel : clustersLabels.values()) {
            clusterSizes.put(clusterLabel, clusterSizes.getOrDefault(clusterLabel, 0) + 1);
        }
        return clusterSizes;
    }

    private static void updatePositions(Graph graph, List<Position> points, double temperature) {
        List<Node> nodes = graph.getNodes();
        for (Node node : nodes) {
            double displacementMagnitude = Math.sqrt(node.displaceX * node.displaceX +
                    node.displaceY * node.displaceY);

            double alpha = Math.min(1, temperature / displacementMagnitude);

            node.displaceX *= alpha;
            node.displaceY *= alpha;
            node.displace();

            Position closestPoint = findClosestUnoccupiedPoint(graph, node, points);
            node.setPosition(closestPoint);
        }
    }

    private static Position findClosestUnoccupiedPoint(Graph graph, Node node, List<Position> points) {
        Position closestPoint = null;
        double closestDistance = Double.POSITIVE_INFINITY;

        List<Position> unoccupiedPositions = new ArrayList<>(points);
        unoccupiedPositions.removeAll(graph.getOccupiedPositions());
        for (Position point : unoccupiedPositions) {
            double currentDistance = Helpers.calculateDistance(node, point);

            if (currentDistance < closestDistance) {
                closestPoint = point;
                closestDistance = currentDistance;
            }
        }

        return closestPoint;
    }

    public static List<HashMap<Integer, Integer>> edgeBetweennessClustering(Graph graph) {
        List<HashMap<Integer, Integer>> labels = new ArrayList<HashMap<Integer, Integer>>();
        // Create a copy of the graph
        Graph copyGraph = new Graph();
        List<Node> nodes = graph.getNodes();
        for (Node node : nodes) {
            copyGraph.addNode(new Node(node.getId(), new Position(node.getX(), node.getY())));
        }

        List<Edge> edges = graph.getEdges();
        for (Edge edge : edges) {
            copyGraph.addEdge(new Edge(edge.getSourceId(), edge.getTargetId()));
        }

        nodes = copyGraph.getNodes();

        // Calculate edge betweenness
        HashMap<Edge, Double> edgeBetweenness;

        // Perform hierarchical clustering based on edge betweenness
        int levelsNum = copyGraph.getEdges().size();
        for (int level = 0; level < levelsNum; level++) {
            HashMap<Integer, Integer> labelsLevel = new HashMap<Integer, Integer>();

            // Assign labels to the nodes based on the connected components
            int currentLabel = 0;
            for (Node node : nodes) {
                if (labelsLevel.containsKey(node.getId()))
                    continue;

                assignLabelToConnectedComponent(node, currentLabel, copyGraph, labelsLevel);
                currentLabel++;
            }

            labels.add(level, labelsLevel);

            edgeBetweenness = calculateEdgeBetweenness(copyGraph);
            Edge edgeToRemove = findEdgeWithMaxBetweenness(edgeBetweenness);
            copyGraph.removeEdge(edgeToRemove);
        }

        return labels;
    }

    private static HashMap<Edge, Double> calculateEdgeBetweenness(Graph graph) {
        HashMap<Edge, Double> edgeBetweenness = new HashMap<>();

        for (Node node : graph.getNodes()) {
            HashMap<Node, Integer> shortestPathCounts = new HashMap<>();
            HashMap<Node, List<List<Node>>> shortestPaths = new HashMap<>();
            calculateShortestPaths(graph, node, shortestPathCounts, shortestPaths);

            for (Node source : shortestPaths.keySet()) {
                for (Node target : shortestPaths.keySet()) {
                    if (source.equals(target))
                        continue;

                    double shortestPathCount = shortestPathCounts.get(target);
                    List<List<Node>> paths = shortestPaths.get(target);

                    double edgeContrib = calculateEdgeContribution(source, target, paths, shortestPathCount);
                    Edge edge = new Edge(source.getId(), target.getId());

                    if (edgeBetweenness.containsKey(edge)) {
                        edgeBetweenness.put(edge, edgeBetweenness.get(edge) + edgeContrib);
                    } else {
                        edgeBetweenness.put(edge, edgeContrib);
                    }

                }
            }
        }

        return edgeBetweenness;
    }

    private static void calculateShortestPaths(Graph graph, Node startNode, HashMap<Node, Integer> shortestPathCounts,
            HashMap<Node, List<List<Node>>> shortestPaths) {

        Queue<Node> queue = new LinkedList<>();
        Set<Node> visited = new HashSet<>();

        queue.add(startNode);
        visited.add(startNode);
        shortestPathCounts.put(startNode, 0);

        while (!queue.isEmpty()) {
            Node currentNode = queue.poll();
            List<Node> currentPath = new ArrayList<>();
            int currentDistance = shortestPathCounts.get(currentNode);

            if (!shortestPaths.containsKey(currentNode)) {
                shortestPaths.put(currentNode, new ArrayList<>());
            }

            shortestPaths.get(currentNode).add(new ArrayList<>(currentPath));

            List<Node> neighbors = graph.getNeighbors(currentNode);

            for (Node neighbor : neighbors) {
                if (!visited.contains(neighbor)) {
                    visited.add(neighbor);
                    queue.add(neighbor);
                    shortestPathCounts.put(neighbor, currentDistance + 1);
                    currentPath.add(neighbor);
                }
            }
        }
    }

    private static double calculateEdgeContribution(Node source, Node target, List<List<Node>> paths,
            double shortestPathCount) {
        double edgeContrib = 0.0;

        for (List<Node> path : paths) {
            if (!(path.contains(source) && path.contains(target)))
                continue;

            int sourceIndex = path.indexOf(source);
            int targetIndex = path.indexOf(target);

            if (Math.abs(sourceIndex - targetIndex) == 1)
                edgeContrib += (shortestPathCount / paths.size());

        }

        return edgeContrib;
    }

    private static Edge findEdgeWithMaxBetweenness(HashMap<Edge, Double> edgeBetweenness) {
        Edge maxEdge = null;
        double maxBetweenness = Double.MIN_VALUE;

        for (Edge edge : edgeBetweenness.keySet()) {
            double betweenness = edgeBetweenness.get(edge);
            if (betweenness > maxBetweenness) {
                maxBetweenness = betweenness;
                maxEdge = edge;
            }
        }

        return maxEdge;
    }

    private static void assignLabelToConnectedComponent(Node startNode, int label, Graph graph,
            HashMap<Integer, Integer> labels) {
        Stack<Node> stack = new Stack<>();
        stack.push(startNode);

        while (!stack.isEmpty()) {
            Node node = stack.pop();

            if (labels.containsKey(node.getId()))
                continue;

            labels.put(node.getId(), label);

            List<Node> neighbors = graph.getNeighbors(node);
            for (Node neighbor : neighbors) {
                if (labels.containsKey(neighbor.getId()))
                    continue;

                stack.push(neighbor);
            }
        }
    }
}
