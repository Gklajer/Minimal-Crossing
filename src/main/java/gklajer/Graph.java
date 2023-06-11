package gklajer;

import java.awt.Color;
import java.awt.Graphics;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.json.JSONObject;

public class Graph {
    private List<Node> nodes;
    private List<Edge> edges;

    private GraphDrawer graphDrawer;

    public Graph(JSONObject graphData, List<Position> points) {
        nodes = new ArrayList<>();
        edges = new ArrayList<>();
        graphDrawer = new GraphDrawer();

        for (Object node : graphData.getJSONArray("nodes"))
            addNode(new Node((JSONObject) node));

        for (Object edge : graphData.getJSONArray("edges"))
            addEdge(new Edge((JSONObject) edge));
    }

    public Graph() {
        nodes = new ArrayList<>();
        edges = new ArrayList<>();
        graphDrawer = new GraphDrawer();
    }

    public void randomLayout(List<Position> positions) {
        List<Node> nodes = getNodes();
        Collections.shuffle(positions, Helpers.random);

        if (nodes.size() > positions.size()) {
            throw new IllegalArgumentException("Number of positions is less than the number of nodes.");
        }

        for (Node node : nodes) {
            Position position = positions.get(node.getId());
            node.setPosition(position);
        }
    }

    public boolean doEdgesCross(Edge edge1, Edge edge2) {
        // Positions of the nodes
        Position p1 = getSource(edge1).getPosition();
        Position p2 = getTarget(edge1).getPosition();
        Position p3 = getSource(edge2).getPosition();
        Position p4 = getTarget(edge2).getPosition();

        if (Helpers.sharingNodeNoOverlapping(edge1, edge2, p1, p2, p3, p4))
            return false; // (negative) edge case

        // Orientations of nodes triplet
        int orientation1 = Helpers.getOrientation(p1, p2, p3);
        int orientation2 = Helpers.getOrientation(p1, p2, p4);
        int orientation3 = Helpers.getOrientation(p3, p4, p1);
        int orientation4 = Helpers.getOrientation(p3, p4, p2);

        return (Helpers.orientationsIntersect(orientation1, orientation2, orientation3, orientation4) ||
                Helpers.isCollinearIntersection(orientation1, orientation2, orientation3, orientation4, p1, p2, p3,
                        p4)); // (positive) edge case
    }

    public int countCrossings() {
        int crossings = 0;

        List<Edge> edges = getEdges();
        int numEdges = edges.size();

        for (int i = 0; i < numEdges - 1; i++) {
            Edge edge1 = edges.get(i);
            for (int j = i + 1; j < numEdges; j++) {
                Edge edge2 = edges.get(j);
                if (doEdgesCross(edge1, edge2)) {
                    crossings++;
                }
            }
        }

        return crossings;
    }

    public List<Position> getOccupiedPositions() {
        List<Position> occupiedPoints = new ArrayList<Position>();

        for (Node node : nodes)
            occupiedPoints.add(node.getPosition());

        return occupiedPoints;
    }

    public Set<Node> getSetOfCoverNodes(Set<Edge> setOfEdges) {
        Set<Node> nodesCovered = new HashSet<Node>();

        for (Edge edge : setOfEdges) {
            nodesCovered.add(getSource(edge));
            nodesCovered.add(getTarget(edge));
        }

        return nodesCovered;
    }

    public Set<Edge> getSetOfCrossingEdges() {
        Set<Edge> crossingEdges = new HashSet<Edge>();
        for (Edge edge1 : edges) {
            if (crossingEdges.contains(edge1))
                continue;
            for (Edge edge2 : edges) {
                if (doEdgesCross(edge1, edge2)) {
                    crossingEdges.add(edge1);
                    crossingEdges.add(edge2);
                    break;
                }
            }
        }
        return crossingEdges;
    }

    public List<Node> getNeighbors(Node node) {
        List<Node> neighbors = new ArrayList<>();
        List<Edge> adjacentEdges = getEdges(node);
        for (Edge edge : adjacentEdges)
            neighbors.add((edge.getSourceId() == node.getId()) ? getTarget(edge) : getSource(edge));

        return neighbors;
    }

    void addNode(Node node) {
        nodes.add(node.getId(), node);
    }

    void addEdge(Edge edge) {
        edges.add(edge);
    }

    public void removeEdge(Edge edge) {
        edges.remove(edge);
    }

    public List<Edge> getEdges(Node node) {
        List<Edge> connectedEdges = new ArrayList<>();

        for (Edge edge : edges) {
            if (getSource(edge).equals(node) || getTarget(edge).equals(node)) {
                connectedEdges.add(edge);
            }
        }

        return connectedEdges;
    }

    public Node getNode(int id) {
        return nodes.get(id);
    }

    public List<Node> getNodes() {
        return nodes;
    }

    public List<Edge> getEdges() {
        return edges;
    }

    public Node getSource(Edge edge) {
        int sourceId = edge.getSourceId();
        return getNode(sourceId);
    }

    public Node getTarget(Edge edge) {
        int targetId = edge.getTargetId();
        return getNode(targetId);
    }

    public void paint(Graphics graphics, double scaling) {
        graphDrawer.paint(graphics, scaling);
    }

    public static int getNodeRad() {
        return GraphDrawer.NODE_RAD;
    }

    private class GraphDrawer {

        private static final int NODE_RAD = App.FRAME_SIZE / 120;

        private static final int NODE_DIAM = 2 * NODE_RAD;

        private static final int NODE_X_SHIFT = App.X_SHIFT;
        private static final int NODE_Y_SHIFT = App.Y_SHIFT;

        private static final int EDGE_X_SHIFT = NODE_X_SHIFT + NODE_RAD;
        private static final int EDGE_Y_SHIFT = NODE_Y_SHIFT + NODE_RAD;

        public void paint(Graphics graphics, double scaling) {
            paintNodes(graphics, scaling);
            paintEdges(graphics, scaling);
        }

        private void paintNodes(Graphics graphics, double scaling) {
            graphics.setColor(Color.BLACK);
            for (Node node : nodes) {
                int nodeX = Helpers.affineTransform(node.getX(), scaling, NODE_X_SHIFT);
                int nodeY = Helpers.affineTransform(node.getY(), scaling, NODE_Y_SHIFT);
                graphics.fillOval(nodeX, nodeY, NODE_DIAM, NODE_DIAM);
            }
        }

        private void paintEdges(Graphics graphics, double scaling) {
            for (Edge edge : edges) {
                Node source = getSource(edge);
                Node target = getTarget(edge);

                int sourceX = Helpers.affineTransform(source.getX(), scaling, EDGE_X_SHIFT);
                int sourceY = Helpers.affineTransform(source.getY(), scaling, EDGE_Y_SHIFT);

                int targetX = Helpers.affineTransform(target.getX(), scaling, EDGE_X_SHIFT);
                int targetY = Helpers.affineTransform(target.getY(), scaling, EDGE_Y_SHIFT);

                graphics.drawLine(sourceX, sourceY, targetX, targetY);
            }
        }
    }
}
