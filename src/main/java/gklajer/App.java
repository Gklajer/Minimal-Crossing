package gklajer;

import java.awt.Canvas;
import java.awt.Color;
import java.awt.Frame;
import java.awt.Graphics;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Scanner;

import org.json.JSONArray;
import org.json.JSONObject;

public class App {
    static final int FRAME_SIZE = 900;

    static final int X_SHIFT = 15;
    static final int Y_SHIFT = 0;

    private Graph graph;

    private List<Position> points = new ArrayList<Position>();

    private AppDrawer appDrawer = new AppDrawer();

    public static void main(String[] args) {
        Scanner scanner = new Scanner(System.in);
        App app;
        do {
            app = new App();
            if (!app.startingRoutine(scanner))
                continue;
            app.randomLayoutRoutine(scanner);
            app.forceDirectedSpatializationRoutine(scanner);
            app.minimizeCrossingsRoutine(scanner);
            app.exportGraphDataRoutine(scanner);
        } while (Helpers.getUserInput(scanner, "Continue (y/n): ", Helpers.YES_NO_PATTERN).equalsIgnoreCase("y"));
        scanner.close();
    }

    public App() {
        graph = new Graph();
        points = new ArrayList<Position>();

        appDrawer = new AppDrawer();
    }

    public App(JSONObject graphData) {
        initWithGraphData(graphData);
    }

    private void initWithGraphData(JSONObject graphData) {
        for (Object point : graphData.getJSONArray("points"))
            addPoint((JSONObject) point);

        graph = new Graph(graphData, points);

        appDrawer.computeScaling();
    }

    private boolean startingRoutine(Scanner scanner) {
        Helpers.clearConsole();

        Integer instanceFilenum = Integer
                .valueOf(Helpers.getUserInput(scanner, "Enter instance file n°", "\\d+"));

        String instanceFilepath = Helpers.getInstanceFilepathFromNumber(instanceFilenum);

        JSONObject graphData = Helpers.getGraphDataFromInstanceFilepath(instanceFilepath);

        if (graphData == null) {
            System.out.println("!DATA NOT FOUND!");
            return false;
        }

        initWithGraphData(graphData);

        int numberOfCrossings = graph.countCrossings();
        System.out.printf("\n#crossings: %d\n", numberOfCrossings);

        appDrawer.draw();

        return true;
    }

    private void randomLayoutRoutine(Scanner scanner) {
        while (Helpers.getUserInput(scanner, "Apply random layout (y/n): ", Helpers.YES_NO_PATTERN)
                .equalsIgnoreCase("y")) {
            graph.randomLayout(points);

            int numberOfCrossings = graph.countCrossings();
            System.out.printf("\n#crossings: %d\n", numberOfCrossings);

            appDrawer.draw();
        }
    }

    private void forceDirectedSpatializationRoutine(Scanner scanner) {
        while (Helpers.getUserInput(scanner, "Apply force directed spatialization algo (y/n): ",
                Helpers.YES_NO_PATTERN).equalsIgnoreCase("y")) {
            Optional<AppDrawer> drawerOptional = Helpers.getUserInput(scanner,
                    "\t- Show algo execution - causes slow down - (y/n): ",
                    Helpers.YES_NO_PATTERN).equalsIgnoreCase("y") ? Optional.of(appDrawer) : Optional.empty();

            int limIter = Integer
                    .valueOf(Helpers.getUserInput(scanner, "\t- Limit the number of iterations (> 0): ", "\\d+"));

            boolean withClustering = Helpers.getUserInput(scanner,
                    "\t- Use clustering (y/n): ",
                    Helpers.YES_NO_PATTERN).equalsIgnoreCase("y");

            boolean withHierarchy = Helpers.getUserInput(scanner,
                    "\t- Use hierachical clustering (y/n): ",
                    Helpers.YES_NO_PATTERN).equalsIgnoreCase("y");

            CrossingOptimizer.forceDirectedSpatialization(graph, points, drawerOptional, limIter, withClustering,
                    withHierarchy);

            int numberOfCrossings = graph.countCrossings();
            System.out.printf("\n#crossings: %d\n", numberOfCrossings);

            appDrawer.draw();
        }
    }

    private void minimizeCrossingsRoutine(Scanner scanner) {

        while (Helpers.getUserInput(scanner, "Minimize crossing (y/n): ", Helpers.YES_NO_PATTERN)
                .equalsIgnoreCase("y")) {
            Optional<AppDrawer> drawerOptional = Helpers.getUserInput(scanner,
                    "\t- Show algo execution - causes slow down - (y/n): ",
                    Helpers.YES_NO_PATTERN).equalsIgnoreCase("y") ? Optional.of(appDrawer) : Optional.empty();

            String limIter = Helpers.getUserInput(scanner, "\t- Limit the number of iterations (> 0 or _): ", "\\d*");
            Optional<Integer> limIterOptional = (!limIter.isEmpty()) ? Optional.of(Integer.valueOf(limIter))
                    : Optional.empty();

            double temperature = Double
                    .valueOf(Helpers.getUserInput(scanner, "\t- temperature > 0 (ex: 100): ", Helpers.DOUBLE_PATTERN));
            double coolingRate = Double
                    .valueOf(Helpers.getUserInput(scanner, "\t- coolingRate (0,1) (ex: 0.96): ", Helpers.DOUBLE_PATTERN));

            int numberOfCrossings;
            CrossingOptimizer.minimizeCrossings(graph, points, drawerOptional, limIterOptional, temperature,
                    coolingRate);

            numberOfCrossings = graph.countCrossings();
            System.out.printf("\n#crossings reduced: %d✅\n", numberOfCrossings);

            appDrawer.draw();
        }
    }

    private void exportGraphDataRoutine(Scanner scanner) {
        if (Helpers.getUserInput(scanner, "Export graph data? (y/n): ", Helpers.YES_NO_PATTERN).equalsIgnoreCase("y")) {
            String filePath = Helpers.getUserInput(scanner, "Enter file path: ", Helpers.FILE_PATH_PATTERN);
            exportGraphData(filePath);
        }
    }

    private void exportGraphData(String filePath) {
        JSONObject graphData = createGraphData();
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(filePath))) {
            writer.write(graphData.toString());
            System.out.println("Graph data exported successfully.");
        } catch (IOException e) {
            e.printStackTrace();
            System.out.println("Failed to export graph data.");
        }
    }

    private JSONObject createGraphData() {
        JSONObject graphData = new JSONObject();
        JSONArray pointsArray = new JSONArray();
        JSONArray nodesArray = new JSONArray();
        JSONArray edgesArray = new JSONArray();

        for (Edge edge : graph.getEdges()) {
            JSONObject edgeObj = new JSONObject();
            edgeObj.put("source", edge.getSourceId());
            edgeObj.put("target", edge.getTargetId());
            edgesArray.put(edgeObj);
        }
        graphData.put("edges", edgesArray);

        for (Node node : graph.getNodes()) {
            JSONObject nodeObj = new JSONObject();
            nodeObj.put("id", node.getId());
            nodeObj.put("x", node.getX());
            nodeObj.put("y", node.getY());
            nodesArray.put(nodeObj);
        }
        graphData.put("nodes", nodesArray);

        for (Position point : points) {
            JSONObject pointObj = new JSONObject();
            pointObj.put("x", point.getX());
            pointObj.put("y", point.getY());
            pointsArray.put(pointObj);
        }

        graphData.put("points", pointsArray);

        return graphData;
    }

    private void addPoint(JSONObject point) {
        int x = point.getInt("x");
        int y = point.getInt("y");
        points.add(new Position(x, y));
    }

    public List<Position> getPoints() {
        return points;
    }

    public Graph getGraph() {
        return graph;
    }

    public AppDrawer getAppDrawer() {
        return appDrawer;
    }
    
    public void draw() {
        appDrawer.draw();
    }

    class AppDrawer extends Canvas {
        private static final int POINT_RAD = FRAME_SIZE / 240;
        private static final int POINT_DIAM = 2 * POINT_RAD;

        private static final int POINT_X_SHIFT = POINT_RAD + X_SHIFT;
        private static final int POINT_Y_SHIFT = POINT_RAD + Y_SHIFT;

        Frame frame = new Frame("Graph & Points Embedded");

        private double scaling;

        private void computeScaling() {
            int leftMostX = Integer.MAX_VALUE;
            int rightMostX = Integer.MIN_VALUE;
            int topMostY = Integer.MIN_VALUE;
            int bottomMostY = Integer.MAX_VALUE;

            for (Position point : points) {
                int x = point.getX();
                int y = point.getY();

                // Update leftmost and rightmost points
                if (x < leftMostX) {
                    leftMostX = x;
                }
                if (x > rightMostX) {
                    rightMostX = x;
                }

                // Update topmost and bottommost points
                if (y > topMostY) {
                    topMostY = y;
                }
                if (y < bottomMostY) {
                    bottomMostY = y;
                }
            }

            int xSpread = rightMostX - leftMostX;
            int ySpread = topMostY - bottomMostY;

            int maxSpread = Math.max(Math.max(xSpread, ySpread), 1);

            scaling = (double) ((FRAME_SIZE - 2 * (X_SHIFT + POINT_DIAM)) / maxSpread);
        }

        private void paintPoints(Graphics graphics) {
            graphics.setColor(Color.GRAY);
            for (Position point : points) {
                int x = Helpers.affineTransform(point.getX(), scaling, POINT_X_SHIFT);
                int y = Helpers.affineTransform(point.getY(), scaling, POINT_Y_SHIFT);
                graphics.fillOval(x, y, POINT_DIAM, POINT_DIAM);
            }
        }

        public void paint(Graphics graphics) {
            paintPoints(graphics);
            graph.paint(graphics, scaling);
        }

        void draw() {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            Frame oldFrame = frame;

            frame = new Frame("Crossing-Minimal Point-Set Embedding");

            frame.addWindowListener(new WindowAdapter() {
                @Override
                public void windowClosing(WindowEvent e) {
                    // Clean up resources or perform any necessary actions
                    // Close the frame
                    Helpers.clearConsole();
                    frame.dispose();
                    // Terminate the application
                    System.exit(0);
                }
            });
            
            frame.setSize(FRAME_SIZE, FRAME_SIZE);
            frame.add(this);

            frame.setVisible(true);

            oldFrame.setVisible(false);
            oldFrame.dispose();
        }
    }
}
