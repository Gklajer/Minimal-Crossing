package gklajer;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.Random;
import java.util.Scanner;

import org.json.JSONObject;

public class Helpers {
    public static final String YES_NO_PATTERN = "[yYnN]";
    public static final String DOUBLE_PATTERN = "[+]?\\d+(\\.\\d+)?";
    public static final String FILE_PATH_PATTERN = ".*[/\\\\]?.+\\.txt";

    public static final String INSTANCE_FILNAME_FORMAT = "test-%d.txt";
    public static final String INSTANCES_DIRPATH = "/Users/gklajer/Desktop/minimal-crossing/src/main/instances/";

    public static final int SEED = 0;
    public static Random random = new Random(SEED);

    public static void reseed() {
        random = new Random(SEED);
    }

    public static boolean sharingNodeNoOverlapping(Edge edge1, Edge edge2, Position p1, Position p2, Position p3,
            Position p4) {

        int sourceId1 = edge1.getSourceId();
        int targetId1 = edge1.getTargetId();
        int sourceId2 = edge2.getSourceId();
        int targetId2 = edge2.getTargetId();

        // No shared node
        if (!(sourceId1 == sourceId2 || sourceId1 == targetId2 || targetId1 == sourceId2 || targetId1 == targetId2))
            return false;

        // Otherwise (shared node)

        // Calculate the direction vectors of the edges
        double dir1X = p2.getX() - p1.getX();
        double dir1Y = p2.getY() - p1.getY();
        double dir2X = p4.getX() - p3.getX();
        double dir2Y = p4.getY() - p3.getY();

        // To check colinearity and direction
        double crossProduct = dir1X * dir2Y - dir1Y * dir2X;
        double dotProduct = dir1X * dir2X + dir1Y * dir2Y;

        int reversedDirections = ((sourceId1 == targetId2) || (targetId1 == sourceId2)) ? -1 : +1; // reverse direction

        return !(Math.abs(crossProduct) < 1e-10 && (reversedDirections * dotProduct) > 0); // is not colinear with same
                                                                                           // direction?
    }

    public static boolean orientationsIntersect(int orientation1, int orientation2, int orientation3,
            int orientation4) {
        return orientation1 != orientation2 && orientation3 != orientation4;
    }

    public static boolean isCollinearIntersection(int orientation1, int orientation2, int orientation3,
            int orientation4, Position p1, Position p2, Position p3, Position p4) {
        return (isCollinearIntersection(orientation1, p1, p2, p3)) ||
                (isCollinearIntersection(orientation2, p1, p2, p4)) ||
                (isCollinearIntersection(orientation3, p3, p4, p1)) ||
                (isCollinearIntersection(orientation4, p3, p4, p2));
    }

    public static boolean isCollinearIntersection(int orientation, Position p1, Position p2, Position p) {
        return orientation == 0 && Helpers.onSegment(p1, p2, p);
    }

    public static boolean onSegment(Position p1, Position p2, Position p3) {
        if (p3.getX() >= Math.min(p1.getX(), p2.getX()) && p3.getX() <= Math.max(p1.getX(), p2.getX()) &&
                p3.getY() >= Math.min(p1.getY(), p2.getY()) && p3.getY() <= Math.max(p1.getY(), p2.getY())) {
            return true;
        }

        return false;
    }

    public static int getOrientation(Position p1, Position p2, Position p3) {
        int val = (p2.getY() - p1.getY()) * (p3.getX() - p2.getX()) - (p2.getX() - p1.getX()) * (p3.getY() - p2.getY());

        if (val == 0) {
            return 0; // Collinear
        } else if (val > 0) {
            return +1; // Clockwise
        } else {
            return -1; // Counterclockwise
        }
    }

    public static int affineTransform(int z, double scaling, int shift) {
        return (int) (scaling * z) + shift;
    }

    public static double calculateDistance(Node node1, Node node2) {
        return calculateDistance(node1.getPosition(), node2.getPosition());
    }

    public static double calculateDistance(Node node, Position point) {
        return calculateDistance(node.getPosition(), point);
    }

    public static double calculateDistance(Position point1, Position point2) {
        double dx = point1.getX() - point2.getX();
        double dy = point1.getY() - point2.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    public static String getUserInput(Scanner scanner, String inputMessage, String inputPatternToMatch) {
        String userInput;

        do {
            System.out.print("\n" + inputMessage);
            userInput = scanner.nextLine();
        } while (!userInput.matches(inputPatternToMatch));

        return userInput;
    }

    public static void clearConsole() {
        // Print enough newline characters to "clear" the console
        System.out.print("\033[H\033[2J");
        System.out.flush();
    }

    public static JSONObject getGraphDataFromInstanceFilepath(String instanceFilepath) {
        try {
            BufferedReader reader = new BufferedReader(new FileReader(instanceFilepath));
            StringBuilder sb = new StringBuilder();
            String line;
            while ((line = reader.readLine()) != null) {
                sb.append(line);
            }
            reader.close();
            return new JSONObject(sb.toString());
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        }
    }

    public static String getInstanceFilepathFromNumber(Integer instanceFilenum) {
        String instanceFilename = String.format(INSTANCE_FILNAME_FORMAT, instanceFilenum);
        return INSTANCES_DIRPATH + instanceFilename;
    }

    public static String booleanToPythonRepr(boolean bool) {
        String text = String.valueOf(bool);
        return (text.substring(0, 1).toUpperCase() + text.substring(1));
    }
}