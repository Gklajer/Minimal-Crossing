package gklajer;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Scanner;
import java.util.Set;

import org.json.JSONObject;

public class Experiment {
    private static final String RESULTS_FILENAME = "experiments/experiment_results.txt";

    private static final int NUM_RUNS = 10;

    private JSONObject graphData;

    private App app = new App();

    private Set<Integer> filenums;

    private ExperimentConfig config;
    private ExperimentResults results = new ExperimentResults();

    public static void main(String[] args) {
        Helpers.clearConsole();
        Scanner scanner = new Scanner(System.in);

        List<String> experimentStrings = new ArrayList<String>();

        HashMap<ExperimentConfig, Set<Integer>> experimentsConfigsToInstances = makeExperimentsConfigs(scanner);
        for (HashMap.Entry<ExperimentConfig, Set<Integer>> experimentConfigToInstances : experimentsConfigsToInstances
                .entrySet()) {
            Experiment experiment = new Experiment(experimentConfigToInstances.getKey(), experimentConfigToInstances.getValue());
            for (int filenum : experiment.filenums) {
                experiment.setGraphDataFromFilenum(filenum);

                if (experiment.graphData == null) {
                    System.out.println("!DATA NOT FOUND!");
                    continue;
                }

                System.out.printf("\nEXPERIMENT ON INSTANCE N°%d\n", filenum);
                for (int i = 0; i < NUM_RUNS; i++) {
                    System.out.println(String.format("\n\t- EXPERIMENTAL SETUP RESET n°%d", i));
                    experiment.resetExperimentalSetup();

                    Graph graph = experiment.app.getGraph();
                    int oldNumCrossings = 0;
                    if (i == 0) {
                        oldNumCrossings = graph.countCrossings(); 
                        experiment.results.addOldNumCrossings(filenum, oldNumCrossings);

                        experiment.app.draw();
                    }

                    experiment.recordMemoryUsageAndTime(filenum, i);

                    if (i == 0) {
                        int newNumCrossings = graph.countCrossings(); 
                        experiment.results.addNewNumCrossings(filenum, newNumCrossings);

                        System.out.printf("\n\t- #crossings: %d -> %d\n", oldNumCrossings, newNumCrossings);  

                        experiment.app.draw();
                    }                    
                }
            }

            experimentStrings.add(experiment.toString());
            if (Helpers.getUserInput(scanner, "Continue (y/n): ", Helpers.YES_NO_PATTERN).equalsIgnoreCase("n"))
                break;
        }
        scanner.close();

        saveInTxtFile(experimentStrings);
    }

    private static void saveInTxtFile(List<String> experimentStrings) {
        try (FileWriter fileWriter = new FileWriter(RESULTS_FILENAME)) {
            fileWriter.write(experimentStrings.toString());
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public Experiment(ExperimentConfig config, Set<Integer> filenums) {
        Helpers.clearConsole();
        System.out.println("NEW EXPERIMENT");
        System.out.println(config.toString());

        this.config = config;
        this.filenums = filenums;
    }

    private void setGraphDataFromFilenum(Integer instanceFilenum) {
        String instanceFilepath = Helpers.getInstanceFilepathFromNumber(instanceFilenum);

        graphData = Helpers.getGraphDataFromInstanceFilepath(instanceFilepath);
    }

    private void resetExperimentalSetup() {
        Helpers.reseed();

        app = new App(graphData);

        Graph graph = app.getGraph();
        graph.randomLayout(app.getPoints());
    }

    private static HashMap<ExperimentConfig, Set<Integer>> makeExperimentsConfigs(Scanner scanner) {
        HashMap<ExperimentConfig, Set<Integer>> configs = new HashMap<ExperimentConfig, Set<Integer>>();
        while (Helpers.getUserInput(scanner, "Add a config (y/n): ", Helpers.YES_NO_PATTERN).equalsIgnoreCase("y")) {
            ExperimentConfig experimentConfig = new ExperimentConfig(scanner);

            boolean onAllInstances = Helpers
                    .getUserInput(scanner, "Apply to all instances - (y/n): ", Helpers.YES_NO_PATTERN)
                    .equalsIgnoreCase("y");

            Set<Integer> filenums = new HashSet<Integer>();

            for (int i = 1; (i <= 6) && onAllInstances; i++) {
                filenums.add(i);
            }

            boolean addAFile = true;
            while (addAFile && !onAllInstances) {
                filenums.add(Integer.valueOf(Helpers.getUserInput(scanner, "Apply to instance [1-6] n°", "[1-6]")));
                addAFile = Helpers.getUserInput(scanner, "Add an instance (y/n): ", Helpers.YES_NO_PATTERN)
                        .equalsIgnoreCase("y");
            }

            configs.put(experimentConfig, filenums);
        }

        Helpers.clearConsole();
        System.out.print(configs);
        return configs;
    }

    public void recordMemoryUsageAndTime(int filenum, int runNum) {
        // Perform garbage collection and finalization to clear memory from previous runs
        System.gc();
        System.runFinalization();
    
        Runtime runtime = Runtime.getRuntime();

        // Calculate memory usage and time for the current run
        long freeMemoryBefore = runtime.freeMemory();
        long startTime = System.nanoTime();
    
        // Execute the algorithm
        run();
    
        // Calculate memory usage and time after the algorithm execution
        long endTime = System.nanoTime();
        long freeMemoryAfter = runtime.freeMemory();

        double executionTime = (endTime - startTime) / 1e9;
        int memoryUsage = (int) (freeMemoryBefore - freeMemoryAfter);

        System.out.printf("\t- executionTime: %f ; memoryUsage: %d\n", executionTime, memoryUsage);

        results.addTime(filenum, executionTime);
        results.addMemory(filenum, memoryUsage);
    }

    private void run() {
        Graph graph = app.getGraph();
        List<Position> points = app.getPoints();

        Optional<App.AppDrawer> drawerOptional = (config.withDrawing) ? Optional.of(app.getAppDrawer()) : Optional.empty();
        if (config.withSpatialization)
            CrossingOptimizer.forceDirectedSpatialization(graph, points, drawerOptional, config.limIterSpatialization,
                    config.withClustering, config.withHierarchy);

        CrossingOptimizer.minimizeCrossings(graph, points, drawerOptional, Optional.of(config.limIterMinimization),
                config.temperatureMinimization, config.coolingRateMinimization);
    }

    @Override
    public String toString() {
        return "Experiment [config=" + config.toString() + ", results=" + results.toString() + "]";
    }

    private static class ExperimentConfig {
        private boolean withDrawing = false;

        private boolean withSpatialization = false;
        private boolean withClustering = false;
        private boolean withHierarchy = false;
        private int limIterSpatialization;

        private int limIterMinimization;
        private double temperatureMinimization;
        private double coolingRateMinimization;

        public ExperimentConfig(Scanner scanner) {
            withDrawing = Helpers.getUserInput(scanner,
                    "Show algos execution - causes slow down - (y/n): ",
                    Helpers.YES_NO_PATTERN).equalsIgnoreCase("y");

            withSpatialization = Helpers.getUserInput(scanner, "Apply force directed spatialization algo (y/n): ",
                    Helpers.YES_NO_PATTERN).equalsIgnoreCase("y");

            if (withSpatialization) {
                limIterSpatialization = Integer
                        .valueOf(Helpers.getUserInput(scanner, "\t- Limit the number of iterations (> 0): ", "\\d+"));

                withClustering = Helpers.getUserInput(scanner,
                        "\t- Use clustering (y/n): ",
                        Helpers.YES_NO_PATTERN).equalsIgnoreCase("y");

                withHierarchy = Helpers.getUserInput(scanner,
                        "\t- Use hierachical clustering (y/n): ",
                        Helpers.YES_NO_PATTERN).equalsIgnoreCase("y");
            }

            System.out.println("\nMinimisation settings");
            limIterMinimization = Integer
                    .valueOf(Helpers.getUserInput(scanner, "\t- Limit the number of iterations (> 0): ", "\\d+"));

            temperatureMinimization = Double
                    .valueOf(Helpers.getUserInput(scanner, "\t- temperature > 0 (ex: 100): ", Helpers.DOUBLE_PATTERN));

            coolingRateMinimization = Double
                    .valueOf(Helpers.getUserInput(scanner, "\t- coolingRate (0,1) (ex: 0.96): ",
                            Helpers.DOUBLE_PATTERN));
        }

        @Override
        public String toString() {
            return "ExperimentConfig {'withDrawing':" + Helpers.booleanToPythonRepr(withDrawing) + ", 'withSpatialization':" + Helpers.booleanToPythonRepr(withSpatialization)
                    + ", 'withClustering':" + Helpers.booleanToPythonRepr(withClustering) + ", 'withHierarchy':" + Helpers.booleanToPythonRepr(withHierarchy)
                    + ", 'limIterSpatialization':" + limIterSpatialization + ", 'limIterMinimization':"
                    + limIterMinimization + ", 'temperatureMinmization':" + temperatureMinimization
                    + ", 'coolingRateMinimization':" + coolingRateMinimization + "}";
        }

        @Override
        public int hashCode() {
            final int prime = 31;
            int result = 1;
            result = prime * result + (withDrawing ? 1231 : 1237);
            result = prime * result + (withSpatialization ? 1231 : 1237);
            result = prime * result + (withClustering ? 1231 : 1237);
            result = prime * result + (withHierarchy ? 1231 : 1237);
            result = prime * result + limIterSpatialization;
            result = prime * result + limIterMinimization;
            long temp;
            temp = Double.doubleToLongBits(temperatureMinimization);
            result = prime * result + (int) (temp ^ (temp >>> 32));
            temp = Double.doubleToLongBits(coolingRateMinimization);
            result = prime * result + (int) (temp ^ (temp >>> 32));
            return result;
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj)
                return true;
            if (obj == null)
                return false;
            if (getClass() != obj.getClass())
                return false;
            ExperimentConfig other = (ExperimentConfig) obj;
            if (withDrawing != other.withDrawing)
                return false;
            if (withSpatialization != other.withSpatialization)
                return false;
            if (withClustering != other.withClustering)
                return false;
            if (withHierarchy != other.withHierarchy)
                return false;
            if (limIterSpatialization != other.limIterSpatialization)
                return false;
            if (limIterMinimization != other.limIterMinimization)
                return false;
            if (Double.doubleToLongBits(temperatureMinimization) != Double
                    .doubleToLongBits(other.temperatureMinimization))
                return false;
            if (Double.doubleToLongBits(coolingRateMinimization) != Double
                    .doubleToLongBits(other.coolingRateMinimization))
                return false;
            return true;
        }
    }
    
    private static class ExperimentResults {
        private HashMap<Integer, List<Double>> computationTimesInSec = new HashMap<Integer, List<Double>>();
        private HashMap<Integer, List<Integer>> computationMemoriesInBytes = new HashMap<Integer, List<Integer>>();
        private HashMap<Integer, Integer> oldNumsCrossings = new HashMap<Integer, Integer>();
        private HashMap<Integer, Integer> newNumsCrossings = new HashMap<Integer, Integer>();

        private void addTime(int filenum, double timeInSec) {
            List<Double> computationTimesInSecForInstance = computationTimesInSec.getOrDefault(filenum,
                    new ArrayList<Double>());
            computationTimesInSecForInstance.add(timeInSec);
            computationTimesInSec.put(filenum, computationTimesInSecForInstance);
        }

        private void addMemory(int filenum, int memoryInBytes) {
            List<Integer> computationMemoriesInBytesForInstance = computationMemoriesInBytes.getOrDefault(filenum,
                    new ArrayList<Integer>());
            computationMemoriesInBytesForInstance.add(memoryInBytes);
            computationMemoriesInBytes.put(filenum, computationMemoriesInBytesForInstance);
        }

        private void addOldNumCrossings(int filenum, int numCrossings) {
            oldNumsCrossings.put(filenum, numCrossings);
        }

        private void addNewNumCrossings(int filenum, int numCrossings) {
            newNumsCrossings.put(filenum, numCrossings);
        }

        @Override
        public String toString() {
            return "ExperimentResults {'computationTimesInSec':" + computationTimesInSec + ", 'computationMemoriesInBytes':"
                    + computationMemoriesInBytes + ", 'oldNumsCrossings':" + oldNumsCrossings + ", 'newNumsCrossings':"
                    + newNumsCrossings + "}";
        }
    }
}
