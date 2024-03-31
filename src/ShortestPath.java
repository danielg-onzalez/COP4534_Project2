import javax.swing.*;
import java.awt.*;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.List;
import java.util.Scanner;
import java.util.*;

class Graph {
    private int V;
    public int[][] graph;

    public Graph(int V) {
        this.V = V;
        graph = new int[V][V];
    }

    public void addEdge(int u, int v, int weight) {
        graph[u][v] = weight;
        graph[v][u] = weight;
    }

    public List<Integer> dijkstra(int src, int dest) {
        int[] minDist = new int[V];
        Arrays.fill(minDist, Integer.MAX_VALUE);
        minDist[src] = 0;

        boolean[] visited = new boolean[V];
        int[] prev = new int[V];
        Arrays.fill(prev, -1);

        PriorityQueue<int[]> pq = new PriorityQueue<>(Comparator.comparingInt(a -> a[1]));
        pq.offer(new int[]{src, 0});

        while (!pq.isEmpty()) {
            int[] pair = pq.poll();
            int u = pair[0];
            if (visited[u]) continue;
            visited[u] = true;

            if (u == dest) break;

            for (int v = 0; v < V; v++) {
                if (graph[u][v] > 0 && !visited[v]) {
                    int newDist = minDist[u] + graph[u][v];
                    if (newDist < minDist[v]) {
                        minDist[v] = newDist;
                        prev[v] = u;
                        pq.offer(new int[]{v, newDist});
                    }
                }
            }
        }

        // Reconstruct the shortest path
        List<Integer> path = new ArrayList<>();
        int crawl = dest;
        while (crawl != -1) {
            path.add(0, crawl);
            crawl = prev[crawl];
        }

        return path;
    }

    public int getCost(List<Integer> path) {
        int cost = 0;
        for (int i = 0; i < path.size() - 1; i++) {
            int u = path.get(i);
            int v = path.get(i + 1);
            cost += graph[u][v]; // Add the weight of the edge between u and v to the total cost
        }
        return cost;
    }

}

public class ShortestPath {
    private static final int RADIUS = 20;

    private Graph g;
    private List<Integer> shortestPath;
    private int shortestPathCost;

    public void createAndShowGUI() {
        try {
            File file = new File("graph_input.txt");
            Scanner scanner = new Scanner(file);

            int V = scanner.nextInt();
            g = new Graph(V);

            for (int i = 0; i < V; i++) {
                for (int j = 0; j < V; j++) {
                    int weight = scanner.nextInt();
                    if (weight != 0) {
                        g.addEdge(i, j, weight);
                    }
                }
            }

            int s = scanner.nextInt();
            int t = scanner.nextInt();

            shortestPath = g.dijkstra(s, t);
            shortestPathCost = g.getCost(shortestPath);

            scanner.close();

            JFrame frame = new JFrame("Shortest Path Graph");
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            frame.setSize(800, 600);

            GraphPanel graphPanel = new GraphPanel(V);
            graphPanel.setGraph(g);
            graphPanel.setShortestPath(shortestPath);

            JLabel pathLabel = new JLabel("Shortest Path: " + shortestPath);
            JLabel costLabel = new JLabel("Shortest Path Cost: " + shortestPathCost);

            JPanel panel = new JPanel(new BorderLayout());
            panel.add(pathLabel, BorderLayout.NORTH);
            panel.add(costLabel, BorderLayout.SOUTH);

            frame.getContentPane().add(graphPanel, BorderLayout.CENTER);
            frame.getContentPane().add(panel, BorderLayout.SOUTH);

            frame.setVisible(true);
        } catch (FileNotFoundException e) {
            System.out.println("File not found.");
            e.printStackTrace();
        }
    }

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            ShortestPath shortestPath = new ShortestPath();
            shortestPath.createAndShowGUI();
        });
    }
}

class GraphPanel extends JPanel {
    private Graph g;
    private List<Integer> shortestPath;
    private int V;
    private static final int RADIUS = 20;

    public GraphPanel(int V) {
        this.V = V;
    }

    public void setGraph(Graph g) {
        this.g = g;
    }

    public void setShortestPath(List<Integer> shortestPath) {
        this.shortestPath = shortestPath;
    }

//    @Override
//    protected void paintComponent(Graphics g) {
//
//    }
}
