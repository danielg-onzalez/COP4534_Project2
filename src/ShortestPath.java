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
        pq.offer(new int[] { src, 0 });

        while (!pq.isEmpty()) {
            int[] pair = pq.poll();
            int u = pair[0];
            if (visited[u])
                continue;
            visited[u] = true;

            if (u == dest)
                break;

            for (int v = 0; v < V; v++) {
                if (graph[u][v] > 0 && !visited[v]) {
                    int newDist = minDist[u] + graph[u][v];
                    if (newDist < minDist[v]) {
                        minDist[v] = newDist;
                        prev[v] = u;
                        pq.offer(new int[] { v, newDist });
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
    private static final int RADIUS = 50;

    public GraphPanel(int V) {
        this.V = V;
    }

    public void setGraph(Graph g) {
        this.g = g;
    }

    public void setShortestPath(List<Integer> shortestPath) {
        this.shortestPath = shortestPath;
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);

        // Calculate the angle step
        double angleStep = 2 * Math.PI / V;

        // Calculate the center of the panel
        int centerX = getWidth() / 2;
        int centerY = getHeight() / 2;

        // Calculate the radius of the circle on which the vertices are located
        int circleRadius = Math.min(getWidth(), getHeight()) / 2 - RADIUS;

        // Draw the vertices and the edges
        for (int i = 0; i < V; i++) {
            // Calculate the position of the vertex
            int x = centerX + (int) (circleRadius * Math.cos(i * angleStep)) - RADIUS / 2;
            int y = centerY + (int) (circleRadius * Math.sin(i * angleStep)) - RADIUS / 2;

            // Draw the edges
            for (int j = 0; j < V; j++) {
                if (this.g.graph[i][j] > 0) {
                    int x2 = centerX + (int) (circleRadius * Math.cos(j * angleStep)) - RADIUS / 2;
                    int y2 = centerY + (int) (circleRadius * Math.sin(j * angleStep)) - RADIUS / 2;
                    g.setColor(Color.BLACK);
                    g.drawLine(x + RADIUS / 2, y + RADIUS / 2, x2 + RADIUS / 2, y2 + RADIUS / 2);
                }
            }

            // Draw the vertex
            g.setColor(Color.ORANGE);
            g.fillOval(x, y, RADIUS, RADIUS);
            g.setColor(Color.BLACK);
            g.drawOval(x, y, RADIUS, RADIUS);
            g.setFont(new Font(Font.SANS_SERIF, Font.BOLD, 24));
            g.drawString(Integer.toString(i), x + RADIUS / 4, y + 3 * RADIUS / 4);
        }
        // Highlight the shortest path
        for (int i = 0; i < shortestPath.size() - 1; i++) {
            int u = shortestPath.get(i);
            int v = shortestPath.get(i + 1);

            // Calculate the positions of the vertices
            int x1 = centerX + (int) (circleRadius * Math.cos(u * angleStep)) - RADIUS / 2;
            int y1 = centerY + (int) (circleRadius * Math.sin(u * angleStep)) - RADIUS / 2;
            int x2 = centerX + (int) (circleRadius * Math.cos(v * angleStep)) - RADIUS / 2;
            int y2 = centerY + (int) (circleRadius * Math.sin(v * angleStep)) - RADIUS / 2;

            // Draw the edge
            g.setColor(Color.RED);
            g.drawLine(x1 + RADIUS / 2, y1 + RADIUS / 2, x2 + RADIUS / 2, y2 + RADIUS / 2);

            // Draw the vertices
            g.setColor(Color.ORANGE); 
            g.fillOval(x1, y1, RADIUS, RADIUS);
            g.fillOval(x2, y2, RADIUS, RADIUS);
            g.setColor(Color.BLACK);
            g.drawOval(x1, y1, RADIUS, RADIUS);
            g.drawOval(x2, y2, RADIUS, RADIUS);

            // Center the numbers in the circle
            FontMetrics fm = g.getFontMetrics();
            double textWidth = fm.getStringBounds(Integer.toString(u), g).getWidth();
            g.drawString(Integer.toString(u), (int) (x1 + RADIUS / 2 - textWidth / 2),
                    (int) (y1 + RADIUS / 2 + fm.getMaxAscent() / 2));

            textWidth = fm.getStringBounds(Integer.toString(v), g).getWidth();
            g.drawString(Integer.toString(v), (int) (x2 + RADIUS / 2 - textWidth / 2),
                    (int) (y2 + RADIUS / 2 + fm.getMaxAscent() / 2));
        }
    }

}
