import javax.swing.*;
import java.awt.*;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.List;
import java.util.Scanner;
import java.util.*;

/**
 * PrimalityTest Class
 *
 * @authors:
 *       Daniel Gonzalez 6285324 Algorithm Techniques UO2
 *       Muhammad Hashim 6349162 Algorithm Techniques UO1
 */

class Graph {
    private int V;
    public int[][] graph;

    public Graph(int V) {
        this.V = V;
        graph = new int[V][V];
    }


    /**
     * Method which adds the edge given the starting vertex, ending vertex, and weight
     *
     * @param u the starting vertex of the edge
     * @param v the ending vertex of the edge
     * @param weight the weight of the edge
     * @return VOID
     */
    public void addEdge(int u, int v, int weight) {
        graph[u][v] = weight;
        graph[v][u] = weight;
    }

    /**
     * Method which taking a source vertex and a destintation vertex and applied dijnkstras algorithm to find the shortest path
     *
     * @param src the starting vertex
     * @param dest the ending vertex
     * @return LIST<Integer> containing the shortest path
     */
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

        List<Integer> path = new ArrayList<>();
        int crawl = dest;
        while (crawl != -1) {
            path.add(0, crawl);
            crawl = prev[crawl];
        }

        return path;
    }

    /**
     * Method which takes in the path found and finds the cost
     *
     * @param path the shortest path provided in which the cost will be calculated
     * @return int, the cost of the path within the list provided
     */
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


    /**
     * Method which creates graph, populated graph with input text, and applies the proper methods to find the shortest path and cost. It then graphs and prints
     * to the console the shortest path and displays the shortest path highlighted red aswell.
     *
     * @return VOID
     */
    public void createAndShowGUI() {
        try {
            File file = new File("graph_input2.txt");
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


            System.out.printf("Shortest path from %d to %d:\n", s, t);
            for(int i = 0; i<shortestPath.size(); i++){
                if(i != shortestPath.size()-1)
                    System.out.print(shortestPath.get(i) + "-");
                else
                    System.out.print(shortestPath.get(i));
            }
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

    /**
     * Main method which will create a shortest Path object and then create and show a GUI with the populated graph for it.
     *
     * @return VOID
     */
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

    /**
     * Method which sets the graph of the class.
     *
     * @param g, the graph we are setting.
     * @return VOID
     */
    public void setGraph(Graph g) {
        this.g = g;
    }

    /**
     * Method which sets the shortestPath for the class
     *
     * @param shortestPath, the list containing the shortestPath whichw ill be used to graph.
     * @return VOID
     */
    public void setShortestPath(List<Integer> shortestPath) {
        this.shortestPath = shortestPath;
    }

    /**
     * Method which draws out a vertex and its edge along with its weight. Method also goes to highlight the edge red if its a part of the shortest path.
     *
     * @param g, the graphic object provided by the swing.
     * @return VOID
     */
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);

        double angleStep = 2 * Math.PI / V;

        int centerX = getWidth() / 2;
        int centerY = getHeight() / 2;

        int circleRadius = Math.min(getWidth(), getHeight()) / 2 - RADIUS;

        for (int i = 0; i < V; i++) {
            int x1 = centerX + (int) (circleRadius * Math.cos(i * angleStep));
            int y1 = centerY + (int) (circleRadius * Math.sin(i * angleStep));

            for (int j = 0; j < V; j++) {
                if (this.g.graph[i][j] > 0) {
                    int x2 = centerX + (int) (circleRadius * Math.cos(j * angleStep));
                    int y2 = centerY + (int) (circleRadius * Math.sin(j * angleStep));

                    double angle = Math.atan2(y2 - y1, x2 - x1);

                    int adjustedX1 = x1 + (int) (RADIUS / 2 * Math.cos(angle));
                    int adjustedY1 = y1 + (int) (RADIUS / 2 * Math.sin(angle));
                    int adjustedX2 = x2 - (int) (RADIUS / 2 * Math.cos(angle));
                    int adjustedY2 = y2 - (int) (RADIUS / 2 * Math.sin(angle));

                    g.setColor(Color.BLACK);
                    g.drawLine(adjustedX1, adjustedY1, adjustedX2, adjustedY2);

                    int midX = (adjustedX1 + adjustedX2) / 2;
                    int midY = (adjustedY1 + adjustedY2) / 2;

                    String weight = Integer.toString(this.g.graph[i][j]);
                    g.setFont(new Font("Default", Font.BOLD, 14));
                    g.drawString(weight, midX, midY-5);
                }
            }

            g.setColor(Color.ORANGE);
            g.fillOval(x1 - RADIUS / 2, y1 - RADIUS / 2, RADIUS, RADIUS);
            g.setColor(Color.BLACK);
            g.drawOval(x1 - RADIUS / 2, y1 - RADIUS / 2, RADIUS, RADIUS);

            String number = Integer.toString(i);
            g.setFont(new Font(Font.SANS_SERIF, Font.BOLD, 24));
            FontMetrics fm = g.getFontMetrics();
            int textWidth = fm.stringWidth(number);
            int textHeight = fm.getAscent() - fm.getDescent();
            int textX = x1 - textWidth / 2;
            int textY = y1 + textHeight / 2;
            g.drawString(number, textX, textY);
        }

        for (int i = 0; i < shortestPath.size() - 1; i++) {
            int u = shortestPath.get(i);
            int v = shortestPath.get(i + 1);

            int x1 = centerX + (int) (circleRadius * Math.cos(u * angleStep)) - RADIUS / 2;
            int y1 = centerY + (int) (circleRadius * Math.sin(u * angleStep)) - RADIUS / 2;
            int x2 = centerX + (int) (circleRadius * Math.cos(v * angleStep)) - RADIUS / 2;
            int y2 = centerY + (int) (circleRadius * Math.sin(v * angleStep)) - RADIUS / 2;

            g.setColor(Color.RED);
            g.drawLine(x1 + RADIUS / 2, y1 + RADIUS / 2, x2 + RADIUS / 2, y2 + RADIUS / 2);

            g.setColor(Color.ORANGE);
            g.fillOval(x1, y1, RADIUS, RADIUS);
            g.fillOval(x2, y2, RADIUS, RADIUS);
            g.setColor(Color.BLACK);
            g.drawOval(x1, y1, RADIUS, RADIUS);
            g.drawOval(x2, y2, RADIUS, RADIUS);

            FontMetrics fm = g.getFontMetrics();
            String uStr = Integer.toString(u);
            String vStr = Integer.toString(v);
            int uTextWidth = fm.stringWidth(uStr);
            int vTextWidth = fm.stringWidth(vStr);
            g.drawString(uStr, x1 + RADIUS / 2 - uTextWidth / 2, y1 + RADIUS / 2 + fm.getAscent() / 2);
            g.drawString(vStr, x2 + RADIUS / 2 - vTextWidth / 2, y2 + RADIUS / 2 + fm.getAscent() / 2);
        }
    }
}
