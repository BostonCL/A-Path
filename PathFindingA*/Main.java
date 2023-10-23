import java.awt.*;
import java.util.*;
import java.util.List;
import javax.swing.*;

class Node {
    int x, y; // coordinates of the node
    double g, h; // cost to reach the node from the start and estimated cost to reach the goal
    Node parent; // reference to the parent node

    public Node(int x, int y) {
        this.x = x;
        this.y = y;
        g = Double.POSITIVE_INFINITY;
        h = 0;
        parent = null;
    }

    public double f() { // f = g + h
        return g + h;
    }
}

class Edge {
    Node from, to;

    public Edge(Node from, Node to) {
        this.from = from;
        this.to = to;
    }
}

class Graph {
    List<Node> nodes;
    List<Edge> edges;

    public Graph() {
        nodes = new ArrayList<>();
        edges = new ArrayList<>();
    }

    public void addNode(Node node) {
        nodes.add(node);
    }

    public void addEdge(Node from, Node to) {
        edges.add(new Edge(from, to));
    }
}

class AStar {
    Graph graph;
    Node start, goal;

    public AStar(Graph graph, Node start, Node goal) {
        this.graph = graph;
        this.start = start;
        this.goal = goal;
    }

    public List<Node> findShortestPath() {
        Set<Node> visited = new HashSet<>();
        PriorityQueue<Node> queue = new PriorityQueue<>(Comparator.comparingDouble(Node::f));

        start.g = 0;
        start.h = heuristic(start, goal);
        queue.add(start);

        while (!queue.isEmpty()) {
            Node current = queue.poll();

            if (current == goal) {
                return getPath(current);
            }

            visited.add(current);

            for (Edge edge : graph.edges) {
                if (edge.from == current && !visited.contains(edge.to)) {
                    double cost = current.g + distance(edge.from, edge.to);
                    if (cost < edge.to.g) {
                        edge.to.g = cost;
                        edge.to.h = heuristic(edge.to, goal);
                        edge.to.parent = current;
                        queue.add(edge.to);
                    }
                }
            }
        }

        return null;
    }

    private double distance(Node from, Node to) {
        // compute the Euclidean distance between two nodes
        return Math.sqrt(Math.pow(from.x - to.x, 2) + Math.pow(from.y - to.y, 2));
    }

    private double heuristic(Node from, Node to) {
        // compute the Manhattan distance between two nodes
        return Math.abs(from.x - to.x) + Math.abs(from.y - to.y);
    }

    private List<Node> getPath(Node goal) {
        List<Node> path = new ArrayList<>();
        Node current = goal;

        while (current != null) {
            path.add(current);
            current = current.parent;
        }

        Collections.reverse(path);
        return path;
    }
}

public class Main extends JPanel {
    public static void main(String[] args) {

        JFrame frame = new JFrame("Drawing Polygons");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        PolygonsJPanel polygonsJPanel = new PolygonsJPanel();
        frame.add(polygonsJPanel); // add polygonsJPanel to frame
        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setResizable(false);
        frame.setSize(1700, 900); // set frame size
        frame.setVisible(true); // display frame

    }
}
    class PolygonsJPanel extends JPanel {
        public void paintComponent(Graphics g) {
            super.paintComponent(g); // call superclass's paintComponent

            // draw polygon with Polygon object
            int[] xValues = {860, 860, 210, 210};
            int[] yValues = {590, 820, 820, 590};
            Polygon polygon1 = new Polygon(xValues, yValues, 4);
            g.drawPolygon(polygon1);


            int[] xValues2 = {100, 310, 450, 350, 150};
            int[] yValues2 = {250, 60, 250, 450, 420};
            Polygon polygon2 = new Polygon(xValues2, yValues2, 5);
            g.drawPolygon(polygon2);

            int xValues3[] = {600, 500, 700};
            int yValues3[] = {170, 500, 500};
            Polygon polygon3 = new Polygon(xValues3, yValues3, 3);
            g.drawPolygon(polygon3);

            int xValues4[] = {900, 1080, 960};
            int yValues4[] = {410, 550, 705};
            Polygon polygon4 = new Polygon(xValues4, yValues4, 3);
            g.drawPolygon(polygon4);

            int xValues5[] = {730, 920, 990, 730};
            int yValues5[] = {40, 30, 140, 320};
            Polygon polygon5 = new Polygon(xValues5, yValues5, 4);
            g.drawPolygon(polygon5);

            int xValues6[] = {1300, 1300, 1030, 1030};
            int yValues6[] = {50, 450, 450, 50};
            Polygon polygon6 = new Polygon(xValues6, yValues6, 4);
            g.drawPolygon(polygon6);

            int xValues7[] = {1330, 1420, 1420, 1300, 1180, 1180};
            int yValues7[] = {480, 570, 740, 800, 740, 570};
            Polygon polygon7 = new Polygon(xValues7, yValues7, 6);
            g.drawPolygon(polygon7);


            int xValues8[] = {1440, 1500, 1460, 1330};
            int yValues8[] = {30, 100, 470, 90};
            Polygon polygon8 = new Polygon(xValues8, yValues8, 4);
            g.drawPolygon(polygon8);

            g.fillOval(100, 705, 20, 20);//start
            g.fillOval(1550, 30, 20, 20);//goal

            Graph graph = new Graph();
            Node start = new Node(100, 705);
            Node goal = new Node(1550, 30);

            // add nodes to the graph
            graph.addNode(start); //start
            graph.addNode(goal);
            graph.addNode(new Node(210, 820)); //CoordA //Node 2
            graph.addNode(new Node(210, 590)); //CoordB 3
            graph.addNode(new Node(860, 590)); //CoordC 4
            graph.addNode(new Node(860, 820)); //CoordD 5
            graph.addNode(new Node(100, 250)); //CoordE 6
            graph.addNode(new Node(310, 60));  //CoordF 7
            graph.addNode(new Node(450, 250)); //CoordG 8
            graph.addNode(new Node(350, 450)); //CoordH 9
            graph.addNode(new Node(150, 420)); //CoordI 10
            graph.addNode(new Node(500, 500)); //CoordJ 11
            graph.addNode(new Node(600, 170)); //CoordK 12
            graph.addNode(new Node(700, 500)); //CoordL 13
            graph.addNode(new Node(730, 320)); //CoordM 14
            graph.addNode(new Node(730, 40));  //CoordN 15
            graph.addNode(new Node(920, 30));  //CoordO 16
            graph.addNode(new Node(990, 140)); //CoordP 17
            graph.addNode(new Node(960, 705)); //CoordQ 18
            graph.addNode(new Node(900, 410)); //CoordR 19
            graph.addNode(new Node(1080, 550));//CoordS 20
            graph.addNode(new Node(1030, 450));//CoordT 21
            graph.addNode(new Node(1030, 50)); //CoordU 22
            graph.addNode(new Node(1300, 50)); //CoordV 23
            graph.addNode(new Node(1300, 450));//CoordW 24
            graph.addNode(new Node(1300, 800)); //Xcoord 25
            graph.addNode(new Node(1180, 740)); //Zcoord 26
            graph.addNode(new Node(1180, 570)); //coordA1 27
            graph.addNode(new Node(1330, 480)); //CoordB1 28
            graph.addNode(new Node(1420, 570)); //CoordC1 29
            graph.addNode(new Node(1420, 160)); //CoordD1 30
            graph.addNode(new Node(1460, 470)); //CoordE1 31
            graph.addNode(new Node(1330, 90));  //CoordF1 32
            graph.addNode(new Node(1440, 30)); //CoordG1 33
            graph.addNode(new Node(1500, 100)); //CoordH1 34

            // add edges to the graph

            //start to neighbor
            graph.addEdge(start, graph.nodes.get(2));
            graph.addEdge(start, graph.nodes.get(3));
            graph.addEdge(start, graph.nodes.get(6));
            graph.addEdge(start, graph.nodes.get(10));

            //node a to neighbor
            graph.addEdge(graph.nodes.get(2), graph.nodes.get(3));
            graph.addEdge(graph.nodes.get(2), graph.nodes.get(5));
            graph.addEdge(graph.nodes.get(2), graph.nodes.get(6));
            graph.addEdge(graph.nodes.get(2), graph.nodes.get(10));

            //node b to neighbor
            graph.addEdge(graph.nodes.get(3), graph.nodes.get(4));
            graph.addEdge(graph.nodes.get(3), graph.nodes.get(9));
            graph.addEdge(graph.nodes.get(3), graph.nodes.get(11));
            graph.addEdge(graph.nodes.get(3), graph.nodes.get(13));

            //node c to neighbor
            graph.addEdge(graph.nodes.get(4), graph.nodes.get(14));
            graph.addEdge(graph.nodes.get(4), graph.nodes.get(18));
            graph.addEdge(graph.nodes.get(4), graph.nodes.get(19));

            //node d to neighbor
            graph.addEdge(graph.nodes.get(5), graph.nodes.get(18));
            graph.addEdge(graph.nodes.get(5), graph.nodes.get(19));
            graph.addEdge(graph.nodes.get(5), graph.nodes.get(25));
            graph.addEdge(graph.nodes.get(5), graph.nodes.get(26));
            graph.addEdge(graph.nodes.get(5), graph.nodes.get(27));

            //node e to neighbor
            graph.addEdge(graph.nodes.get(6), graph.nodes.get(7));
            graph.addEdge(graph.nodes.get(6), graph.nodes.get(10));

            //node f to neighbor
            graph.addEdge(graph.nodes.get(7), graph.nodes.get(8));
            graph.addEdge(graph.nodes.get(7), graph.nodes.get(12));
            graph.addEdge(graph.nodes.get(7), graph.nodes.get(15));
            graph.addEdge(graph.nodes.get(7), graph.nodes.get(16));

            //node g to neighbor
            graph.addEdge(graph.nodes.get(8), graph.nodes.get(12));
            graph.addEdge(graph.nodes.get(8), graph.nodes.get(15));
            graph.addEdge(graph.nodes.get(8), graph.nodes.get(11));

            //node h to neighbor

            graph.addEdge(graph.nodes.get(9), graph.nodes.get(12));
            graph.addEdge(graph.nodes.get(9), graph.nodes.get(11));

            //node j to neighbor
            graph.addEdge(graph.nodes.get(11), graph.nodes.get(12));
            graph.addEdge(graph.nodes.get(11), graph.nodes.get(13));
            graph.addEdge(graph.nodes.get(11), graph.nodes.get(4));

            //node k to neighbor
            graph.addEdge(graph.nodes.get(12), graph.nodes.get(15));
            graph.addEdge(graph.nodes.get(12), graph.nodes.get(14));
            graph.addEdge(graph.nodes.get(12), graph.nodes.get(4));

            //node l to neighbor
            graph.addEdge(graph.nodes.get(13), graph.nodes.get(14));
            graph.addEdge(graph.nodes.get(13), graph.nodes.get(17));
            graph.addEdge(graph.nodes.get(13), graph.nodes.get(19));

            //node m to neighbor
            graph.addEdge(graph.nodes.get(14), graph.nodes.get(17));
            graph.addEdge(graph.nodes.get(14), graph.nodes.get(19));
            graph.addEdge(graph.nodes.get(14), graph.nodes.get(15));
            graph.addEdge(graph.nodes.get(14), graph.nodes.get(21));

            //node n to neighbor
            graph.addEdge(graph.nodes.get(15), graph.nodes.get(16));

            //node o to neighbor
            graph.addEdge(graph.nodes.get(16), graph.nodes.get(22));
            graph.addEdge(graph.nodes.get(16), graph.nodes.get(23));
            graph.addEdge(graph.nodes.get(16), graph.nodes.get(33));

            //node p to neighbor
            graph.addEdge(graph.nodes.get(17), graph.nodes.get(22));
            graph.addEdge(graph.nodes.get(17), graph.nodes.get(21));

            //node q to neighbor
            graph.addEdge(graph.nodes.get(18), graph.nodes.get(19));
            graph.addEdge(graph.nodes.get(18), graph.nodes.get(20));
            graph.addEdge(graph.nodes.get(18), graph.nodes.get(24));
            graph.addEdge(graph.nodes.get(18), graph.nodes.get(26));
            graph.addEdge(graph.nodes.get(18), graph.nodes.get(27));

            //node R to neighbor
            graph.addEdge(graph.nodes.get(19), graph.nodes.get(21));
            graph.addEdge(graph.nodes.get(19), graph.nodes.get(27));

            //node s to neighbor
            graph.addEdge(graph.nodes.get(20), graph.nodes.get(24));
            graph.addEdge(graph.nodes.get(20), graph.nodes.get(28));

            //node t to neighbor
            graph.addEdge(graph.nodes.get(21), graph.nodes.get(22));
            graph.addEdge(graph.nodes.get(21), graph.nodes.get(24));
            graph.addEdge(graph.nodes.get(21), graph.nodes.get(28));

            //node u to neighbor
            graph.addEdge(graph.nodes.get(22), graph.nodes.get(23));
            graph.addEdge(graph.nodes.get(22), graph.nodes.get(33));

            //node v to neighbor
            graph.addEdge(graph.nodes.get(23), graph.nodes.get(33));

            //node w to neighbor
            graph.addEdge(graph.nodes.get(24), graph.nodes.get(23));
            graph.addEdge(graph.nodes.get(24), graph.nodes.get(31));
            graph.addEdge(graph.nodes.get(24), graph.nodes.get(32));

            //node x to neighbor
            graph.addEdge(graph.nodes.get(25), graph.nodes.get(30));

            //node z to neighbor
            graph.addEdge(graph.nodes.get(26), graph.nodes.get(27));
            graph.addEdge(graph.nodes.get(26), graph.nodes.get(25));

            //node a1 to neighbor
            graph.addEdge(graph.nodes.get(27), graph.nodes.get(24));
            graph.addEdge(graph.nodes.get(27), graph.nodes.get(28));

            //node b1 to neighbor
            graph.addEdge(graph.nodes.get(28), graph.nodes.get(23));
            graph.addEdge(graph.nodes.get(28), graph.nodes.get(32));
            graph.addEdge(graph.nodes.get(28), graph.nodes.get(31));

            //node c1 to neighbor
            graph.addEdge(graph.nodes.get(29), graph.nodes.get(31));
            graph.addEdge(graph.nodes.get(29), graph.nodes.get(32));

            //node d1 to neighbor
            graph.addEdge(graph.nodes.get(30), graph.nodes.get(31));

            //node e1 to neighbor
            graph.addEdge(graph.nodes.get(31), goal);

            //node f1 to neighbor
            graph.addEdge(graph.nodes.get(32), graph.nodes.get(33));
            graph.addEdge(graph.nodes.get(32), graph.nodes.get(31));

            //node g1
            graph.addEdge(graph.nodes.get(33), goal);

            AStar astar = new AStar(graph, start, goal);
            List<Node> path = astar.findShortestPath();

            if (path == null) {
               System.out.println("No path found.");
           } else {
               g.setColor(Color.red);
               Node prevNode = null;
               for (Node node : path) {
                   if (prevNode != null) {
                       g.drawLine(prevNode.x + 2, prevNode.y + 2, node.x + 2, node.y + 2);
                   }
                   prevNode = node;
               }
           }

        }
    }

