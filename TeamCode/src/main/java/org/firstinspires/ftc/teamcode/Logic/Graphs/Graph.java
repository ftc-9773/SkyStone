package org.firstinspires.ftc.teamcode.Logic.Graphs;

import java.util.ArrayList;

public class Graph {
    public ArrayList<Node> nodes;
    private Matrix edgeLookup;
    private int numNodes = 0;

    public Graph(){
        this.nodes = new ArrayList<>();

    }
}
