package org.firstinspires.ftc.teamcode.Logic.Graphs;

import java.util.ArrayList;

public class Node {
    public String name;
    public int id;

    public Node(String name, int id){
        this.name = name;
        this.id = id;
    }

    public Node(int id){
        this.id = id;
        this.name = "Node: " + id;
    }
}
