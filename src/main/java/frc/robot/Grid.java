// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Grid {
    private final Node leftNode;
    private final Node middleNode;
    private final Node rightNode;

    public Grid(Node leftNode, Node middleNode, Node rightNode) {
        this.leftNode = leftNode;
        this.middleNode = middleNode;
        this.rightNode = rightNode;

    }

    public Node getLeftNode() {
        return this.leftNode;
    }

    public Node getMiddleNode() {
        return this.middleNode;
    }

    public Node getRightNode() {
        return this.rightNode;
    }


}
