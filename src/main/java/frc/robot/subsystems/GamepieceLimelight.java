// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class GamepieceLimelight {
    private NetworkTable table;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;
    private NetworkTableEntry tv;

    //current robot angle from pidgeon  + horizontal angle offset (tx) = target angle to turn to

    public GamepieceLimelight(String name) {
        /**
        * tx - Horizontal Offset
        * ty - Vertical Offset 
        * ta - Area of target 
        * tv - Target Visible
        */
        this.table = NetworkTableInstance.getDefault().getTable(name);
        this.tx = table.getEntry("tx");
        this.ty = table.getEntry("ty");
        this.ta = table.getEntry("ta");
        this.tv = table.getEntry("tv");
    }
}
