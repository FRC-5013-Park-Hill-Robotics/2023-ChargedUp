package frc.robot.subsystems;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.IOException;
import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

import frc.robot.RobotContainer;



public class PhotonVision extends SubsystemBase{

    private PhotonCamera m_frontCamera = new PhotonCamera("frontcamera");
    private PhotonCamera m_rearCamera = new PhotonCamera("backcamera");
    private RobotContainer m_robotContainer;

    Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); 

    ArrayList<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();

    AprilTagFieldLayout aprilTagFieldLayout;
        RobotPoseEstimator robotPoseEstimator = new RobotPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);



    public PhotonVision(){
        super();
        camList.add(new Pair<PhotonCamera, Transform3d>(m_frontCamera, robotToCam));
        try{
            aprilTagFieldLayout = new AprilTagFieldLayout(AprilTagFields.k2022RapidReact.m_resourceFile);
        } catch (IOException e){
            System.out.println(e.toString());
        }
    }

}

