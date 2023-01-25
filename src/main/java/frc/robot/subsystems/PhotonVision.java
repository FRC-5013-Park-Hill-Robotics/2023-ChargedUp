package frc.robot.subsystems;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;


import frc.robot.RobotContainer;
import frc.robot.constants.PhotonVisionConstants;
import frc.robot.constants.PhotonVisionConstants.FrontCamera;
import frc.robot.constants.PhotonVisionConstants.RearCamera;



public class PhotonVision extends SubsystemBase{

    private PhotonCamera m_frontCamera = new PhotonCamera(FrontCamera.name);
    private PhotonCamera m_rearCamera = new PhotonCamera(RearCamera.name);
    private RobotContainer m_robotContainer;
    private Alliance m_alliance;

    ArrayList<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();

    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonPoseEstimator photonPoseEstimator;
    private boolean isinitialized = false;


    public PhotonVision(){
        super();
        camList.add(new Pair<PhotonCamera, Transform3d>(m_frontCamera, FrontCamera.robotToCam));
        camList.add(new Pair<PhotonCamera, Transform3d>(m_rearCamera, RearCamera.robotToCam));
        try{
            aprilTagFieldLayout = new AprilTagFieldLayout(AprilTagFields.k2022RapidReact.m_resourceFile);
        } catch (IOException e){
            System.out.println(e.toString());
        }
        
        //Alliance alliance;
        //if(alliance  != Alliance.Invalid){
            //if (isinitialized){
                //if(alliance == Alliance.Red){
                    //aprilTagFieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
                //} else {
                    //aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
                //}
            //}
        //}

    }
    
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    
    //public Command updateDrivetrainPose(){
        
      //  m_robotContainer.getDrivetrain().updatePoseEstimator(photonPoseEstimator);
    //}
}

