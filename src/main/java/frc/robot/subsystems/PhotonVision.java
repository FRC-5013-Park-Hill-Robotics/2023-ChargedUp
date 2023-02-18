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
import org.photonvision.PhotonPoseEstimator.PoseStrategy;


import frc.robot.RobotContainer;
import frc.robot.constants.PhotonVisionConstants.FrontCamera;
import frc.robot.constants.PhotonVisionConstants.RearCamera;
import edu.wpi.first.wpilibj.DriverStation;
import static frc.robot.constants.PhotonVisionConstants.FrontCamera;
import static frc.robot.constants.PhotonVisionConstants.RearCamera;



public class PhotonVision extends SubsystemBase{

    private PhotonCamera m_frontCamera = new PhotonCamera(FrontCamera.name);
    private PhotonCamera m_rearCamera = new PhotonCamera(RearCamera.name);
    private RobotContainer m_robotContainer;

    private ArrayList<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();

    private AprilTagFieldLayout aprilTagFieldLayout;
    private PhotonPoseEstimator m_frontPoseEstimator;
    private PhotonPoseEstimator m_rearPoseEstimator;
    private boolean isinitialized = false;

    public PhotonVision(){
        super();
        try{
            aprilTagFieldLayout = new AprilTagFieldLayout(AprilTagFields.k2022RapidReact.m_resourceFile);
        } catch (IOException e){
            System.out.println(e.toString());
        }

    }
    
    public void initialize(){
        Alliance alliance =  DriverStation.getAlliance();
        if(alliance  != Alliance.Invalid){
            isinitialized = true;
            if(alliance == Alliance.Red){
                aprilTagFieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
            } else {
                aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
            } 

            m_frontPoseEstimator =new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, m_frontCamera,FrontCamera.robotToCam );
            m_rearPoseEstimator =new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, m_rearCamera,RearCamera.robotToCam );
        }
    }

    @Override
    public void periodic(){ 
        if (isinitialized){
            if (m_frontPoseEstimator != null){
                m_robotContainer.getDrivetrain().updatePoseEstimator(m_frontPoseEstimator);
            }
            if (m_rearPoseEstimator != null){
                m_robotContainer.getDrivetrain().updatePoseEstimator(m_rearPoseEstimator);
            }
        } 
    }

}

