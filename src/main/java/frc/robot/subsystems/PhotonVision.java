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
import frc.robot.constants.PhotonVisionConstants.LeftCamera;
import frc.robot.constants.PhotonVisionConstants.RightCamera;
import edu.wpi.first.wpilibj.DriverStation;
import static frc.robot.constants.PhotonVisionConstants.LeftCamera;
import static frc.robot.constants.PhotonVisionConstants.RightCamera;



public class PhotonVision extends SubsystemBase{

    private PhotonCamera m_leftCamera = new PhotonCamera(LeftCamera.name);
    private PhotonCamera m_rightCamera = new PhotonCamera(RightCamera.name);
    private RobotContainer m_robotContainer;

    private ArrayList<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();

    private AprilTagFieldLayout aprilTagFieldLayout;
    private PhotonPoseEstimator m_leftPoseEstimator;
    private PhotonPoseEstimator m_rightPoseEstimator;
    private boolean isinitialized = false;

    public PhotonVision(){
        super();
        try{
            aprilTagFieldLayout = new AprilTagFieldLayout(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e){
            System.out.println(e.toString());
        }

    }
    
    public void initialize(){
        Alliance alliance =  DriverStation.getAlliance();
        if(alliance  != Alliance.Invalid){
            if (aprilTagFieldLayout == null){
                try{
                    aprilTagFieldLayout = new AprilTagFieldLayout(AprilTagFields.k2023ChargedUp.m_resourceFile);
                } catch (IOException e){
                    System.out.println(e.toString());
                }
            }
            if(alliance == Alliance.Red){
                aprilTagFieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
            } else {
                aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
            } 

            m_leftPoseEstimator =new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, m_leftCamera,LeftCamera.robotToCam );
            m_rightPoseEstimator =new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, m_rightCamera,RightCamera.robotToCam );

            isinitialized = true;
        }
    }

    @Override
    public void periodic(){ 
        if (isinitialized){
            System.out.println("Photon initilized");
            if (m_leftPoseEstimator != null){
                System.out.println("left");
                m_robotContainer.getDrivetrain().updatePoseEstimator(m_leftPoseEstimator);
            }
            if (m_rightPoseEstimator != null){
                System.out.println("right");
                m_robotContainer.getDrivetrain().updatePoseEstimator(m_rightPoseEstimator);
            }
        } 
    }

}

