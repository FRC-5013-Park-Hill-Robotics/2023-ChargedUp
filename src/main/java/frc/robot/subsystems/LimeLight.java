package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.LimeLightConstants;
import frc.robot.trobot5013lib.LimelightHelpers;
import webblib.util.RectanglePoseArea;

public class LimeLight extends SubsystemBase {
  /** Creates a new LimeLight. */
  public static final RectanglePoseArea field =
  new RectanglePoseArea(new Translation2d(0.0, 0.0), new Translation2d(16.54, 8.02));
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tx = table.getEntry("tx");
  private NetworkTableEntry ty = table.getEntry("ty");
  private NetworkTableEntry ta = table.getEntry("ta");
  private NetworkTableEntry tv = table.getEntry("tv");
  private RobotContainer m_robotContainer;
  Alliance alliance;
  private Boolean enable = true;
  private Boolean trust = false;
  private int fieldError = 0;
  private int distanceError = 0;
  private Pose2d botpose;
  public LimeLight() {
    /**
     * tx - Horizontal Offset
     * ty - Vertical Offset 
     * ta - Area of target 
     * tv - Target Visible
     */

    
    this.tx = table.getEntry("tx");
    this.ty = table.getEntry("ty");
    this.ta = table.getEntry("ta");
    this.tv = table.getEntry("tv");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

     //read values periodically
    double x = this.tx.getDouble(0.0);
    double y = this.ty.getDouble(0.0);
    double area = this.ta.getDouble(0.0);


    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    Drivetrain drivebase = RobotContainer.getInstance().getDrivetrain();
    LimelightHelpers.Results result =
          LimelightHelpers.getLatestResults("limelight").targetingResults;
      if (!(result.botpose[0] == 0 && result.botpose[1] == 0)) {
        if (alliance == Alliance.Blue) {
          botpose = LimelightHelpers.toPose2D(result.botpose_wpiblue);
        } else if (alliance == Alliance.Red) {
          botpose = LimelightHelpers.toPose2D(result.botpose_wpired);
        }
        if (botpose != null){
        if (field.isPoseWithinArea(botpose)) {
          if (drivebase.getPose().getTranslation().getDistance(botpose.getTranslation()) < 0.33
              || trust || result.targets_Fiducials.length > 1) {
            /**drivebase.addVisionMeasurement(
                botpose,
                Timer.getFPGATimestamp()
                    - (result.latency_capture / 1000.0)
                    - (result.latency_pipeline / 1000.0),
                true,
                1.0);
            **/
          } else {
            distanceError++;
            SmartDashboard.putNumber("Limelight Error", distanceError);
          }
        } else {
          fieldError++;
          SmartDashboard.putNumber("Field Error", fieldError);
        }
      }
    }

  }

  public void setTrust(boolean newTrust){
    trust = newTrust;
  }
  public void setAlliance(Alliance alliance) {
    this.alliance = alliance;
  }
  public double getHorizontalAngleOfErrorDegrees(){
    //+1 is a fudge factor cor camera mounting
    return getTx().getDouble(0.0) + LimeLightConstants.HORIZONTAL_OFFSET;
  }

  public double getVerticalAngleOfErrorDegrees(){
    //+1 is a fudge factor cor camera mounting
    return getTy().getDouble(0.0) + LimeLightConstants.VERTICAL_OFFSET;
  }


 public NetworkTableEntry getTx() {
    return tx;
  }

  public NetworkTableEntry getTy() {
    return ty;
  }

  public NetworkTableEntry getTa() {
    return ta;
  }

  public double getTxAngleRadians() {
    return Units.degreesToRadians(tx.getDouble(0));
  }

  public double getTargetAngleRadians() {
    return getTxAngleRadians()+m_robotContainer.getDrivetrain().getHeadingRadians();
  }
  public boolean hasTarget(){
    SmartDashboard.putNumber("tv; ", tv.getDouble(0));
    return tv.getDouble(0) != 0;
  }
}

