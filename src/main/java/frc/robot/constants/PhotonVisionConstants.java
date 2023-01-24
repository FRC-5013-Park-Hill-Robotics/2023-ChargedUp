
package frc.robot.constants;



import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class PhotonVisionConstants{
    public static Transform3d robotToCam1 = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
    public static Transform3d robotToCam2 = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));  
}

