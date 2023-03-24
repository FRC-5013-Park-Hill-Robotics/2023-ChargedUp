package frc.robot.constants;



import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class PhotonVisionConstants{
    
    
    public static final class LeftCamera{
        public static final Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,Units.degreesToRadians(-30),0));
        public static final String name = "Left Camera";
    }
    public final static class RightCamera{
        public static final Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,Units.degreesToRadians(30),0));
        public static final String name = "Right Camera";
    }
}