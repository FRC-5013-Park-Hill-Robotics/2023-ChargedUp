package frc.robot.constants;



import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class PhotonVisionConstants{
    
    
    public static final class FrontCamera{
        public static final Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
        public static final String name = "frontCamera";
    }
    public final static class RearCamera{
        public static final Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
        public static final String name = "rearCamera";
    }
}