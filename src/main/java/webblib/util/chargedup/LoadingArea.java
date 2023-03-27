package webblib.util.chargedup;

import edu.wpi.first.math.geometry.Pose2d;
import webblib.util.HolonomicPose2d;
import webblib.util.RectanglePoseArea;

public class LoadingArea {
  private final RectanglePoseArea largeLoadingRect;
  private final RectanglePoseArea smallLoadingRect;
  private final HolonomicPose2d doubleSubstationBarrier;
  private final HolonomicPose2d doubleSubstationRail;

  public LoadingArea(
      RectanglePoseArea largeLoadingRect,
      RectanglePoseArea smallLoadingRect,
      HolonomicPose2d doubleSubstationRail,
      HolonomicPose2d doubleSubstationBarrier) {
    this.largeLoadingRect = largeLoadingRect;
    this.smallLoadingRect = smallLoadingRect;
    this.doubleSubstationBarrier = doubleSubstationBarrier;
    this.doubleSubstationRail = doubleSubstationRail;
  }

  public RectanglePoseArea getLargeLoadingRectangle() {
    return largeLoadingRect;
  }

  public RectanglePoseArea getSmallLoadingRectangle() {
    return smallLoadingRect;
  }

  public HolonomicPose2d getDoubleSubstationBarrier() {
    return doubleSubstationBarrier;
  }

  public HolonomicPose2d getDoubleSubstationRail() {
    return doubleSubstationRail;
  }

  public boolean isPoseWithinLoadingArea(Pose2d pose) {
    return largeLoadingRect.isPoseWithinArea(pose) || smallLoadingRect.isPoseWithinArea(pose);
  }
}
