package frc.robot.lib.Vision;

import static edu.wpi.first.units.Units.Meters;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** 
 * Same as PhotonCamera but adds back in removed methods. Extends PhotonCamera.
 * @see {@link PhotonCamera}
 */
public class BreakaCamera extends PhotonCamera {
    private PhotonPoseEstimator poseEstimator;
    private final boolean noPoseEst;
    private List<Integer> validTagsBlue, validTagsRed;
    private Distance maxDistance = Meters.of(5);
    private final InterpolatingDoubleTreeMap tagsStds= new InterpolatingDoubleTreeMap();

    /**
     * Creates a BreakaCamera
     * @param cameraName Name of camera being created. Get from PV dashboard.
     */
    public BreakaCamera(String cameraName, int[] blueTags, int[] redTags) {
        super(cameraName);
        noPoseEst = true;
        validTagsBlue = Arrays.stream(blueTags).boxed().toList();
        validTagsRed = Arrays.stream(redTags).boxed().toList();
        fillTree();
    }
    
    /**
     * Creates a BreakaCamera
     * @param cameraName Name of camera being created. Get from PV dashboard.
     * @param poseEstimator Pose estimator to be used for pose estimation. 
     */
    public BreakaCamera(String cameraName, PhotonPoseEstimator poseEstimator, int[] blueTags, int[] redTags) {
        super(cameraName);
        noPoseEst = false;
        this.poseEstimator = poseEstimator;
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        validTagsBlue = Arrays.stream(blueTags).boxed().toList();
        validTagsRed = Arrays.stream(redTags).boxed().toList();
        fillTree();
    }

    /**
     * @return The latest result from the camera
     */
    public PhotonPipelineResult getLatest() {
      return super.getLatestResult();
    }

    /**
     * Gets update pose from camera/coprocessor. 
     * 
     * @throws IllegalStateException thrown if no pose estmator is provided. 
     * @return Latest Pose Estimate
     */
    public Optional<EstimatedRobotPose> getEstimatedPose() {
      if(noPoseEst) {
          throw new IllegalStateException("No pose estimator provided. Do not call this method or add pose estimator.");
      }
      else {
          return poseEstimator.update(getLatest());
      }
    }

    public void updatePose(BiConsumer<EstimatedRobotPose, Vector<N3>> updateOdom) {
      var result = getEstimatedPose();

      if(result.isPresent()) {
        var distance = getAverageTagDistanceX(result).in(Meters);
        if(validTags(result) && distance < maxDistance.in(Meters)) {
          updateOdom.accept(result.get(), calcStd(distance));
        }
      }
    }

    private boolean validTags(Optional<EstimatedRobotPose> result) {
      var alliance = DriverStation.getAlliance();
      boolean isValid = false;
      List<PhotonTrackedTarget> targets;

      if(result.isPresent()) {
        targets = result.get().targetsUsed;
      }
      else {
        return false;
      }

      if(alliance.isPresent()) {
        if(alliance.get().equals(Alliance.Red)) {
          for(var target : targets) {
            if(validTagsRed.contains(target.fiducialId)) {
              isValid = false;
            }
          }
        }
        else {
          for(var target : targets) {
            if(validTagsBlue.contains(target.fiducialId)) {
              isValid = false;
            }
          }
        }
      }

      return isValid;
    }

    public Distance getAverageTagDistanceX(Optional<EstimatedRobotPose> result) {
      if(!result.isEmpty()) {
        double averageDistance = 0;
        for(int i = 0; i < result.get().targetsUsed.size(); i++) {
          averageDistance += Math.abs(result.get().targetsUsed.get(i).getBestCameraToTarget().getX());
        }
        return Meters.of(averageDistance /= (double) result.get().targetsUsed.size());
      }
      else {
        return Meters.of(9999);
      }
    }

    public Vector<N3> calcStd(double distance) {
      double xy = tagsStds.get(distance) * 2.25;
      return VecBuilder.fill(xy, xy, 99999);
    }

    public void fillTree() {
      tagsStds.put(0.752358, 0.005);
      tagsStds.put(1.016358, 0.0135);
      tagsStds.put(1.296358, 0.016);
      tagsStds.put(1.574358, 0.038);
      tagsStds.put(1.913358, 0.0515);
      tagsStds.put(2.184358, 0.0925);
      tagsStds.put(2.493358, 0.12);
      tagsStds.put(2.758358, 0.14);
      tagsStds.put(3.223358, 0.17);
      tagsStds.put(4.093358, 0.27);
      tagsStds.put(4.726358, 0.38);
    }

    /**
     * @return Pose Estimator passed to the object
     */
    public PhotonPoseEstimator getPhotonPoseEstimator() {
        return poseEstimator;
    }

    /**
     * @return Boolean that shows if camera is connected
     */
    public boolean isDead(){
      if(!super.isConnected()){
        return true;
      }
      else{
        return false;
      }
    }
}
