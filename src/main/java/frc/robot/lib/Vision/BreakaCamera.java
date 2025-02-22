package frc.robot.lib.Vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

/** 
 * Same as PhotonCamera but adds back in removed methods. Extends PhotonCamera.
 * @see {@link PhotonCamera}
 */
public class BreakaCamera extends PhotonCamera {
    private PhotonPoseEstimator poseEstimator;
    private final boolean noPoseEst;

    /**
     * Creates a BreakaCamera
     * @param cameraName Name of camera being created. Get from PV dashboard.
     */
    public BreakaCamera(String cameraName) {
        super(cameraName);
        noPoseEst = true;
    }
    
    /**
     * Creates a BreakaCamera
     * @param cameraName Name of camera being created. Get from PV dashboard.
     * @param poseEstimator Pose estimator to be used for pose estimation. 
     */
    public BreakaCamera(String cameraName, PhotonPoseEstimator poseEstimator) {
        super(cameraName);
        noPoseEst = false;
        this.poseEstimator = poseEstimator;
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
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
