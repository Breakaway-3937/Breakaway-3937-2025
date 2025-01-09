package frc.robot.lib.Vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;

/** 
 * Same as PhotonCamera but adds back in removed methods. Extends PhotonCamera.
 * @see {@link PhotonCamera}
 */
public class BreakaCamera extends PhotonCamera {
    PhotonPoseEstimator poseEstimator;
    boolean noPoseEst;
    SimCameraProperties cameraProp;

    /**
     * Creates a BreakaCamera
     * @param cameraName Name of camera beign created. Get from PV dashboard
     */
    public BreakaCamera(String cameraName) {
        super(cameraName);
        noPoseEst = true;

        if(Robot.isSimulation()) {
          cameraProp = new SimCameraProperties();
          configCameraSimulation();
        }
    }
    
    /**
     * Creates a BreakaCamera
     * @param cameraName Name of camera beign created. Get from PV dashboard
     * @param poseEstimator Pose estmator to be used for Pose estimation. 
     */
    public BreakaCamera(String cameraName, PhotonPoseEstimator poseEstimator) {
        super(cameraName);
        noPoseEst = false;
        this.poseEstimator = poseEstimator;

        if(Robot.isSimulation()) {
          cameraProp = new SimCameraProperties();
          configCameraSimulation();
        }
    }

    public PhotonPipelineResult getLatest() {
      var results = super.getAllUnreadResults();

      if(results.isEmpty()) {
        return new PhotonPipelineResult();
      }
      else {
        return results.get(results.size() - 1);
      }
    }

    /**
     * Gets update pose from camera/coprocesser. 
     * 
     * @throws IllegalStateException thrown if no pose estmator is provided. 
     * @return Latest Pose estimate
     */
    public Optional<EstimatedRobotPose> getEstimatedPose() {
      if(noPoseEst) {
          throw new IllegalStateException("No pose Estimator provided. Do not call this method or add pose estimtor"); //Maybe change
      }
      else {
          return poseEstimator.update(getLatest());
      }
  }

    public PhotonPoseEstimator getPhotonPoseEstimator() {
        return poseEstimator;
    }

    public boolean isDead(){
      if(!super.isConnected()){
        return true;
      }
      else{
        return false;
      }
    }

    private void configCameraSimulation() {
      cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
      cameraProp.setCalibError(0.25, 0.25);
      cameraProp.setFPS(20);
      cameraProp.setAvgLatencyMs(20);
      cameraProp.setLatencyStdDevMs(5);
    }

    public PhotonCameraSim getSimCamera() {
      if(!Robot.isSimulation()) {
        System.out.println("Sim Camera not Configed. Not in Simulation");
        DriverStation.reportWarning("Sim Camera not Configed. Not in Simulation", false);
      }
      return new PhotonCameraSim(this, cameraProp);
    }
}
