package frc.robot.subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.utilities.DaisyMath;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.LimelightHelpers.LimelightResults;
import frc.robot.utilities.LimelightHelpers.LimelightTarget_Detector;

public class LimelightInterface extends SubsystemBase {
  private static LimelightInterface mInstance;

  public static LimelightInterface getInstance() {
    if (mInstance == null)
      mInstance = new LimelightInterface();
    return mInstance;
  }

  public double tv; // velocity
  public double tx; // x position
  public double ty; // y position
  public double ta; // area
  public double ts; // timestamp
  public double tl; // timestamp
  public double cl; // timestamp
  public double tx_4; // x position
  public double tx_7; // x position

  public LimelightInterface() {
    tv = 0.0;
    tx = 0.0;
    ty = 0.0;
    ta = 0.0;
    ts = 0.0;
    tl = 0.0;
    cl = 0.0;

    tx_4 = 0.0;
    tx_7 = 0.0;

  }

  /**
   * This is the method for pulling values from LimeLight
   * This is called in the Robot Periodic Method
   */
  public void run() {

    tv = NetworkTableInstance.getDefault().getTable("limelight-host").getEntry("tv").getDouble(0);
    tx = -1.0 * NetworkTableInstance.getDefault().getTable("limelight-host").getEntry("tx").getDouble(0);
    ty = NetworkTableInstance.getDefault().getTable("limelight-host").getEntry("ty").getDouble(0);
    ta = NetworkTableInstance.getDefault().getTable("limelight-host").getEntry("ta").getDouble(0);
    ts = NetworkTableInstance.getDefault().getTable("limelight-host").getEntry("ts").getDouble(0);
    tl = NetworkTableInstance.getDefault().getTable("limelight-host").getEntry("tl").getDouble(0);
    cl = NetworkTableInstance.getDefault().getTable("limelight-host").getEntry("cl").getDouble(0);

  }

  /**
   * Calculates the Distance of the robot from the goal
   */

  public LimelightTarget_Detector returnPieceData() {
    LimelightResults newLimelightResults = LimelightHelpers.getLatestResults("limelight-ai");
    if (newLimelightResults.targetingResults.targets_Detector.length > 0) {
      return newLimelightResults.targetingResults.targets_Detector[0];
    }
    return null;
  }

  /**
   * Sets the LED State of the LimeLight
   * 
   * @param ledState - 1: force off 2: force blink 3: force on
   */
  public void setLimeLightLED(int ledState) {
    // NetworkTableInstance.getDefault().getTable("limelight-hood").getEntry("ledMode").setNumber(ledState);
    // TODO REMOVED HOOD LIMELIGHT
    NetworkTableInstance.getDefault().getTable("limelight-tag").getEntry("ledMode").setNumber(ledState);
    NetworkTableInstance.getDefault().getTable("limelight-host").getEntry("ledMode").setNumber(ledState);

  }

  /**
   * Method that logs values to dashboard
   */
  public void logToDashBoard() {
    if (Constants.FMSDETACHED)

      SmartDashboard.putNumber("Vision/tx", tx);

  }

  public double getID(String limelightName) {
    return LimelightHelpers.getFiducialID(limelightName);

  }

  public Pose3d getCameraPoseInTargetSpace(String limelightName) {
    if (!hasTarget(limelightName))
      return null;
    Pose3d BotPose3d = LimelightHelpers.getTargetPose3d_CameraSpace(Constants.Vision.LIMELIGHT_NAME);
    return new Pose3d(BotPose3d.getZ(), -BotPose3d.getX(), BotPose3d.getY(), new Rotation3d(
        BotPose3d.getRotation().getZ(), -BotPose3d.getRotation().getX(), BotPose3d.getRotation().getY()));
  }

  public static boolean isAligned = false;
  public static double yError = 0.0;

  private boolean isValidPose(Pose2d pose) {
    if (pose.getX() == 0 && pose.getY() == 0) {
      return false;
    }
    if (Math.abs(Swerve.getInstance().getAngularSpeed()) > 20.) {

      return false;
    }

    return DaisyMath.isInRange(pose.getY(), -5, FieldConstants.fieldWidth + 5)
        && DaisyMath.isInRange(pose.getX(), -5, FieldConstants.fieldLength + 5);
  }

  public int getNumTargets(String m_name) {
    return countStringOccurences(NetworkTableInstance.getDefault().getTable(m_name).getEntry("json").getString(""),
        "pts");
  }

  private int countStringOccurences(String str, String substr) {
    int occurrences = 0;
    for (int i = 0; i < str.length() - substr.length() + 1; i++) {
      if (str.substring(i, i + substr.length()).equals(substr)) {
        occurrences++;
      }
    }
    return occurrences;

  }

  public Pair<Pose2d, Double> getRobotPoseInFieldSpace(String limelightName) {


    LimelightHelpers.SetRobotOrientation(limelightName,
        Swerve.getInstance().isRed ? (Swerve.getInstance().getYaw().getDegrees() + 180.)
            : Swerve.getInstance().getYaw().getDegrees(),
        0, 0, 0, 0, 0);

    LimelightHelpers.PoseEstimate p = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    if (p.tagCount == 0)
      return null;
    if (p.rawFiducials[0].ambiguity > 0.9)
      return null;

    if (limelightName.equals("limelight-host")) {
      for (var tag : p.rawFiducials) {
        if (tag.id == 4) {
          tx_4 = -tag.txnc;
        }
        if (tag.id == 7) {
          tx_7 = -tag.txnc;

        }
      }
    }

    Pose2d pose = p.pose;

    if (!isValidPose(pose)) {
      return null;

    }

   

 
    if (Constants.FMSDETACHED)
      SmartDashboard.putNumber("Vision Timestamp", p.timestampSeconds);
    return new Pair<Pose2d, Double>(pose, p.timestampSeconds);

  }

  public void setLimelightPipelineIndex(int idx) {
    LimelightHelpers.setPipelineIndex(Constants.Vision.LIMELIGHT_NAME, idx);
  }

  public void setLimelightPipelineIndexAI(int idx) {
    LimelightHelpers.setPipelineIndex("limelight-ai", idx);
  }

  public int getLimelightPipelineIndex() {
    return (int) LimelightHelpers.getCurrentPipelineIndex(Constants.Vision.LIMELIGHT_NAME);
  }

  int cnt = 0;

  /**
   * returns if there is a target detected by the limelight
   */
  public boolean hasTarget(String limelightName) {
    return NetworkTableInstance.getDefault().getTable(limelightName).getEntry("tv").getDouble(0) > 0;
  }

  @Override
  public void periodic() {
    run();
  }

}
