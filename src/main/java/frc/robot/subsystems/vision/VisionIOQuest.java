package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.LinkedList;
import java.util.List;

public class VisionIOQuest implements VisionIO {
  private final QuestNav questNav = new QuestNav();
  private static final AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public VisionIOQuest() {
    questNav.onCommandSuccess(
        response ->
            System.out.println("Pose reset succeeded for command ID: " + response.getCommandId()));
    questNav.onCommandFailure(
        response ->
            DriverStation.reportError("Pose reset failed: " + response.getErrorMessage(), false));

    // Warn when tracking is lost or acquired
    questNav.onTrackingAcquired(() -> System.out.println("Quest tracking acquired!"));
    questNav.onTrackingLost(() -> DriverStation.reportWarning("Quest tracking lost!", false));

    // Warn when battery reaches 20% or below
    questNav.onLowBattery(
        20, level -> DriverStation.reportWarning("Quest battery low: " + level + "%", false));
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = questNav.isConnected();

    questNav.commandPeriodic();
    // Get the latest pose data frames from the Quest
    PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

    List<PoseObservation> poseObservations = new LinkedList<>();
    // Loop over the pose data frames and send them to the pose estimator
    for (PoseFrame questFrame : questFrames) {
      // Make sure the Quest was tracking the pose for this frame
      if (questFrame.isTracking()) {
        // Get the pose of the Quest
        Pose3d questPose = questFrame.questPose3d();
        // Get timestamp for when the data was sent
        double timestamp = questFrame.dataTimestamp();

        // Transform by the mount pose to get your robot pose
        Pose3d robotPose = questPose.transformBy(VisionConstants.robotToQuest.inverse());
        if (!shouldReject(robotPose)) {
          // Add the measurement to our estimator
          poseObservations.add(
              new PoseObservation(timestamp, robotPose, 0.0, 1, 1, PoseObservationType.QUEST));

          // drive.addVisionMeasurement(robotPose.toPose2d(), timestamp, QUESTNAV_STD_DEVS);
        } else {
          DriverStation.reportWarning("Rejected pose from QuestNav: " + robotPose, false);
        }
      }
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }
  }

  public void setRobotPose(Pose2d pose) {
    // Assume this is the requested reset pose
    Pose3d robotPose = new Pose3d(pose);

    // Transform by the offset to get the Quest pose
    Pose3d questPose = robotPose.transformBy(VisionConstants.robotToQuest);

    // Set the robot pose in QuestNav
    questNav.setPose(questPose);
  }

  private boolean shouldReject(Pose3d pose) {
    return pose.getX() < 0.0
        || pose.getX() > FIELD_LAYOUT.getFieldLength()
        || pose.getY() < 0.0
        || pose.getY() > FIELD_LAYOUT.getFieldWidth();
  }
}
