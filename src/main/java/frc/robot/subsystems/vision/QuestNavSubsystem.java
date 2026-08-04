package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.QuestNav;
import gg.questnav.questnav.PoseFrame;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class QuestNavSubsystem extends SubsystemBase{
    private final QuestNav questNav = new QuestNav();
        @Override
        public void periodic() {
            questNav.commandPeriodic();
        }
        public Pose3d getRobotPose() {
            Pose3d robotPose = new Pose3d(); // Initialize with a default pose
            // Geometrical transform from the robot center to the Quest headset.
            // This should be set to the physical transform of your Quest headset on the robot
            // x = forward, y = left, z = up
            // Example: if 50cm forward, 10cm left, and 20cm up, then use (0.5, 0.1, 0.2)
            Transform3d ROBOT_TO_QUEST = new Transform3d(0, 0, 0, new Rotation3d());
    
            // Get the latest pose data frames from the Quest
            PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
    
            if (poseFrames.length > 0) {
                // Get the most recent Quest pose
                Pose3d questPose = poseFrames[poseFrames.length - 1].questPose3d();
    
                // Transform by the mount pose to get your robot pose
                robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());
            }
        return robotPose;
    }
    public void setRobotPose(Pose3d pose) {
        // Geometrical transform from the robot center to the Quest headset
        Transform3d ROBOT_TO_QUEST = new Transform3d(0, 0, 0, new Rotation3d());

        // Assume this is the requested reset pose
        Pose3d robotPose = new Pose3d(pose.getTranslation(), pose.getRotation());

        // Transform by the offset to get the Quest pose
        Pose3d questPose = robotPose.transformBy(ROBOT_TO_QUEST);

        // Set the robot pose in QuestNav
        questNav.setPose(questPose);
    }
}