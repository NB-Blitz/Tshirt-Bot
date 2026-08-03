import gg.questnav.questnav.QuestNav;

package frc.robot.subsystems.vision;
public class QuestNav {
    private final QuestNav questNav = new QuestNav();
    @Override
    public void periodic() {
        questNav.commandPeriodic();
    }
}
