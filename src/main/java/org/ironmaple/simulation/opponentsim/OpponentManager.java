package org.ironmaple.simulation.opponentsim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import org.ironmaple.simulation.gamepieces.GamePiece;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class OpponentManager {
    protected Optional<List<SmartOpponent>> redOpponents;
    protected Optional<List<SmartOpponent>> blueOpponents;

    public void registerOpponent(SmartOpponent opponent, DriverStation.Alliance alliance) {
        if (alliance == DriverStation.Alliance.Blue) blueOpponents.get().add(opponent);
        else redOpponents.get().add(opponent);
    }

    public List<SmartOpponent> getOpponents(DriverStation.Alliance alliance) {
        return alliance.equals(DriverStation.Alliance.Blue) ? blueOpponents.get() : redOpponents.get();
    }

    public List<SmartOpponent> getOpponents() {
        List<SmartOpponent> allOpponents = new ArrayList<>(blueOpponents.get());
        allOpponents.addAll(redOpponents.get());
        return allOpponents;
    }

    public List<Pose2d> getOpponentTargets(DriverStation.Alliance alliance) {
        List<Pose2d> targets = new ArrayList<>();
        if (alliance == DriverStation.Alliance.Blue) {
            for (SmartOpponent blueOpponent : blueOpponents.get()) {
                targets.add(blueOpponent.getTarget());
            }
        } else {
            for (SmartOpponent redOpponent : redOpponents.get()) {
                targets.add(redOpponent.getTarget());
            }
        }
        return targets;
    }

    public List<Pose2d> getOpponentTargets() {
        List<SmartOpponent> allOpponents = getOpponents();
        List<Pose2d> targets = new ArrayList<>();
        for (SmartOpponent opponent : allOpponents) {
            targets.add(opponent.getTarget());
        }
        return targets;
    }

    public List<Pose2d> getOpponentPoses(DriverStation.Alliance alliance) {
        List<Pose2d> poses = new ArrayList<>();
        if (alliance == DriverStation.Alliance.Blue) {
            for (SmartOpponent blueOpponent : blueOpponents.get()) {
                poses.add(blueOpponent.getPose());
            }
        } else {
            for (SmartOpponent redOpponent : redOpponents.get()) {
                poses.add(redOpponent.getPose());
            }
        }
        return poses;
    }

    public List<Pose2d> getOpponentPoses() {
        List<SmartOpponent> allOpponents = getOpponents();
        List<Pose2d> allPoses = new ArrayList<>();
        for (SmartOpponent opponent : allOpponents) {
            allPoses.add(opponent.getPose());
        }
        return allPoses;
    }

    /**
     * Used by the opponent bot, season-specific.
     */
    protected Pose2d getStartingPose(DriverStation.Alliance alliance, int id) {
        return new Pose2d();
    }

    /**
     * Used by the opponent bot, season-specific.
     */
    protected Pose2d getQueeningPose(DriverStation.Alliance alliance, int id) {
        return new Pose2d();
    }

    /**
     * Used by the opponent bot, season-specific.
     */
    protected Pose2d getNextScoreTarget(DriverStation.Alliance alliance) {
        return new Pose2d();
    }

    // idk what to do with this yet
    protected String getNextScoreTargetName(DriverStation.Alliance alliance, int id) {
        return "";
    }

    /**
     * Used by the opponent bot, season-specific.
     */
    protected Pose2d getNextCollectTarget(DriverStation.Alliance alliance) {
        return new Pose2d();
    }

    // Simple obstacle rectangle class
    public static class ObstacleRect {
        public double x, y, width, height, centerX, centerY;

        public ObstacleRect(double x, double y, double width, double height) {
            this.x = x;
            this.y = y;
            this.width = width;
            this.height = height;
            this.centerX = x + width / 2;
            this.centerY = y + height / 2;
        }

        public boolean intersectsLine(double sx, double sy, double ex, double ey, double buffer) {
            double expandedX = x - buffer, expandedY = y - buffer;
            double expandedW = width + 2 * buffer, expandedH = height + 2 * buffer;
            return !(ey < expandedY || sy > expandedY + expandedH ||
                    ex < expandedX || sx > expandedX + expandedW);
        }
    }

}
