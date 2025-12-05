package org.ironmaple.simulation.opponentsim;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.ironmaple.simulation.opponentsim.pathfinding.MapleADStar;
import org.ironmaple.utils.FieldMirroringUtils;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class OpponentManager {
    protected List<Pair<Pose2d, Integer>> robotTargets = new ArrayList<>();
    protected List<Pair<Integer, Pose2d>> blueStartingPositions = new ArrayList<>();
    protected List<Pair<Integer, Pose2d>> redStartingPositions = new ArrayList<>();
    protected List<SmartOpponent> redOpponents = new ArrayList<>();
    protected List<SmartOpponent> blueOpponents = new ArrayList<>();

    /**
     * MapleSim Opponent sim currently relies on Pathplanner with a modified pathfinder.
     * This is to be changed soon. ^TM

     */
    public OpponentManager() {
        PathfindingCommand.warmupCommand().schedule();
    }

    public void registerOpponent(SmartOpponent opponent, DriverStation.Alliance alliance) {
        if (alliance == DriverStation.Alliance.Blue) blueOpponents.add(opponent);
        else redOpponents.add(opponent);
    }

    public List<SmartOpponent> getOpponents(DriverStation.Alliance alliance) {
        return alliance.equals(DriverStation.Alliance.Blue) ? blueOpponents : redOpponents;
    }

    public List<SmartOpponent> getOpponents() {
        List<SmartOpponent> allOpponents = new ArrayList<>(blueOpponents);
        allOpponents.addAll(redOpponents);
        return allOpponents;
    }

    public List<Pose2d> getOpponentTargets(DriverStation.Alliance alliance) {
        List<Pose2d> targets = new ArrayList<>();
        if (alliance == DriverStation.Alliance.Blue) {
            for (SmartOpponent blueOpponent : blueOpponents) {
                targets.add(blueOpponent.getTargetTask().getFirst());
            }
        } else {
            for (SmartOpponent redOpponent : redOpponents) {
                targets.add(redOpponent.getTargetTask().getFirst());
            }
        }
        return targets;
    }

    public List<Pose2d> getOpponentTargets() {
        List<SmartOpponent> allOpponents = getOpponents();
        List<Pose2d> targets = new ArrayList<>();
        for (SmartOpponent opponent : allOpponents) {
            targets.add(opponent.getTargetTask().getFirst());
        }
        return targets;
    }

    public List<Pose2d> getOpponentPoses(DriverStation.Alliance alliance) {
        List<Pose2d> poses = new ArrayList<>();
        if (alliance == DriverStation.Alliance.Blue) {
            for (SmartOpponent blueOpponent : blueOpponents) {
                poses.add(blueOpponent.getPose());
            }
        } else {
            for (SmartOpponent redOpponent : redOpponents) {
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
     * Used by the opponent bot, season-specific. id 1-3, id is the pose ID
     */
    protected Pose2d getStartingPose(DriverStation.Alliance alliance, int id) {
        if (alliance == DriverStation.Alliance.Blue) {
            for (Pair<Integer, Pose2d> pair : blueStartingPositions) {
                if (pair.getFirst() == id) return pair.getSecond();
            } // If not found, add it to the list
            blueStartingPositions.add(new Pair<>(id, ifShouldFlip(alliance, ManagerConstants.STARTING_POSITIONS[blueStartingPositions.size()])));
        } else {
            for (Pair<Integer, Pose2d> pair : redStartingPositions) {
                if (pair.getFirst() == id) return pair.getSecond();
            } // If not found, add it to the list
            redStartingPositions.add(new Pair<>(id, ifShouldFlip(alliance, ManagerConstants.STARTING_POSITIONS[redStartingPositions.size()])));
        }
        return getStartingPose(alliance, id);
    }

    /**
     * Used by the opponent bot, season-specific. id 1-6, id is the pose ID
     */
    public Pose2d getQueeningPose(int id) {
        return ManagerConstants.ROBOT_QUEENING_POSITIONS[id - 1];
    }

    /**
     * Used by the opponent bot, season-specific.
     */
    public Pair<Pose2d, String> getNextScoreTarget(DriverStation.Alliance alliance, int id) {
        return Pair.of(new Pose2d(), "Task");
    }

    /**
     * Used by the opponent bot, season-specific.
     */
    public Pair<Pose2d, String> getNextCollectTarget(DriverStation.Alliance alliance, int id) {
        return Pair.of(new Pose2d(), "Task");
    }

    /**
     *
     * @param id current robot ID, used to remove self as an obstacle.
     * @return
     */
    public List<Pair<Translation2d, Translation2d>> getObstacles(int id) {
        List<SmartOpponent> robots = getOpponents();
        List<Pair<Translation2d, Translation2d>> obs = new ArrayList<>(robots.size() + 1);
        Translation2d offset = new Translation2d(.5, .5);
        for (SmartOpponent robot : robots) {
            if (robot.id != id) {
                obs.add(new Pair<>(
                        robot.getPose().getTranslation().plus(offset),
                        robot.getPose().getTranslation().minus(offset)));
            }
        }
        return obs;
    }

    public static class ManagerConstants {
        // Add more starting positions externally if needed. This only covers a regular match for now.
        public static final Pose2d[] ROBOT_QUEENING_POSITIONS = new Pose2d[] {
                        new Pose2d(-6, 0, new Rotation2d()),
                        new Pose2d(-5, 0, new Rotation2d()),
                        new Pose2d(-4, 0, new Rotation2d()),
                        new Pose2d(-3, 0, new Rotation2d()),
                        new Pose2d(-2, 0, new Rotation2d())
                };
        public static final Pose2d[] STARTING_POSITIONS = new Pose2d[]
                {
                        new Pose2d(15, 6, Rotation2d.fromDegrees(180)),
                        new Pose2d(15, 4, Rotation2d.fromDegrees(180)),
                        new Pose2d(15, 2, Rotation2d.fromDegrees(180))
                };
    }

    /**
     * @param pose
     * @return
     */
    public Pose2d ifShouldFlip(DriverStation.Alliance alliance, Pose2d pose) {
        if (alliance == DriverStation.Alliance.Red) {
            return pose;
        } else {
            return new Pose2d(
                    FieldMirroringUtils.flip(pose.getTranslation()),
                    FieldMirroringUtils.flip(pose.getRotation()));
        }
    }

}
