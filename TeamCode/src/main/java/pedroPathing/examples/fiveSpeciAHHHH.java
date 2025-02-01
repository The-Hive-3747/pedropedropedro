package pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.pedropathing.commands.FollowPath;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.arcrobotics.ftclib.command.*;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "five speci ahh", group = "Examples")
public class fiveSpeciAHHHH extends OpMode {
    
    public static Pose startPose = new Pose(8,61.5, Math.toRadians(0));

    
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    public static PathChain preload() {
        return new PathBuilder()
            .addPath(newBezierLine(
                new Point(8, 61.5, Point.CARTESIAN),
                new Point(41, 75, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(0))
            .build();

    public static PathChain pushSamples() {
        return new PathBuilder()
            .addPath(new BezierCurve( // after place, goes to first sample
                new Point(41, 75, Point.CARTESIAN),
                new Point(0.7, 43.8, Point.CARTESIAN),
                new Point(65, 28, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(0))
            .addPath(new BezierCurve( // pushes first sample
                new Point(65, 28, Point.CARTESIAN),
                new Point(65, 22, Point.CARTESIAN),
                new Point(15, 25, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(0))
            .addPath(new BezierLine( // goes to second sample
                new Point(15, 25, Point.CARTESIAN),
                new Point(65, 23, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(0))
            .addPath(new BezierCurve( // pushes second sample
                new Point(65, 23, Point.CARTESIAN),
                new Point(65, 10, Point.CARTESIAN),
                new Point(15, 15, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(0))
            .addPath(new BezierLine( // goes to third sample
                new Point(15, 15, Point.CARTESIAN),
                new Point(65, 13, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(0))
            .addPath(new BezierCurve( // pushes third sample
                new Point(65, 13, Point.CARTESIAN),
                new Point(65, 2, Point.CARTESIAN),
                new Point(15, 3, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(0))
            .build();
                
            
/*
public void autonomousPathUpdate() {
    switch (pathState) {
        case 0: // Move from start to scoring position
            follower.followPath(one);
            setPathState(1);
            break;

        case 1: // Wait until the robot is near the scoring position
            if (!follower.isBusy()) {
                follower.followPath(pushSamples, true);
                setPathState(2);
            }
            break;

        case 2: // Wait until the robot is near the scoring position
            if (!follower.isBusy()) {
                setPathState(-1); // End the autonomous routine
            }
            break;
    }
}

public void setPathState(int pState) {
    pathState = pState;
    pathTimer.resetTimer();
}
*/
@Override
public void init() {
    //pathTimer = new Timer();
    Constants.setConstants(FConstants.class, LConstants.class);
    follower = new Follower(hardwareMap);
    follower.setStartingPose(startPose);
    //buildPaths();
}

@Override
public void start() {
    schedule(
        new SequentialCommandGroup(
            new FollowPath(preload, true)
            new WaitCommand(200).andThen(new FollowPath(pushSamples(), true))
        )
        );
}}

@Override
public void stop() {
    CommandScheduler.getInstance().reset();
}
