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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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
                
            
            
            .
public void buildPaths() {
        
}

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

        /*
        case 2: // Wait until the robot is near the first sample pickup position
            if (!follower.isBusy()) {
                follower.followPath(three, true);
                setPathState(3);
            }
            break;

        case 3: // Wait until the robot returns to the scoring position
            if (!follower.isBusy()) {
                follower.followPath(four, true);
                setPathState(4);
            }
            break;

        case 4: // Wait until the robot is near the second sample pickup position
            if (!follower.isBusy()) {
                follower.followPath(five, true);
                setPathState(5);
            }
            break;

        case 5: // Wait until the robot returns to the scoring position
            if (!follower.isBusy()) {
                follower.followPath(six, true);
                setPathState(6);
            }
            break;

        case 6: // Wait until the robot is near the third sample pickup position
            if (!follower.isBusy()) {
                follower.followPath(seven, true);
                setPathState(7);
            }
            break;

        case 7: // Wait until the robot returns to the scoring position
            if (!follower.isBusy()) {
                follower.followPath(eight, true);
                setPathState(8);
            }
            break;

        case 8: // Wait until the robot returns to the scoring position
            if (!follower.isBusy()) {
                follower.followPath(nine, true);
                setPathState(9);
            }
            break;

        case 9: // Wait until the robot returns to the scoring position
            if (!follower.isBusy()) {
                follower.followPath(ten, true);
                setPathState(9);
            }
            break;

        case 10: // Wait until the robot returns to the scoring position
            if (!follower.isBusy()) {
                follower.followPath(eleven, true);
                setPathState(10);
            }
            break;

        case 11: // Wait until the robot returns to the scoring position
            if (!follower.isBusy()) {
                follower.followPath(twelve, true);
                setPathState(11);
            }
            break;

        case 12: // Wait until the robot returns to the scoring position
            if (!follower.isBusy()) {
                follower.followPath(thirteen, true);
                setPathState(12);
            }
            break;

        case 13: // Wait until the robot returns to the scoring position
            if (!follower.isBusy()) {
                follower.followPath(fourteen, true);
                setPathState(13);
            }
            break;

        case 14: // Wait until the robot returns to the scoring position
            if (!follower.isBusy()) {
                follower.followPath(fifteen, true);
                setPathState(14);
            }
            break;

        case 15: // Wait until the robot returns to the scoring position
            if (!follower.isBusy()) {
                follower.followPath(sixteen, true);
                setPathState(15);
            }
            break;

        case 16: // Wait until the robot returns to the scoring position
            if (!follower.isBusy()) {
                follower.followPath(seventeen, true);
                setPathState(16);
            }
            break;

        case 17: // Wait until the robot returns to the scoring position
            if (!follower.isBusy()) {
                follower.followPath(eighteen, true);
                setPathState(17);
            }
            break;

        case 18: // Wait until the robot returns to the scoring position
            if (!follower.isBusy()) {
                follower.followPath(nineteen, true);
                setPathState(18);
            }
            break;

        case 19: // Wait until the robot is near the parking position
            if (!follower.isBusy()) {
                setPathState(-1); // End the autonomous routine
            }
            break;

         */
    }
}

public void setPathState(int pState) {
    pathState = pState;
    pathTimer.resetTimer();
}

@Override
public void init() {
    pathTimer = new Timer();
    Constants.setConstants(FConstants.class, LConstants.class);
    follower = new Follower(hardwareMap);
    follower.setStartingPose(startPose);
    buildPaths();
}

@Override
public void loop() {
    follower.update();
    autonomousPathUpdate();
    telemetry.addData("Path State", pathState);
    telemetry.addData("Position", follower.getPose().toString());
    telemetry.update();
}}
