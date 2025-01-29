package pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

private final Pose startPose = new Pose(0.355,71.645, Math.toRadians(0));  // Starting position

private Path scorePreload, park;
private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

public void buildPaths() {
    // Path for scoring preload
    one = new Path(new BezierLine(new Point(0.355,71.645,Point.CARTESIAN), new Point(48.768,78.030,Point.CARTESIAN)));
    one.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
  
    two = follower.pathBuilder()
            .addPath(new BezierCurve(new Point(48.768,78.030,Point.CARTESIAN),new Point(14.187,46.818,Point.CARTESIAN),new Point(61.714,32.099,Point.CARTESIAN))
            .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
            .build();

    three = follower.pathBuilder()
            .addPath(new BezierCurve(new Point(61.714, 32.099, Point.CARTESIAN), new Point(61.537, 15.429, Point.CARTESIAN),new Point(4.256, 22.167, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
            .build();

    four = follower.pathBuilder()
            .addPath(new BezierCurve(
          new Point(4.256, 22.167, Point.CARTESIAN),
          new Point(99.310, 22.522, Point.CARTESIAN),
          new Point(64.729, 5.498, Point.CARTESIAN),
          new Point(4.433, 9.576, Point.CARTESIAN)
        ))
            .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
            .build();

    five = follower.pathBuilder()
            .addPath(new BezierCurve(
          new Point(4.433, 9.576, Point.CARTESIAN),
          new Point(86.187, 10.108, Point.CARTESIAN),
          new Point(58.522, 0.177, Point.CARTESIAN)
        ))
            .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
            .build();

    six = follower.pathBuilder()
            .addPath(new BezierLine(
          new Point(58.522, 0.177, Point.CARTESIAN),
          new Point(7.803, 0.887, Point.CARTESIAN)
        ))
            .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
            .build();

    seven = follower.pathBuilder()
            .addPath(new BezierLine(
          new Point(7.803, 0.887, Point.CARTESIAN),
          new Point(17.379, 31.921, Point.CARTESIAN)
        ))
            .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
            .build();

    eight = follower.pathBuilder()
            .addPath(new BezierLine(
          new Point(17.379, 31.921, Point.CARTESIAN),
          new Point(2.483, 31.567, Point.CARTESIAN)
        ))
            .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
            .build();

    nine = follower.pathBuilder()
            .addPath(new BezierCurve(
          new Point(2.483, 31.567, Point.CARTESIAN),
          new Point(16.670, 71.113, Point.CARTESIAN),
          new Point(48.768, 74.837, Point.CARTESIAN)
        ))
            .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
            .build();

    ten = follower.pathBuilder()
            .addPath(new BezierCurve(
          new Point(48.768, 74.837, Point.CARTESIAN),
          new Point(16.670, 71.291, Point.CARTESIAN),
          new Point(18.266, 31.567, Point.CARTESIAN)
        ))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
            .build();

    eleven = follower.pathBuilder()
            .addPath(new BezierLine(
          new Point(18.266, 31.567, Point.CARTESIAN),
          new Point(3.369, 31.921, Point.CARTESIAN)
        ))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
            .build();

    twelve = follower.pathBuilder()
            .addPath(new BezierCurve(
          new Point(3.369, 31.921, Point.CARTESIAN),
          new Point(17.025, 70.936, Point.CARTESIAN),
          new Point(48.946, 71.291, Point.CARTESIAN)
        ))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
            .build();

    thirteen = follower.pathBuilder()
            .addPath(new BezierCurve(
          new Point(48.946, 71.291, Point.CARTESIAN),
          new Point(17.202, 71.113, Point.CARTESIAN),
          new Point(17.557, 31.744, Point.CARTESIAN)
        ))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
            .build();

    fourteen = follower.pathBuilder()
            .addPath(new BezierLine(
          new Point(17.557, 31.744, Point.CARTESIAN),
          new Point(3.015, 31.389, Point.CARTESIAN)
        ))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
            .build();

    fifteen = follower.pathBuilder()
            .addPath(new BezierCurve(
          new Point(3.015, 31.389, Point.CARTESIAN),
          new Point(7.803, 62.069, Point.CARTESIAN),
          new Point(48.414, 67.567, Point.CARTESIAN)
        ))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
            .build();

    sixteen = follower.pathBuilder()
            .addPath(new BezierCurve(
          new Point(48.414, 67.567, Point.CARTESIAN),
          new Point(15.429, 65.261, Point.CARTESIAN),
          new Point(17.557, 31.212, Point.CARTESIAN)
        ))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
            .build();

    seventeen = follower.pathBuilder()
            .addPath(new BezierLine(
          new Point(17.557, 31.212, Point.CARTESIAN),
          new Point(3.015, 31.212, Point.CARTESIAN)
        ))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
            .build();

    eighteen = follower.pathBuilder()
            .addPath(new BezierCurve(
          new Point(3.015, 31.212, Point.CARTESIAN),
          new Point(10.463, 62.069, Point.CARTESIAN),
          new Point(48.059, 63.665, Point.CARTESIAN)
        ))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
            .build();

    eleven = follower.pathBuilder()
            .addPath(new BezierCurve(
          new Point(48.059, 63.665, Point.CARTESIAN),
          new Point(11.350, 64.552, Point.CARTESIAN),
          new Point(8.512, 29.261, Point.CARTESIAN)
        ))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
            .build();

}

public void autonomousPathUpdate() {
    switch (pathState) {
        case 0: // Move from start to scoring position
            follower.followPath(one);
            setPathState(1);
            break;

        case 1: // Wait until the robot is near the scoring position
            if (!follower.isBusy()) {
                follower.followPath(two, true);
                setPathState(2);
            }
            break;

        case 2: // Wait until the robot is near the first sample pickup position
            if (!follower.isBusy()) {
                follower.followPath(three, true);
                setPathState(3);
            }
            break;

        case 3: // Wait until the robot returns to the scoring position
            if (!follower.isBusy()) {
                follower.followPath(fourth, true);
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
}
