package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "left_front";
        FollowerConstants.leftRearMotorName = "left_back";
        FollowerConstants.rightFrontMotorName = "right_front";
        FollowerConstants.rightRearMotorName = "right_back";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 14.42424;

        FollowerConstants.xMovement = 76.4931;
        FollowerConstants.yMovement = 58.7995;

        FollowerConstants.forwardZeroPowerAcceleration = -36.5691;
        FollowerConstants.lateralZeroPowerAcceleration = -76.5822;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.09,0,0.009,0);
        FollowerConstants.useSecondaryTranslationalPID = true;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.01,0,0.0001,0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(0.65,0,0.01,0);
        FollowerConstants.useSecondaryHeadingPID = true;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.1,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.009,0,0.001,0,0);
        FollowerConstants.useSecondaryDrivePID = true;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.01,0,0.0001,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 4;
        FollowerConstants.centripetalScaling = 0.0004;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;

        FollowerConstants.useBrakeModeInTeleOp = true;
    }
}
