package pedroPathing.subsystem;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


public class LimelightVision {
    Limelight3A limelight;
    HardwareMap hardwareMap = null;
    Pose3D defaultPose = new Pose3D(new Position(DistanceUnit.CM,0,0,0,0),new YawPitchRollAngles(AngleUnit.DEGREES,0,0,0,0));

    public LimelightVision(HardwareMap hm) {
        hardwareMap = hm;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);
    }

    public LLResult checkLimelight() {
        return limelight.getLatestResult();
    }
    public Pose getRobotPose(double robotYaw) {
        //double robotYaw = imu.getAngularOrientation().firstAngle;
        LLResult result = limelight.getLatestResult();
        limelight.updateRobotOrientation(robotYaw);
        Pose3D returnPose = defaultPose;
        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                returnPose = botpose_mt2;
            }
        }
        return new Pose(returnPose.getPosition().x*39.37,returnPose.getPosition().y*39.37,returnPose.getOrientation().getYaw(AngleUnit.RADIANS));

    }
    /*
    public class updatePoseWithLimelight extends CommandBase {
        boolean isDone = false;
        public updatePoseWithLimelight() {}

        @Override
        public void initialize() {}
    }*/

}

