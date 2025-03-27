package pedroPathing.subsystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


public class LimelightVision {
    Limelight3A limelight;
    HardwareMap hardwareMap = null;

    public LimelightVision(HardwareMap hm) {
        hardwareMap = hm;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
    }

    public LLResult checkLimelight() {
        return limelight.getLatestResult();
    }
}

