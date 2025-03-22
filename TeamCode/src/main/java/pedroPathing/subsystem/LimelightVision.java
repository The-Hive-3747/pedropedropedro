package pedroPathing.subsystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;


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

