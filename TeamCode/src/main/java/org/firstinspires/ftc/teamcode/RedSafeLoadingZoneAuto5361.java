package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name="Red Loading (Bad Al.)", group="Linear Opmode") // Assuming our teammate will do nothing
public class RedSafeLoadingZoneAuto5361 extends SafeLoadingZoneAuto5361 {

    @Override
    public boolean getIsBlueAlliance() {return false;}
}
