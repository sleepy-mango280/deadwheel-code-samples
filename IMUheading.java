@TeleOp(name = "IMU Test")
public class IMUTest extends LinearOpMode {

    IMU imu;

    @Override
    public void runOpMode() {

        // initialize the IMU
        imu = hardwareMap.get(IMU.class, "imu");

        // tell the IMU how it's physically mounted on your robot
        IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        );

        imu.initialize(parameters);
        imu.resetYaw();

        waitForStart();

        while (opModeIsActive()) {

            // get the current heading in radians
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            telemetry.addData("Heading (radians)", heading);
            telemetry.addData("Heading (degrees)", Math.toDegrees(heading));
            telemetry.update();
        }
    }
}
