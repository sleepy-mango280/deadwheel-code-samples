@TeleOp(name = "Odometry Test")
public class OdometryTest extends LinearOpMode {

    DcMotor parallelEncoder;
    DcMotor perpEncoder;
    IMU imu;

    static final double TICKS_PER_REV  = 2000;
    static final double WHEEL_DIAMETER = 1.378;
    static final double CIRCUMFERENCE  = Math.PI * WHEEL_DIAMETER;

    @Override
    public void runOpMode() {

        parallelEncoder = hardwareMap.get(DcMotor.class, "parallel");
        perpEncoder     = hardwareMap.get(DcMotor.class, "perp");
        imu             = hardwareMap.get(IMU.class, "imu");

        parallelEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parallelEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        perpEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        );
        imu.initialize(parameters);
        imu.resetYaw();

        int lastParallel = 0;
        int lastPerp     = 0;

        double x       = 0.0;
        double y       = 0.0;

        waitForStart();

        while (opModeIsActive()) {

            // step 1 -- read sensors
            int currentParallel = parallelEncoder.getCurrentPosition();
            int currentPerp     = perpEncoder.getCurrentPosition();
            double heading      = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // step 2 -- delta ticks
            int deltaParallel = currentParallel - lastParallel;
            int deltaPerp     = currentPerp     - lastPerp;

            // step 3 -- ticks to inches
            double deltaForward = (deltaParallel / TICKS_PER_REV) * CIRCUMFERENCE;
            double deltaStrafe  = (deltaPerp     / TICKS_PER_REV) * CIRCUMFERENCE;

            // step 4 -- local to global coordinate transform
            double deltaX = (deltaForward * Math.cos(heading)) - (deltaStrafe * Math.sin(heading));
            double deltaY = (deltaForward * Math.sin(heading)) + (deltaStrafe * Math.cos(heading));

            // step 5 -- accumulate position
            x += deltaX;
            y += deltaY;

            // step 6 -- update previous readings
            lastParallel = currentParallel;
            lastPerp     = currentPerp;

            telemetry.addData("X (inches)",        x);
            telemetry.addData("Y (inches)",        y);
            telemetry.addData("Heading (degrees)", Math.toDegrees(heading));
            telemetry.update();
        }
    }
}
