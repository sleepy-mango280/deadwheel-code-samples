import com.acmerobotics.dashboard.config.Config;

@Config
@TeleOp(name = "PD Controller XY")
public class PDControllerXY extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight;
    DcMotor parallelEncoder;
    DcMotor perpEncoder;
    IMU imu;

    static final double TICKS_PER_REV  = 2000;
    static final double WHEEL_DIAMETER = 1.378;
    static final double CIRCUMFERENCE  = Math.PI * WHEEL_DIAMETER;

    public static double kP = 0.05;
    public static double kD = 0.005;

    public static double TARGET_X = 24.0;  // inches
    public static double TARGET_Y = 24.0;  // inches

    @Override
    public void runOpMode() {

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        parallelEncoder = hardwareMap.get(DcMotor.class, "parallel");
        perpEncoder     = hardwareMap.get(DcMotor.class, "perp");

        parallelEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parallelEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        perpEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        );
        imu.initialize(parameters);
        imu.resetYaw();

        // odometry tracking
        int lastParallel = 0;
        int lastPerp     = 0;
        double x         = 0.0;
        double y         = 0.0;

        // PD memory
        double lastErrorX = 0.0;
        double lastErrorY = 0.0;

        waitForStart();

        while (opModeIsActive()) {

            // -- odometry update --
            int currentParallel = parallelEncoder.getCurrentPosition();
            int currentPerp     = perpEncoder.getCurrentPosition();
            double heading      = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            int deltaParallel = currentParallel - lastParallel;
            int deltaPerp     = currentPerp     - lastPerp;

            double deltaForward = (deltaParallel / TICKS_PER_REV) * CIRCUMFERENCE;
            double deltaStrafe  = (deltaPerp     / TICKS_PER_REV) * CIRCUMFERENCE;

            double deltaX = (deltaForward * Math.cos(heading)) - (deltaStrafe * Math.sin(heading));
            double deltaY = (deltaForward * Math.sin(heading)) + (deltaStrafe * Math.cos(heading));

            x += deltaX;
            y += deltaY;

            lastParallel = currentParallel;
            lastPerp     = currentPerp;

            // -- PD controller --
            double errorX = TARGET_X - x;
            double errorY = TARGET_Y - y;

            double pTermX = errorX * kP;
            double pTermY = errorY * kP;

            double dTermX = (errorX - lastErrorX) * kD;
            double dTermY = (errorY - lastErrorY) * kD;

            double powerX = pTermX + dTermX;
            double powerY = pTermY + dTermY;

            lastErrorX = errorX;
            lastErrorY = errorY;

            // -- mecanum drive mixing --
            frontLeft.setPower(powerY  + powerX);
            frontRight.setPower(powerY - powerX);
            backLeft.setPower(powerY   - powerX);
            backRight.setPower(powerY  + powerX);

            telemetry.addData("X (in)",    x);
            telemetry.addData("Y (in)",    y);
            telemetry.addData("errorX",    errorX);
            telemetry.addData("errorY",    errorY);
            telemetry.addData("powerX",    powerX);
            telemetry.addData("powerY",    powerY);
            telemetry.update();
        }
    }
}
