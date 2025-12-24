// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.ShooterConstants;
import frc.robot.subsystems.CoordinationSubsystem.AbsoluteStates;

public class ShooterSubsystem extends SubsystemBase {

  private static ShooterSubsystem INSTANCE;
  private static VisionSubsystem vision = RobotContainer.vision;

  public enum ShooterState {
    OVERRIDE,
    IDLING,
    SHOOTING
  }

  public enum AngleState {
    OVERRIDE,
    ZEROING,
    STORING,
    IDLING,
    AIMING
  }


  private ShooterState currentShooterState = ShooterState.IDLING;
  private AngleState currentAngleState = AngleState.STORING;
  
  private TalonFX motorWheelLeader = new TalonFX(ShooterConstants.FRONT_WHEEL_MOTOR_ID, RobotConstants.CANIVORE_BUS);
  private TalonFX motorWheelFollower = new TalonFX(ShooterConstants.BACK_WHEEL_MOTOR_ID, RobotConstants.CANIVORE_BUS);
  private TalonFX motorAngle = new TalonFX(ShooterConstants.ANGLE_MOTOR_ID, RobotConstants.CANIVORE_BUS);

  private VoltageOut voltageRequest;

  private boolean wheelsSpunUp = false;

  private double flywheelVoltage = 0;
  private double angleSetpoint = 0;
  private double angleVoltage = 0;

  // Motion Magic parameters
  private LoggedNetworkNumber logMMVeloc = new LoggedNetworkNumber("/Tuning/Shooter/Angle/Velocity", 24);
  private LoggedNetworkNumber logMMAccel = new LoggedNetworkNumber("/Tuning/Shooter/Angle/Acceleration", 24);
  private LoggedNetworkNumber logMMJerk = new LoggedNetworkNumber("/Tuning/Shooter/Angle/Jerk", 1000);
  private double mmVelocity = logMMVeloc.get();
  private double mmAcceleration = logMMAccel.get();
  private double mmJerk = logMMJerk.get();

  // Feedforward and PIDF constants
  private LoggedNetworkNumber logMMKS = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kS", 0.15);
  private LoggedNetworkNumber logMMKV = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kV", 0.33);
  private LoggedNetworkNumber logMMKA = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kA", 0.1);
  private LoggedNetworkNumber logMMKP = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kP", 6);
  private LoggedNetworkNumber logMMKI = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kI", 0);
  private LoggedNetworkNumber logMMKD = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kD", 0);
  private LoggedNetworkNumber logMMKG = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kG", 0.05);
  private double mmKS = logMMKS.get();
  private double mmKV = logMMKV.get();
  private double mmKA = logMMKA.get();
  private double mmKP = logMMKP.get();
  private double mmKI = logMMKI.get();
  private double mmKD = logMMKD.get();
  private double mmKG = logMMKG.get();
  private DynamicMotionMagicVoltage mmRequest;

  private LoggedNetworkNumber logVKS = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/kS", 0.1);
  private LoggedNetworkNumber logVKV = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/kV", 0.1);
  private LoggedNetworkNumber logVKA = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/kA", 0.01);
  private LoggedNetworkNumber logVKP = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/kP", 0.6);
  private LoggedNetworkNumber logVKFF = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/kFF", 1);
  private LoggedNetworkNumber logVAccel = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/Acceleration", 100);
  private LoggedNetworkNumber logVJerk = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/Jerk", 4000);
  private double vKS = logVKS.get();
  private double vKV = logVKV.get();
  private double vKA = logVKA.get();
  private double vKP = logVKP.get();
  private double vKFF = logVKFF.get();
  private double vAccel = logVAccel.get();
  private double vJerk = logVJerk.get();
  private final int vSlot = 0;
  private final MotionMagicVelocityVoltage vRequest;
  //private LoggedNetworkNumber logVoltageSetpoint = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/Voltage Setpoint", 4); // used to be 45
  private double voltageSetpoint = 0;//logVoltageSetpoint.get(); Uncomment for voltage tuning
  // private LoggedNetworkNumber logVelocitySetpoint = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/Velocity Setpoint", 45); // used to be 45
   private double velocitySetpoint = 0; //logVelocitySetpoint.get();  // Uncomment for velocity tuning

  private double aimingSetpoint = 0;

  private LoggedNetworkNumber logDistanceAway = new LoggedNetworkNumber("/Tuning/Shooter/Distance", 1);
  private double distanceAway = logDistanceAway.get();
  private double calculatedDistance = 0;
  private double allowedDistanceDiff = 0.1;

 /* distance, height */
  InterpolatingDoubleTreeMap downtownPowerMapVoltage = new InterpolatingDoubleTreeMap(); // Add a position later
  InterpolatingDoubleTreeMap downtownAngleMapVoltage = new InterpolatingDoubleTreeMap(); // Add a position later

  InterpolatingDoubleTreeMap downtownPowerMapVelocity = new InterpolatingDoubleTreeMap(); // Add a position later
  InterpolatingDoubleTreeMap downtownAngleMapVelocity = new InterpolatingDoubleTreeMap(); // Add a position later

  private final double maxHoodVoltage;

  private boolean onlyOnChange = false;

  private boolean isZeroingDone = false;
  
  public boolean changeOnce = true;

  public ShooterSubsystem() {

    configureShooterMotors();
    configureAngleMotor();

    mmRequest = new DynamicMotionMagicVoltage(0, mmVelocity, mmAcceleration, mmJerk);
    vRequest = new MotionMagicVelocityVoltage(0).withSlot(vSlot);
    voltageRequest = new VoltageOut(0);

    maxHoodVoltage = 1;
    
    isZeroingDone = false;
    addDataToMap();
  }

  public void addDataToMap() {
    // VOLTAGE
    downtownPowerMapVoltage.put(.9017, 4.85);
    downtownAngleMapVoltage.put(.9017, 4.15);

    downtownPowerMapVoltage.put(1.1557, 5.15);
    downtownAngleMapVoltage.put(1.1557, 4.5);

    downtownPowerMapVoltage.put(2.9845, 7.7);
    downtownAngleMapVoltage.put(2.9845, 1.4);

    // VELOCITY
    downtownPowerMapVelocity.put(1.0922, 33.0);
    downtownAngleMapVelocity.put(1.0922, 11.5125381); //1.89

    downtownPowerMapVelocity.put(1.3843, 34.0);
    downtownAngleMapVelocity.put(1.3843, 14.009967); //2.3

    downtownPowerMapVelocity.put(1.7018, 34.0);
    downtownAngleMapVelocity.put(1.7018, 18.276387); //3.0

    downtownPowerMapVelocity.put(2.0066, 37.5);
    downtownAngleMapVelocity.put(2.0066, 16.5073959); //2.71

    downtownPowerMapVelocity.put(2.3622, 40.0);
    downtownAngleMapVelocity.put(2.3622, 19.6139538); //3.22
    

  }
  private void configureShooterMotors() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 80;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLowerTime = 3;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 25;

    motorConfig.Feedback.SensorToMechanismRatio = 9/8; //1.125

    Slot0Configs slot0Config = new Slot0Configs().withKS(vKS)
                                                 .withKV(vKV)
                                                 .withKA(vKA)
                                                 .withKP(vKP);

    motorConfig.Slot0 = slot0Config;

    motorConfig.MotionMagic.MotionMagicAcceleration = vAccel;
    motorConfig.MotionMagic.MotionMagicJerk = vJerk;

    motorWheelLeader.getConfigurator().apply(motorConfig);
    motorWheelFollower.getConfigurator().apply(motorConfig);

    motorWheelFollower.setControl(new StrictFollower(motorWheelLeader.getDeviceID()));
  }

  private void configureAngleMotor() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 40;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 20;

    Slot0Configs slot0Config = new Slot0Configs().withKS(mmKS)
                                                 .withKV(mmKV)
                                                 .withKA(mmKA)
                                                 .withKP(mmKP)
                                                 .withKI(mmKI)
                                                 .withKD(mmKD)
                                                 .withKG(mmKG);

    motorConfig.Slot0 = slot0Config;

    motorConfig.MotionMagic.MotionMagicAcceleration = mmAcceleration;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = mmVelocity;
    motorConfig.MotionMagic.MotionMagicJerk = mmJerk;

    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 27.595215;

    motorAngle.getConfigurator().apply(motorConfig);
  }

  private void updateShooterAngleConstants() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 40;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 20;

    Slot0Configs slot0Config = new Slot0Configs().withKS(mmKS)
                                                 .withKV(mmKV)
                                                 .withKA(mmKA)
                                                 .withKP(mmKP)
                                                 .withKI(mmKI)
                                                 .withKD(mmKD)
                                                 .withKG(mmKG);
    motorConfig.Slot0 = slot0Config;

    motorAngle.getConfigurator().apply(motorConfig);
  }

  private void updateShooterVelocityConstants() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    Slot0Configs slot0Config = new Slot0Configs().withKS(vKS)
                                                 .withKV(vKV)
                                                 .withKA(vKA)
                                                 .withKP(vKP);

    motorConfig.Slot0 = slot0Config;

    motorConfig.MotionMagic.MotionMagicAcceleration = vAccel;
    motorConfig.MotionMagic.MotionMagicJerk = vJerk;

    motorWheelLeader.getConfigurator().apply(motorConfig);
    motorWheelFollower.getConfigurator().apply(motorConfig);
  }


  private boolean zeroEncoder() {
    if (Math.abs(motorAngle.getMotorVoltage().getValueAsDouble()) > 0.6 && Math.abs(motorAngle.getVelocity().getValueAsDouble()) < 0.05) {
      motorAngle.setPosition(0);
      motorAngle.setVoltage(0);
      currentAngleState = AngleState.STORING;
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    if (!isZeroingDone) {
      currentAngleState = AngleState.ZEROING;
      isZeroingDone = zeroEncoder();
    }
    double currentDistance = vision.getDistanceToTag19();
    if (isZeroingDone && Math.abs(currentDistance - calculatedDistance) > allowedDistanceDiff) {
      calculatedDistance = currentDistance;
      double shooterVeloc = downtownPowerMapVelocity.get(calculatedDistance);
      double shooterAngle = downtownAngleMapVelocity.get(calculatedDistance);
      velocitySetpoint = shooterVeloc;
      aimingSetpoint = shooterAngle;
    }
    publishLogs();
    applyState();
    wheelsSpunUp = rpmWithinTolerance();
    shooterReady();
  }

  private void applyState() {
    switch (currentShooterState) {
      case OVERRIDE:
        motorWheelLeader.setControl(voltageRequest);
        break;

      case SHOOTING:
        motorWheelLeader.setControl(vRequest.withVelocity(velocitySetpoint).withFeedForward(vKFF));
        //motorWheelLeader.setControl(new VoltageOut(voltageSetpoint));
        break;

      case IDLING:
        setShooterVoltage(0);
        motorWheelLeader.setControl(voltageRequest);
        break;
    }

    switch (currentAngleState) {
      case OVERRIDE:
        motorAngle.setControl(mmRequest.withPosition(angleSetpoint));
        break;

      case ZEROING:
        motorAngle.setVoltage(-0.8);
        break;

      case STORING:
        setShooterAngleSetpoint(0);
        motorAngle.setControl(mmRequest.withPosition(0));
        break;

      case IDLING:
        if (onlyOnChange) {
          onlyOnChange = false;
          setShooterAngleSetpoint(motorAngle.getPosition().getValueAsDouble());
        }
        motorAngle.setControl(mmRequest.withPosition(angleSetpoint));
        break;

      case AIMING:
        motorAngle.setControl(mmRequest.withPosition(aimingSetpoint));
        break;
    }
  }

  private void publishLogs() {
    Logger.recordOutput("HeroHeist/Shooter/Hood/Voltage", motorAngle.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("HeroHeist/Shooter/Hood/Velocity", motorAngle.getVelocity().getValueAsDouble());
    Logger.recordOutput("HeroHeist/Shooter/Hood/currently Zeroing?",!isZeroingDone);
    Logger.recordOutput("HeroHeist/Shooter/Hood/Current State", currentAngleState);
    Logger.recordOutput("HeroHeist/Shooter/Wheels/Current State", currentShooterState);
    Logger.recordOutput("HeroHeist/Shooter/Hood/Position", motorAngle.getPosition().getValueAsDouble());
    Logger.recordOutput("HeroHeist/Shooter/Hood/Setpoint", aimingSetpoint);
    Logger.recordOutput("HeroHeist/Shooter/Wheels/VoltageSetpoint", voltageSetpoint);
    Logger.recordOutput("HeroHeist/Shooter/Wheels/VelocitySetpoint", velocitySetpoint);
    Logger.recordOutput("HeroHeist/Shooter/Wheels/Velocity", motorWheelLeader.getVelocity().getValueAsDouble());
    Logger.recordOutput("HeroHeist/Shooter/Wheels/Front Voltage", motorWheelLeader.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("HeroHeist/Shooter/Wheels/Back Voltage", motorWheelFollower.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("HeroHeist/Shooter/Wheels/Velocity", motorWheelLeader.getVelocity().getValueAsDouble());
    Logger.recordOutput("HeroHeist/Shooter/Wheels/Spun Up", wheelsSpunUp);
    Logger.recordOutput("HeroHeist/Shooter/Wheels/AngleReady", angleAtSetpoint());
    Logger.recordOutput("HeroHeist/Shooter/ChangeOnce", changeOnce);
    Logger.recordOutput("HeroHeist/Shooter/Calculated Distance", calculatedDistance);

  }

  /* --------------- Calculations --------------- */

  public void updateShooterValues(double distance) {
    // Voltage //
    // aimingSetpoint = MathUtil.clamp(downtownAngleMapVoltage.get(distance), 0, 4.53);
    // voltageSetpoint = MathUtil.clamp(downtownPowerMapVoltage.get(distance), 0, 12);

    // Velocity //
    aimingSetpoint = MathUtil.clamp(downtownAngleMapVelocity.get(distance), 0, 4.53);
    velocitySetpoint = MathUtil.clamp(downtownPowerMapVelocity.get(distance), 0, 60); // Change the max later // +0.5 to the get(dist)
  }

  /**
   * uses a default tolerance
   * @return
   */
  private boolean rpmWithinTolerance() {
    return rpmWithinTolerance(.2); // used to be 2 for velocity got to tune it more
  }

  /**
   * Calculates if the RPS of the shooter is withing a certain range of where we wish it to be
   * @param tolerance rotation per second off it can be from where we want it to be
   * @return if withing tolerance
   */
  private boolean rpmWithinTolerance(double tolerance) {
    //return Math.abs(voltageSetpoint - motorWheelLeader.getMotorVoltage().getValueAsDouble()) < tolerance;
    return Math.abs(velocitySetpoint - motorWheelLeader.getVelocity().getValueAsDouble()) < tolerance;
  }

  /* --------------- Setters --------------- */

  public void setVelocitySetpoint(double setpoint) {
    velocitySetpoint = setpoint;
  }

  public void setShooterAngleSetpoint(double setpoint) {
    angleSetpoint = MathUtil.clamp(setpoint, 0, 4.53);
  }

  public void setWantedState(AngleState angleState, ShooterState shooterState) {
    onlyOnChange = true;
    currentAngleState = angleState;
    currentShooterState = shooterState;
  }

  /**
   * Only takes effect if Angle state is in OVERRIDE, if not then it'll use PID-type controlling
   * @param voltage
   */
  public void setAngleVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -maxHoodVoltage, maxHoodVoltage);
    angleVoltage = voltage;
  }

  public void setShooterVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -12, 12);
    flywheelVoltage = voltage;
    voltageRequest.Output = voltage;
  }

  public void updateShooterConstants() {
    boolean updateOuttakeVals = false;
    boolean updateAngleVals = false;
    boolean updateSetpoints = false;

    double lVKSVal = logVKS.get();
    double lVKVVal = logVKV.get();
    double lVKAVal = logVKA.get();
    double lVKPVal = logVKP.get();
    double lVKFFVal = logVKFF.get();
    double lVAccel = logVAccel.get();
    double lVJerk = logVJerk.get();
    // double lVVSVal = logVelocitySetpoint.get(); //Uncomment for velocity tuning
    //double lVoltageSetpoint = logVoltageSetpoint.get(); Uncomment for voltage tuning
    
    double lDistanceAway = logDistanceAway.get();
    updateSetpoints = (distanceAway != lDistanceAway)
                    // || (velocitySetpoint != lVVSVal)  // Uncomment for velocity tuning
                     //|| (voltageSetpoint != lVoltageSetpoint) Uncomment for voltage tuning
                     ;
    if(updateSetpoints) {
      distanceAway = lDistanceAway;
      changeOnce = true;
      //voltageSetpoint = lVoltageSetpoint;
      // velocitySetpoint = lVVSVal;  // Uncomment for velocity tuning

    }
    updateOuttakeVals = (vKS != lVKSVal)
                     || (vKV != lVKVVal)
                     || (vKA != lVKAVal)
                     || (vKP != lVKPVal)
                     || (vKFF != lVKFFVal)
                     || (vAccel != lVAccel)
                     || (vJerk != lVJerk)
                     || (vKFF != lVKFFVal);

    if (updateOuttakeVals) {
      vKS = lVKSVal;
      vKV = lVKVVal;
      vKA =lVKAVal;
      vKP = lVKPVal;
      vKFF = lVKFFVal;
      vAccel = lVAccel;
      vJerk = lVJerk;
      updateShooterVelocityConstants();
    }

    double lMMKSVal = logMMKS.get();
    double lMMKVVal = logMMKV.get();
    double lMMKAVal = logMMKA.get();
    double lMMKPVal = logMMKP.get();
    double lMMKIVal = logMMKI.get();
    double lMMKDVal = logMMKD.get();
    double lMMKGVal = logMMKG.get();
    double lMMVeloc = logMMVeloc.get();
    double lMMAcc = logMMAccel.get();
    double lMMJerk = logMMJerk.get();

    updateAngleVals = (mmKS != lMMKSVal)
                   || (mmKV != lMMKVVal)
                   || (mmKA != lMMKAVal)
                   || (mmKP != lMMKPVal)
                   || (mmKI != lMMKIVal)
                   || (mmKD != lMMKDVal)
                   || (mmKG != lMMKGVal)
                   || (mmVelocity != lMMVeloc)
                   || (mmAcceleration != lMMAcc)
                   || (mmJerk != lMMJerk);

    if (updateAngleVals) {
      mmKS = lMMKSVal;
      mmKV = lMMKVVal;
      mmKA = lMMKAVal;
      mmKP = lMMKPVal;
      mmKI = lMMKIVal;
      mmKD = lMMKDVal;
      mmKG = lMMKGVal;

      updateShooterAngleConstants();

      mmVelocity = lMMVeloc;
      mmAcceleration = lMMAcc;
      mmJerk = lMMJerk;

      mmRequest = new DynamicMotionMagicVoltage(0, mmVelocity, mmAcceleration, mmJerk);
    }
    
  }

  public void forceHoodZero() {
    if (DriverStation.isDisabled()) {
      motorAngle.setPosition(0);
    }
  }

  /* --------------- Getters -------------- */

  public double getDistance() {
    return distanceAway
    ;
  }

  public boolean shooterReady () {
    if(wheelsSpunUp && angleAtSetpoint()) { // comment for tuning
      return true;
    }
    return false;
  }

  public boolean angleAtSetpoint() {
    return Math.abs(motorAngle.getPosition().getValueAsDouble() - aimingSetpoint) < .2;
  }

  public static ShooterSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new ShooterSubsystem();
    }
    return INSTANCE;
  } 

}
