// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.drivers;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class PearadoxTalonFX extends TalonFX{
    /**
     * Creates a new TalonFX with the necessary configurations.
     * @param deviceId The device ID.
     * @param mode The neutral mode (Brake/Coast).
     * @param limit The current limit.
     * @param isInverted The invert type of the motor.
     */
    public PearadoxTalonFX(int deviceId, NeutralModeValue mode, int limit, boolean isInverted){
        super(deviceId);
        this.setNeutralMode(mode);
        this.setInverted(isInverted);

        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.withSupplyCurrentLimitEnable(true);
        currentLimitsConfigs.withSupplyCurrentLimit(limit);
        currentLimitsConfigs.withSupplyTimeThreshold(limit);
        currentLimitsConfigs.withSupplyTimeThreshold(1);

        this.getConfigurator().apply(currentLimitsConfigs);
    }
}
