package frc.lib.ctre;

import com.ctre.phoenix.sensors.SensorInitializationStrategy;

public class MagEncoderAbsoluteConfiguration {
    private final int id;
    private final double offset;
    private final String canbus;
    private final SensorInitializationStrategy initStrategy;

    public MagEncoderAbsoluteConfiguration(int id, double offset, String canbus, SensorInitializationStrategy initStrategy) {
        this.id = id;
        this.offset = offset;
        this.canbus = canbus;
        this.initStrategy = initStrategy;
    }

    public MagEncoderAbsoluteConfiguration(int id, double offset, String canbus){
        this(id, offset, canbus, SensorInitializationStrategy.BootToAbsolutePosition);
    }

    public MagEncoderAbsoluteConfiguration(int id, double offset){
        this(id, offset, "");
    }

    public int getId() {
        return id;
    }

    public double getOffset() {
        return offset;
    }

    public String getCanbus(){
        return canbus;
    }

    public SensorInitializationStrategy getInitStrategy(){
        return initStrategy;
    }

    @Override
    public String toString() {
        return "MagEncoderConf{id=" + this.id + ", offset=" + this.offset + "}"; 
    }
}
