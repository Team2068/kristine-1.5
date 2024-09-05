package frc.robot.utility;

import java.lang.Object;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DebugTable {
    static NetworkTable NetTabInst = NetworkTableInstance.getDefault().getTable("Debug");
    
    public static Object get(String DataName) {
        Object obj = NetTabInst.getValue(DataName).getValue();
        if (obj == null) {
            set(DataName, 0.0);
            return get(DataName);
        }
        return obj;
    }

    public static Object get(String DataName, Object defaultValue) {
        Object obj = NetTabInst.getValue(DataName).getValue();
        if (obj == null) {
            set(DataName, defaultValue);
            return defaultValue;
        }
        return obj;
    }

    public static void set(String DataName, Object Data) {
        NetTabInst.getEntry(DataName).setValue(Data);
    }
}