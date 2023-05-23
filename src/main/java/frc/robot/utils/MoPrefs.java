package frc.robot.utils;

import java.util.EnumSet;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Function;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.StringPublisher;

public final class MoPrefs {
    public static Pref<Double> fl_zero = doublePref("FL_Zero", 0);
    public static Pref<Double> fl_scale = doublePref("FL_Scale", 1);
    public static Pref<Double> fr_zero = doublePref("FR_Zero", 0);
    public static Pref<Double> fr_scale = doublePref("FR_Scale", 1);
    public static Pref<Double> rl_zero = doublePref("RL_Zero", 0);
    public static Pref<Double> rl_scale = doublePref("RL_Scale", 1);
    public static Pref<Double> rr_zero = doublePref("RR_Zero", 0);
    public static Pref<Double> rr_scale = doublePref("RR_Scale", 1);


    // MtrScale: units of ticks per meter
    public static Pref<Double> fl_driveMtrScale = doublePref("FL_driveMtrScale", 1);
    public static Pref<Double> fr_driveMtrScale = doublePref("FR_driveMtrScale", 1);
    public static Pref<Double> rl_driveMtrScale = doublePref("RL_driveMtrScale", 1);
    public static Pref<Double> rr_driveMtrScale = doublePref("RR_driveMtrScale", 1);

    public static Pref<Double> chassis_size_x = doublePref("Chassis Size X", 1);
    public static Pref<Double> chassis_size_y = doublePref("Chassis Size Y", 1);

    public static Pref<Double> maxLinearSpeed = doublePref("Maximum Linear Speed", 4);
    public static Pref<Double> maxAngularSpeed = doublePref("Maximum Angular Speed", Math.PI);

    public final class Pref<T> {
        public final String key;
        private Function<NetworkTableValue, T> getter;
        private BiFunction<NetworkTableEntry, T, Boolean> setter;

        private final NetworkTableEntry entry;

        private Consumer<T> subscriber = null;

        public Pref(String key, T defaultValue, Function<NetworkTableValue, T> getter, BiFunction<NetworkTableEntry, T, Boolean> setter) {
            this.key = key;
            this.getter = getter;
            this.setter = setter;

            this.entry = table.getEntry(key);
            this.entry.setDefaultValue(defaultValue);
            this.entry.setPersistent();
        }

        public T get() {
            return getter.apply(entry.getValue());
        }

        public void set(T value) {
            setter.apply(entry, value);
        }

        public void subscribe(Consumer<T> consumer) {
            subscribe(consumer, false);
        }

        public void subscribe(Consumer<T> consumer, boolean notifyImmediately) {
            if(subscriber != null) {
                subscriber = subscriber.andThen(consumer);
            } else {
                subscriber = consumer;
                entry.getInstance().addListener(
                    entry,
                    EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                    (e) -> consumer.accept(getter.apply(e.valueData.value))
                );
            }

            if(notifyImmediately) {
                consumer.accept(this.get());
            }
        }
    }

    private static MoPrefs instance;
    private NetworkTable table;
    private StringPublisher typePublisher;

    private static MoPrefs getInstance() {
        if(instance == null) {
            instance = new MoPrefs();
        }
        return instance;
    }

    private MoPrefs() {
        table = NetworkTableInstance.getDefault().getTable("Preferences");
        typePublisher = table.getStringTopic(".type").publish();
        typePublisher.set("RobotPreferences");
    }

    private static Pref<Double> doublePref(String key, double defaultValue) {
        return getInstance().new Pref<>(key, defaultValue, NetworkTableValue::getDouble, NetworkTableEntry::setDouble);
    }
}
