package frc.robot.utility.networking;

public class NetworkingObserver {
    public String objectName;
    private NetworkingCall call;

    public String replyName;

    public NetworkingObserver(String objectName, String replyName, NetworkingCall call) {
        this.objectName = objectName;
        this.call = call;
        this.replyName = replyName;
    }

    public NetworkingObserver(String objectName, NetworkingCall call) {
        this.objectName = objectName;
        this.call = call;
    }

    public <T> void update(T value) {
        call.update(value);
    }
}