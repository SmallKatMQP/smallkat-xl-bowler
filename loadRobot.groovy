@GrabResolver(name='sonatype', root='https://oss.sonatype.org/content/repositories/releases/')
@Grab(group='com.neuronrobotics', module='SimplePacketComsJava', version='0.6.3')
@Grab(group='org.hid4java', module='hid4java', version='0.5.0')

import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.phy.HIDSimplePacketComs;
import com.neuronrobotics.sdk.addons.kinematics.imu.*;

if (args==null)
	args=[
		"https://github.com/SmallKatMQP/smallkat-xl-bowler.git",
		"Bowler/xlKat.xml"]

public class SimpleServoHID extends HIDSimplePacketComs {
	private PacketType servos = new edu.wpi.SimplePacketComs.BytePacketType(1962, 64);
	private PacketType imuData = new edu.wpi.SimplePacketComs.FloatPacketType(1804, 15);
	private final double[] status = new double[12];
	private final byte[] data = new byte[64];
	private final byte[] dataUp = new byte[64];

	public SimpleServoHID(int vidIn, int pidIn) {
		super(vidIn, pidIn);
		servos.waitToSendMode();
		addPollingPacket(servos);
		addPollingPacket(imuData);
		addEvent(1962, {
			writeBytes(1962, data);
			readBytes(1962, dataUp);
		});
		addEvent(1804, {
			readFloats(1804,status);
		});
	}
	public double[] getImuData() {
		return status;
	}
	public byte[] getData() {
		return data;
	}
	public byte[] getDataUp() {
		return dataUp;
	}
}


public class HIDSimpleComsDevice extends NonBowlerDevice{
	SimpleServoHID simple;

	public HIDSimpleComsDevice(int vidIn, int pidIn){
		simple = new SimpleServoHID(vidIn,pidIn)
		setScriptingName("hidbowler")
	}
	@Override
	public  void disconnectDeviceImp(){
		simple.disconnect()
		println "HID device Termination signel shutdown"
	}

	@Override
	public  boolean connectDeviceImp(){
		simple.connect()
	}
	void setValue(int i,int position){
		byte[] shiftedPos = new byte[2];
		shiftedPos = angleToBytes(position);
		simple.getData()[i*2]=shiftedPos[0];
		simple.getData()[i*2+1]=shiftedPos[1];
		simple.servos.pollingMode();
		//if(i == 0){
		//	for(int x = 0;x<64;x++){
				
				//System.out.println(simple.getData()[x])
				
		//	}
		//}
		//System.out.println()
		
	}
	int getValue(int i){
		int angle;
		byte[] data = new byte[2];
		data[0] = simple.getDataUp()[2*i]
		data[1] = simple.getDataUp()[2*i+1]
		angle = bytesToAngle(data);
		return (angle)
	}
	public float[] getImuData() {
		return simple.getImuData();
	}
	@Override
	public  ArrayList<String>  getNamespacesImp(){
		// no namespaces on dummy
		return [];
	}
	  public static byte[] angleToBytes(int angle){
        byte[] angleBytes = new byte[2];
        angleBytes[0] = (byte)(angle>>8)
        angleBytes[1] = (byte)(angle);
        return angleBytes;
    }
    public static int bytesToAngle(byte[] data){
        int angle;
        int []  parts = new int[2];
        for (int i = 0; i<2;i++){
        	if((int)data[i]<0){
        		parts[i] = ((int)data[i]+256)
      	  }
      	else{
      		parts[i] = (int)data[i];
      	}
        }
        angle = (parts[0]<<8)+parts[1];
       
        //System.out.println("Angle"+angle);
        return angle;
    }


}

public class HIDRotoryLink extends AbstractRotoryLink{
	HIDSimpleComsDevice device;
	int index =0;
	int lastPushedVal = 0;
	private static final Integer command =1962
	/**
	 * Instantiates a new HID rotory link.
	 *
	 * @param c the c
	 * @param conf the conf
	 */
	public HIDRotoryLink(HIDSimpleComsDevice c,LinkConfiguration conf) {
		super(conf);
		index = conf.getHardwareIndex()
		device=c
		if(device ==null)
			throw new RuntimeException("Device can not be null")
		c.simple.addEvent(command,{
			int val= getCurrentPosition();
			if(lastPushedVal!=val){
				//println "Fire Link Listner "+index+" value "+getCurrentPosition()
				try{
				fireLinkListener(val);
				}catch(java.lang.NullPointerException e){}
				lastPushedVal=val
			}else{
				//println index+" value same "+getCurrentPosition()
			}

		})

	}

	/* (non-Javadoc)
	 * @see com.neuronrobotics.sdk.addons.kinematics.AbstractLink#cacheTargetValueDevice()
	 */
	@Override
	public void cacheTargetValueDevice() {
		device.setValue(index,(int)getTargetValue())
	}

	/* (non-Javadoc)
	 * @see com.neuronrobotics.sdk.addons.kinematics.AbstractLink#flush(double)
	 */
	@Override
	public void flushDevice(double time) {
		// auto flushing
	}

	/* (non-Javadoc)
	 * @see com.neuronrobotics.sdk.addons.kinematics.AbstractLink#flushAll(double)
	 */
	@Override
	public void flushAllDevice(double time) {
		// auto flushing
	}

	/* (non-Javadoc)
	 * @see com.neuronrobotics.sdk.addons.kinematics.AbstractLink#getCurrentPosition()
	 */
	@Override
	public double getCurrentPosition() {
		return (double)device.getValue(index);
	}

}
def dev = DeviceManager.getSpecificDevice( "hidDevice",{
	//If the device does not exist, prompt for the connection

	HIDSimpleComsDevice d = new HIDSimpleComsDevice(0x3742 ,0x5751 )
	d.connect(); // Connect to it.

	LinkFactory.addLinkProvider("hidfast",{LinkConfiguration conf->
				println "Loading link "
				return new HIDRotoryLink(d,conf)
		}
	)
	println "Connecting new device: "+d
	return d
})


def cat =DeviceManager.getSpecificDevice( "XLKat",{
	//If the device does not exist, prompt for the connection
	MobileBase m = MobileBaseLoader.fromGit(
		args[0],
		args[1]
		)
	dev.simple.addEvent(1804, {
		 double[] imuDataValues = dev.simple.getImuData()
		 m.getImu()
		 .setHardwareState(
		 		new IMUUpdate(
		 			imuDataValues[0],//Double xAcceleration,
		 			imuDataValues[1],//Double yAcceleration
			 		imuDataValues[2],//,Double zAcceleration
					imuDataValues[3],//Double rotxAcceleration,
					imuDataValues[4],//Double rotyAcceleration,
					imuDataValues[5],//Double rotzAcceleration
			))


	});
	if(m==null)
		throw new RuntimeException("Arm failed to assemble itself")
	println "Connecting new device robot arm "+m
	return m
})
return null



return cat

