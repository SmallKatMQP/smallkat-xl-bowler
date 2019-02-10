//Your code here
import com.neuronrobotics.bowlerstudio.creature.ICadGenerator;
import com.neuronrobotics.bowlerstudio.creature.CreatureLab;
import org.apache.commons.io.IOUtils;
import com.neuronrobotics.bowlerstudio.vitamins.*;
import java.nio.file.Paths;
import eu.mihosoft.vrl.v3d.FileUtil;
import eu.mihosoft.vrl.v3d.Transform;
import javafx.scene.transform.Affine;
import com.neuronrobotics.bowlerstudio.physics.TransformFactory;
println "Loading STL file"
return new ICadGenerator(){

	private CSG moveDHValues(CSG incoming,DHLink dh ){
		TransformNR step = new TransformNR(dh.DhStep(0)).inverse()
		Transform move = TransformFactory.nrToCSG(step)
		return incoming.transformed(move)

	}

	@Override
	public ArrayList<CSG> generateCad(DHParameterKinematics d, int linkIndex) {
		
		ArrayList<CSG> allCad=new ArrayList<>();
		ArrayList<DHLink> dhLinks = d.getChain().getLinks()
		DHLink dh = dhLinks.get(linkIndex)
		// Hardware to engineering units configuration
		LinkConfiguration conf = d.getLinkConfiguration(linkIndex);
		// Engineering units to kinematics link (limits and hardware type abstraction)
		AbstractLink abstractLink = d.getAbstractLink(linkIndex);// Transform used by the UI to render the location of the object
		// Transform used by the UI to render the location of the object
		Affine manipulator = dh.getListener();
		Affine lastLinkFrame
		
		if(linkIndex==0)
			lastLinkFrame=d.getRootListener()
		else
			lastLinkFrame=dhLinks.get(linkIndex-1).getListener();
		
		def rVal = new Cube(dh.getR()>0?dh.getR():5,5,5).toCSG()
					.toXMax()
					.toZMax()
		rVal.setColor(javafx.scene.paint.Color.RED)
		CSG profile = new Cube(1,// x dimention
					20,// y dimention
				
				1//  Z dimention
				)
				.toCSG()// converts it to a CSG tor display
				.toYMin()
				.toZMin()
				//.toXMin()
				
		CSG theta;
		double thetaval = Math.toDegrees(dh.getTheta())
		if(Math.abs(thetaval)>1){
			theta= CSG.unionAll(
			Extrude.revolve(profile,
						0, // rotation center radius, if 0 it is a circle, larger is a donut. Note it can be negative too
						Math.abs(thetaval),// degrees through wich it should sweep
						(int)10)//number of sweep increments
			)
		}else{
			theta = profile
		}
		
		if(thetaval>0){
			theta= theta.rotz(-thetaval)
		}
		
		theta.setColor(thetaval>0?javafx.scene.paint.Color.BLUE:javafx.scene.paint.Color.AQUA)

		CSG alpha;
		double alphaVal = Math.toDegrees(dh.getAlpha())
		if(Math.abs(alphaVal)>0){
			alpha= CSG.unionAll(
			Extrude.revolve(profile,
						0, // rotation center radius, if 0 it is a circle, larger is a donut. Note it can be negative too
						Math.abs(alphaVal),// degrees through wich it should sweep
						(int)10)//number of sweep increments
			)
			//.rotz(alphaVal<0?-alphaVal:0)
		}else{
			alpha = profile
		}
		alpha= alpha.roty(90)
		if(alphaVal<0){
			alpha= alpha.rotx(-alphaVal)	
		}
		alpha= alpha.rotz(-thetaval)
				.movez(dh.getD())
		alpha.setColor(alphaVal>0?javafx.scene.paint.Color.YELLOW:javafx.scene.paint.Color.GOLDENROD)

		def dpart = new Cube(5,5,dh.getD()>0?dh.getD():2.5).toCSG()
					.toZMin()
		double upperLimit = -abstractLink.getMaxEngineeringUnits()
		double lowerLimit = -abstractLink.getMinEngineeringUnits()
		double totalRange = -(upperLimit-lowerLimit)
		def min = 20
		if(totalRange>360-min)
			totalRange=360-min
		if(totalRange<min)
			totalRange=min
		println "Link range = "+totalRange
		def rangeComp = 360-totalRange
		def orentationAdjust = -thetaval+90
		def Range = CSG.unionAll(
		Extrude.revolve(profile,
				0, // rotation center radius, if 0 it is a circle, larger is a donut. Note it can be negative too
				rangeComp,// degrees through wich it should sweep
				(int)min)//number of sweep increments
		)
		.rotz(lowerLimit+orentationAdjust)
		.movez(-2)
		.setColor(javafx.scene.paint.Color.LIGHTGREEN)
		def upperLim = profile
					.rotz(upperLimit+orentationAdjust)
					.setColor(javafx.scene.paint.Color.HOTPINK)
		def lowerLim = profile
					.rotz(lowerLimit+orentationAdjust)
					.setColor(javafx.scene.paint.Color.WHITE)
		def zeroLim = profile
					.rotz(orentationAdjust)
					.setColor(javafx.scene.paint.Color.INDIGO)
		def lastFrameParts = [theta,alpha,dpart,upperLim,lowerLim,zeroLim,Range]
		def parts = [rVal] as ArrayList<CSG>
		for(int i=0;i<parts.size();i++){
			parts.get(i).setManipulator(manipulator);
			//parts.get(i).setColor(javafx.scene.paint.Color.RED)
		}
		for(int i=0;i<lastFrameParts.size();i++){
			lastFrameParts.get(i).setManipulator(lastLinkFrame);
			//parts.get(i).setColor(javafx.scene.paint.Color.RED)
		}
		parts.addAll(lastFrameParts)
		return parts;

	}



@Override
	public ArrayList<CSG> generateBody(MobileBase b ) {
		ArrayList<CSG> allCad=new ArrayList<>();

	// Load the .CSG from the disk and cache it in memory
		CSG body  = new Cube(5).toCSG();

		body.setManipulator(b.getRootListener());
		body.setColor(javafx.scene.paint.Color.WHITE)
		def parts = [body ] as ArrayList<CSG>
		for(int i=0;i<parts.size();i++){
			parts.get(i).setColor(javafx.scene.paint.Color.GRAY)
		}
		return parts;
	}
}
