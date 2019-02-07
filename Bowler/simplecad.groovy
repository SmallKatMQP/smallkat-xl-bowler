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

		def rVal = new Cube(dh.getR()>0?dh.getR():5,5,5).toCSG()
					.toXMax()
		rVal.setColor(javafx.scene.paint.Color.RED)
		CSG profile = new Cube(1,// x dimention
					20,// y dimention
				
				1//  Z dimention
				)
				.toCSG()// converts it to a CSG tor display
				.toYMin()
				.toZMin()
				.toXMin()
				
		CSG theta;
		double thetaval = Math.toDegrees(dh.getTheta())
		if(thetaval>1){
			theta= CSG.unionAll(
			Extrude.revolve(profile,
						(double)5, // rotation center radius, if 0 it is a circle, larger is a donut. Note it can be negative too
						thetaval,// degrees through wich it should sweep
						(int)10)//number of sweep increments
			)
		}else{
			theta = profile.movex(5)
		}
		theta= moveDHValues(theta, dh )
		theta.setColor(javafx.scene.paint.Color.BLUE)

		CSG alpha;
		double alphaVal = Math.toDegrees(dh.getAlpha())
		if(alphaVal>1){
			alpha= CSG.unionAll(
			Extrude.revolve(profile,
						(double)5, // rotation center radius, if 0 it is a circle, larger is a donut. Note it can be negative too
						alphaVal,// degrees through wich it should sweep
						(int)10)//number of sweep increments
			)
		}else{
			alpha = profile.movex(5)
		}
		alpha= moveDHValues(alpha, dh )
		alpha.setColor(javafx.scene.paint.Color.YELLOW)
		
		def parts = [rVal,theta,alpha] as ArrayList<CSG>
		for(int i=0;i<parts.size();i++){
			parts.get(i).setManipulator(manipulator);
			//parts.get(i).setColor(javafx.scene.paint.Color.RED)
		}
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
