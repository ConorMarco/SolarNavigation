import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class OrbitShip extends Ship {
	public static final double MARGIN_ERROR = 10;
	public static final double ANGLE_ERROR = 1e-3;
	
	List<TransferInstruction> transfers;
	int index = 0;
	TransferInstruction current;
	double dv;
	
	public OrbitShip(double mass, Vector3D startPos, Vector3D startVel, double thrust, List<TransferInstruction> transfers) {
		super(mass, startPos, startVel, thrust);
		this.transfers = transfers;
		index = 0;
		current = transfers.get(0);
		dv = 0;
	}

	@Override
	public Vector3D getSteering(float dt, double time) {
		if(current.getType() == TransferInstruction.TransferType.ENTER || 
				(current.getType() == TransferInstruction.TransferType.EXIT && current.body.getPos(time).distance(this.getPos()) < MARGIN_ERROR)) {
			if(dv > 0) {
				double magnitude;
				
				if(thrust * dt > dv) {
					magnitude = dv/dt;
					dv = 0;
					
					updateTransfer();
				} else {
					magnitude = thrust;
					dv -= thrust * dt;
				}
				
				return this.getVel().normalize().scalarMultiply(magnitude);
			}
		}
		
		if(current.getType() == TransferInstruction.TransferType.CHASE) {
			if(this.getPos().distance(current.body.getPos(time)) < current.desired_r) {
				if(dv > 0) {
					double magnitude;
					
					if(thrust * dt > dv) {
						magnitude = dv/dt;
						dv = 0;
					} else {
						magnitude = thrust;
						dv -= thrust * dt;
					}
					
					int dir = this.getVel().crossProduct(current.body.getPos(time)).getZ() > 0 ? 1 : -1;
					Vector3D steer = this.getPos().subtract(current.body.getPos(time)).scalarMultiply(dir * magnitude);
					
					if(dv == 0) {
						updateTransfer();
					}

					return steer;
				}
			}
			
			Vector3D v1 = current.body.getPos(time).subtract(current.body.getParent().getPos(time));
			Vector3D v2 = this.getPos().subtract(current.body.getParent().getPos(time));
			int ahead = v1.crossProduct(v2).getZ() > 0 ? 1 : -1;
			
			double r_error = this.getPos().distance(current.body.getParent().getPos(time))
					- current.body.getPos(time).distance(current.body.getParent().getPos(time));
			Vector3D to_parent = current.body.getParent().getPos(time).subtract(this.getPos());
			Vector3D correction = to_parent.normalize().scalarMultiply(thrust * r_error * 1e-6);
			return this.getVel().normalize().scalarMultiply(thrust * ahead * 1e-2).add(correction);
		}
		
		if(current.getType() == TransferInstruction.TransferType.WAIT) {
			Vector3D v1 = current.body.getPos(time).subtract(current.body.getParent().getPos(time));
			Vector3D v2 = this.getPos().subtract(current.body.getParent().getPos(time));
			double angle = Vector3D.angle(v1, v2);
			
			double ahead = v1.crossProduct(v2).getZ() * current.dir; //true if > 0
			if(Math.abs(angle - current.desiredAngle) < ANGLE_ERROR && ahead > 0) {
				updateTransfer();
				return getSteering(dt, time);				
			}
		}
		
		return Vector3D.ZERO;
	}

	private void updateTransfer() {
		if(++index < transfers.size()) {
			current = transfers.get(index);
			dv = current.dv;
		}
	}

}
