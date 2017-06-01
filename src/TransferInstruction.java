
public class TransferInstruction {
	public enum TransferType {
		ENTER, EXIT, WAIT, CHASE	//enter/exit Hohmann transfer, wait for alignment, chase object in orbit
	}
	
	TransferType type;
	double dv;
	int dir; //1 is in same direction, 0 is not
	double desired_r;
	double desiredAngle;
	CelestialBody body;
	
	public TransferInstruction(double dv, int dir) {
		this.dv = dv;
		this.dir = dir;
		this.type = TransferType.ENTER;
	}
	
	public TransferInstruction(double dv, int dir, double r, CelestialBody body) {
		this.dv = dv;
		this.dir = dir;
		this.desired_r = r;
		this.body = body;
		this.type = TransferType.EXIT;
	}
	
	public TransferInstruction(double angle, CelestialBody body, int dir) {
		desiredAngle = angle;
		this.body = body;
		this.dir = dir;		// +dir means body is ahead of you
		this.type = TransferType.WAIT;
	}
	
	public TransferInstruction(CelestialBody body) {
		this.body = body;
		desired_r = body.SOI();
		dv = Math.sqrt(PhysicsEngine.G * body.mass / desired_r);
		this.type = TransferType.CHASE;
	}
	
	public TransferType getType() {
		return type;
	}
}


