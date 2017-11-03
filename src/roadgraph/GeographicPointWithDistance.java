package roadgraph;

import geography.GeographicPoint;

public class GeographicPointWithDistance extends GeographicPoint {

	// UID
	private static final long serialVersionUID = -8879962719904192751L;
	// distance from start point so far
	private double priority;
	
	public GeographicPointWithDistance(double latitude, double longitude) {
		super(latitude, longitude);
	}
	
	public GeographicPointWithDistance(GeographicPoint geography, double distance) {
		super(geography.getX(), geography.getY());
		this.priority = distance;
	}
	
	public GeographicPoint toParent() {
		return new GeographicPoint(super.getX(), super.getY());
	}

	public double getPriority() {
		return priority;
	}

	@Override
	public int hashCode() {
		return super.hashCode() + (int)this.priority;
	}

	@Override
	public boolean equals(Object obj) {
		if (obj instanceof GeographicPointWithDistance) {
			return super.equals(obj) && this.priority == ((GeographicPointWithDistance)obj).getPriority();
		}
		else {
			return super.equals(obj);
		}
	}
	
	
}
