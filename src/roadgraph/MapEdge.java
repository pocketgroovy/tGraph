package roadgraph;

import geography.GeographicPoint;

public class MapEdge {
	private GeographicPoint _startPoint; 
	private GeographicPoint _endPoint; 
	private String _roadName;
	private String _roadType;
	private double _length;
	
	/** create a new MapEdge
	 * @param from geographic starting point
	 * @param to geographic ending point
	 * @param roadName
	 * @param roadType
	 * @param length length of the edge from the start to the end pint
	 */
	public MapEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length){
		_startPoint = from;
		_endPoint = to;
		_roadName = roadName;
		_roadType = roadType;
		_length = length;
	}
	
	/**
	 * @return geographic starting point
	 */
	public GeographicPoint getStartPoint(){
		return _startPoint;
	}
	
	/**
	 * @return geographic ending point
	 */
	public GeographicPoint getEndPoint(){
		return _endPoint;
	}
	
	/**
	 * @return road name
	 */
	public String getRoadName(){
		return _roadName;
	}
	
	/**
	 * @return road type
	 */
	public String getRoadType(){
		return _roadType;
	}
	
	/**
	 * @return length between the starting point and ending point of this edge
	 */
	public double getLength(){
		return _length;
	}
	
	/* (non-Javadoc)
	 * @see java.lang.Object#hashCode()
	 */
	@Override
    public int hashCode() {
        long bits = java.lang.Double.doubleToLongBits(_startPoint.getX() + _endPoint.getX());
        bits ^= java.lang.Double.doubleToLongBits(_startPoint.getY() + _endPoint.getY()) * 31;
        return (((int) bits) ^ ((int) (bits >> 32)));
    }

    /**
     * Determines whether or not two  are equal. Two instances of
     * <code>MapEdge</code> are equal if the values of their
     * <code>end  point</code> and <code>road name</code> and
     * <code>road type </code> and <code>length</code> are the same.
     * @param obj an object to be compared with this <code>MapEdge</code>
     * @return <code>true</code> if the object to be compared is
     *         an instance of <code>MapEdge</code> and has
     *         the same values; <code>false</code> otherwise.
     * @since 1.2
     */
	@Override
    public boolean equals(Object obj) {
    	if(obj instanceof MapEdge){
    		MapEdge edge = (MapEdge)obj;
	            return (_startPoint.equals(edge.getStartPoint()) && _endPoint.equals(edge.getEndPoint()) 
	            		&& _roadName.equals(edge.getRoadName()) && _roadType.equals(edge.getRoadType()) 
	            		&& _length == edge.getLength());
        }
        return super.equals(obj);
    }
}
