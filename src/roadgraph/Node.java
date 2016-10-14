package roadgraph;

import java.util.ArrayList;
import java.util.List;

import geography.GeographicPoint;

public class Node {
	private GeographicPoint _nodePosition;
	private List<MapEdge> _edges;
	private double _distance;
	private double _predictedDistance;

	int _numOutdegree;
	
	/** 
	 * Create a new Node with Geographic Point
	 * initializes List of its associated edges 
	 */
	public Node(GeographicPoint from){
		_nodePosition = from;
		_edges = new ArrayList<>();
	}
	
	/**
	 * Get this node distance from start point
	 * @return distance
	 */
	public double getDistance() {
		return _distance;
	}

	/** Set/update distance from start point
	 * @param distance
	 */
	public void setDistance(double distance) {
		_distance = distance;
	}
	
	/** Get the predicted Distance from start to end
	 * @return predicted distance
	 */
	public double getPredictedDistance() {
		return _predictedDistance;
	}

	/** Set predicted distance: distance to goal from this node with distance from start point to this node
	 * @param predictedDistance
	 */
	public void setPredictedDistance(double predictedDistance) {
		_predictedDistance = predictedDistance;
	}
	
	/** add to list of edge(or neighbor node position, road name, road type and length information)
	 * @param from geographic starting point
	 * @param to geographic ending point
	 * @param roadName
	 * @param roadType
	 * @param length length of the edge from the start to the end pint
	 * @return true if edges are newly added if already exists return false to avoid duplicates
	 */
	public boolean addEdge(GeographicPoint from,GeographicPoint to, String roadName, String roadType, double length){
		MapEdge edge = new MapEdge(from, to, roadName, roadType, length);
		if(!_edges.contains(edge)){
			_edges.add(edge);
			_numOutdegree++;
			return true;
		}
		return false;
	}
	
	
	/**
	 * @return neighbor node count
	 */
	public int getNumOutdegree(){
		return _numOutdegree;
	}
	
	/**
	 * @return this node's geographic point
	 */
	public GeographicPoint getNodePosition(){
		return _nodePosition;
	}
	
	
	/**
	 * @return list of edges( as neighbors)
	 */
	public List<MapEdge> getEdges(){
		return new ArrayList<MapEdge>(_edges); // return a copy of the data
	}
	
	/* (non-Javadoc)
	 * @see java.lang.Object#hashCode()
	 */
	@Override
    public int hashCode() {
        long bits = java.lang.Double.doubleToLongBits(_nodePosition.getX());
        bits ^= java.lang.Double.doubleToLongBits(_nodePosition.getY()) * 31;
        return (((int) bits) ^ ((int) (bits >> 32)));
    }

    /**
     * Determines whether or not two  are equal. Two instances of
     * <code>Node</code> are equal if the values of their
     * <code>nodePositions</code> are the same.
     * @param obj an object to be compared with this <code>Node</code>
     * @return <code>true</code> if the object to be compared is
     *         an instance of <code>Node</code> and has
     *         the same values; <code>false</code> otherwise.
     * @since 1.2
     */
	@Override
    public boolean equals(Object obj) {
    	if(obj instanceof Node){
    		Node node = (Node)obj;
	            return (_nodePosition.equals(node.getNodePosition()));
        }
        return super.equals(obj);
    }
	
	/* (non-Javadoc)
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString()
    {
    	return _nodePosition.toString();
    }
}
