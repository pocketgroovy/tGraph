/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 2
	private int _numVertices;
	private int _numEdges;
	private HashMap<GeographicPoint, Node> _nodeMap; // adjacency list
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
		_nodeMap = new HashMap<>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		return _numVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{		
		//TODO: Implement this method in WEEK 2
		Set<GeographicPoint> vertices = _nodeMap.keySet();
		return vertices;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2
		return _numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 2
		if(location != null && !_nodeMap.containsKey(location)){
			_numVertices++; // increment vertex count for faster access
			Node node = new Node(location);
			// add the vertex with associated node information
			_nodeMap.put(location, node);
			return true;
		}
		return false;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
				String roadType, double length) throws IllegalArgumentException {
		//TODO: Implement this method in WEEK 2
		if(from == null || to == null || roadName == null || roadType == null || length == 0.0){
			throw new IllegalArgumentException();
		}
		if(!_nodeMap.containsKey(from) || !_nodeMap.containsKey(to)){
			throw new IllegalArgumentException();	
		}
		
		Node node = _nodeMap.get(from);
	
		// add edge and increment the edge count if this is never added before for this node.
		if(node.addEdge(from, to, roadName, roadType, length)){ 
			_numEdges++;
		}
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 2
		List<GeographicPoint> list = new ArrayList<>();
		HashMap<Node, Boolean> visited = exploredMap();
		HashMap<Node, Node> parent = new HashMap<>(); // current node as key, its parent as value
		
		if(!_nodeMap.containsKey(start) || !_nodeMap.containsKey(goal)) {return null;}
		
		Queue<Node> queue = new LinkedList<Node>();
		Node current = _nodeMap.get(start);
		Node target = _nodeMap.get(goal);
		queue.offer(current);
		visited.put(current, true);
		parent.put(current, null); // first node with no parent
		
		while(!queue.isEmpty()){
			current = queue.poll();	
			if(current == target) break;
			
			if(current.getNumOutdegree() > 0){
				for(int i = 0; i < current.getNumOutdegree(); i++){
					Node next = _nodeMap.get(current.getEdges().get(i).getEndPoint());
					if(!visited.get(next)){
						queue.offer(next);
						visited.put(next, true);
						parent.put(next, current);
						nodeSearched.accept(next.getNodePosition());
					}
				}
			}			
		}
		list = getPaths(parent, target); // A list of parent will be returned in reverse order
		if(list== null) return null; // if no relationship between start and goal, list will be null
		
		Collections.reverse(list);
		return list;
	}
	
	/** initialize the map for visited nodes
	 * @return initialized map with node as key and false as value
	 */
	private HashMap<Node, Boolean> exploredMap(){
		HashMap<Node, Boolean> visited = new HashMap<>();
		Iterator<GeographicPoint> gps = _nodeMap.keySet().iterator();
		while(gps.hasNext()){
			visited.put(new Node(gps.next()), false);
		}
		return visited;
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * @param map Parent Map
	 * @param goal The goal location
	 * @return The list of parents from goal to start
	 */
	private List<GeographicPoint> getPaths(HashMap<Node, Node> map, Node goal){
		List<GeographicPoint> list = new ArrayList<>();
		list.add(goal.getNodePosition());
		Node parent = map.get(goal);
		if(parent == null) return null;
		
		while(parent !=null){
			list.add(parent.getNodePosition());
			parent = map.get(parent);
		}
		return list;
	}
	
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		resetDistance(_nodeMap); // initialize the distances each time the search needs to run
		List<GeographicPoint> list = new ArrayList<>();
		HashMap<Node, Boolean> visited = exploredMap();
		HashMap<Node, Node> parent = new HashMap<>(); // current node as key, its parent as value
		if(!_nodeMap.containsKey(start) || !_nodeMap.containsKey(goal)) {return null;}
		
		// initialize priority queues with node comparator which will create ascending order of their distance
		PriorityQueue<Node> queue = new PriorityQueue<Node>(_numVertices, new NodeComparator());
		
		Node current = _nodeMap.get(start);
		current.setDistance(0.0); // start node with 0 distance
		queue.offer(current);
		parent.put(current, null); // first node with no parent
		
		Node target = _nodeMap.get(goal);
		int counter = 0;
		while(!queue.isEmpty()){
			current = queue.poll();	
			counter++;
			nodeSearched.accept(current.getNodePosition());
			if(!visited.get(current)){
				visited.put(current, true);
				if(current == target)	break;
				
				// iterate through all the neighbors
				for(int i = 0; i < current.getNumOutdegree(); i++){
					Node next = _nodeMap.get(current.getEdges().get(i).getEndPoint());
					if(!visited.get(next)){
						// compare sum of current's distance and the edge's length with next's distance
						double newDistance = current.getEdges().get(i).getLength() + current.getDistance();
						if(next.getDistance() > newDistance){
							//update the distance with shorter one
							next.setDistance(newDistance);
							parent.put(next, current);
							queue.offer(next);
						}
					}
				}	
			}
		}
		list = getPaths(parent, target); // A list of parent will be returned in reverse order
		if(list== null) return null; // if no relationship between start and goal, list will be null
		
		Collections.reverse(list);
		// testing for quiz 
		System.out.print("Dijkstra: ");
		printSizeOfVisited(visited);
		System.out.println("counter: "+counter);
		return list;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		resetDistance(_nodeMap); // initialize the distances each time the search needs to run
		resetPredictedDistance(_nodeMap); // initialize the predicted distances each time the search needs to run

		List<GeographicPoint> list = new ArrayList<>();
		HashMap<Node, Boolean> visited = exploredMap();
		HashMap<Node, Node> parent = new HashMap<>(); // current node as key, its parent as value
		
		if(!_nodeMap.containsKey(start) || !_nodeMap.containsKey(goal)) {return null;}
		
		// initialize priority queues with node comparator which will create ascending order of their predicted distance
		PriorityQueue<Node> queue = new PriorityQueue<Node>(_numVertices, new NodePredictedComparator());
		
		Node current = _nodeMap.get(start);
		current.setDistance(0.0); // start node with 0 distance
		current.setPredictedDistance(0.0); // start node with 0 predicted distance
		queue.offer(current);
		parent.put(current, null); // first node with no parent
		
		Node target = _nodeMap.get(goal);
		int counter = 0;
		while(!queue.isEmpty()){
			current = queue.poll();	
			counter++;
			nodeSearched.accept(current.getNodePosition());
			if(!visited.get(current)){
				visited.put(current, true);
				if(current == target) break;
				
				// iterate through all the neighbors
				for(int i = 0; i < current.getNumOutdegree(); i++){
					Node next = _nodeMap.get(current.getEdges().get(i).getEndPoint());
					if(!visited.get(next)){
						double currentEdgeLength = current.getEdges().get(i).getLength();
						double newDistance = currentEdgeLength + current.getDistance();
						
						// compare sum of current's distance and the edge's length with next's distance
						if(next.getDistance() > newDistance){
							next.setDistance(newDistance);
						}								
						// compare sum of current's distance and the edge's length to next's distance and distance to goal 
						// from the next node with the next's predicted distance
						double newPredictedDistance = currentEdgeLength + current.getDistance() +
								next.getNodePosition().distance(goal);
						if(next.getPredictedDistance() > newPredictedDistance){
							//update the distance with shorter one
							next.setPredictedDistance(newPredictedDistance);
							parent.put(next, current);
							queue.offer(next);
						}
					}
				}	
			}
		}
		list = getPaths(parent, target); // A list of parent will be returned in reverse order
		if(list== null) return null; // if no relationship between start and goal, list will be null
		
		Collections.reverse(list);
		// testing for quiz 
		System.out.print("A*: ");
		printSizeOfVisited(visited);
		System.out.println("counter: "+counter);
		return list;
	}

	/** set distance to infinity
	 * @param nodeMap
	 */
	private void resetDistance(HashMap<GeographicPoint, Node> nodeMap){
		Iterator<GeographicPoint> vertices = nodeMap.keySet().iterator();
		while(vertices.hasNext()){
			Node node = nodeMap.get(vertices.next());
			node.setDistance(Double.POSITIVE_INFINITY);
		}
	}
	
	/** set predicted distance to infinity
	 * @param nodeMap
	 */
	private void resetPredictedDistance(HashMap<GeographicPoint, Node> nodeMap){
		Iterator<GeographicPoint> vertices = nodeMap.keySet().iterator();
		while(vertices.hasNext()){
			Node node = nodeMap.get(vertices.next());
			node.setPredictedDistance(Double.POSITIVE_INFINITY);
		}	
	}
	
	/** print out number of visited nodes.
	 *  used for testing
	 * @param visited
	 */
	private void printSizeOfVisited(HashMap<Node, Boolean> visited){
		Iterator<Node> nodes = visited.keySet().iterator();
		int counter = 0;
		while(nodes.hasNext()){
			if(visited.get(nodes.next())){
				counter++;
			}
		}
		System.out.println(counter);
	}
	
	public static void main(String[] args)
	{
//		System.out.print("Making a new map...");
//		MapGraph firstMap = new MapGraph();
//		System.out.print("DONE. \nLoading the map...");
//		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
//		System.out.println("DONE.");
//		
//		// You can use this method for testing.  
//		
//		
//		/* Here are some test cases you should try before you attempt 
//		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
//		 * programming assignment.
//		 */
////		
//		MapGraph simpleTestMap = new MapGraph();
//		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
////		
//		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
//		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
////		
////		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
//		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
//		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
//		
//		
//		MapGraph testMap = new MapGraph();
//		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
//		
//		// A very simple test using real data
//		testStart = new GeographicPoint(32.869423, -117.220917);
//		testEnd = new GeographicPoint(32.869255, -117.216927);
//		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
//		testroute = testMap.dijkstra(testStart,testEnd);
//		testroute2 = testMap.aStarSearch(testStart,testEnd);
//		
//		
//		// A slightly more complex test using real data
//		testStart = new GeographicPoint(32.8674388, -117.2190213);
//		testEnd = new GeographicPoint(32.8697828, -117.2244506);
//		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
//		testroute = testMap.dijkstra(testStart,testEnd);
//		testroute2 = testMap.aStarSearch(testStart,testEnd);
//		
//		
//		
		/* Use this code in Week 3 End of Week Quiz */
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		
		
	}
	
}
