/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
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
	
	private Map<GeographicPoint, List<RoadSegment>> grahpic;
	
	private int numEdges;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		grahpic = new HashMap<>();
		numEdges = 0;
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return grahpic.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		return grahpic.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return numEdges;
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
		if (null == location || grahpic.containsKey(location)) {
			return false;
		}
		else {
			grahpic.put(location, new ArrayList<>());
			return true;
		}
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
		
		argumentsCheck(from, to, roadName, roadType, length);
		
		RoadSegment e1 = new RoadSegment(from, to, new ArrayList<>(), roadName, roadType, length);
		
		grahpic.get(from).add(e1);
		
		numEdges += 1;
		
	}
	
	private void argumentsCheck(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length) {
		if(null == from) {
			throw new IllegalArgumentException("from can't be null!");
		}
		else if(null == to) {
			throw new IllegalArgumentException("to can't be null!");
		}
		else if(null == roadType || "".equals(roadType)) {
			throw new IllegalArgumentException("roadType can't be null or empty!");
		}
		else if(length < 0) {
			throw new IllegalArgumentException("Length can't less than 0.");
		}
		else if(!grahpic.containsKey(from)) {
			throw new IllegalArgumentException("from is not exist in the graph!");
		}
		else if(!grahpic.containsKey(to)) {
			throw new IllegalArgumentException("to is not exist in the graph!");
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
		if(null == start || null == goal) {
			System.out.println("Start or goal node is null!  No path exists.");
			return new ArrayList<>();
		}
		
		HashSet<GeographicPoint> visited = new HashSet<>();
		Queue<GeographicPoint> toExplore = new LinkedList<>();
		HashMap<GeographicPoint, GeographicPoint> parent = new HashMap<>();
		toExplore.add(start);
		visited.add(start);
		GeographicPoint currentGeo = null;
		boolean isFound = false;
		
		while (!toExplore.isEmpty()) {
			currentGeo = toExplore.remove();
			System.out.println("Start checking " + currentGeo);
			nodeSearched.accept(currentGeo);
			if (currentGeo.equals(goal)) {
				isFound = true;
				break;
			}
			else {
				List<GeographicPoint> curNeighbors = getNeighbors(currentGeo);
				for (GeographicPoint neighbor : curNeighbors) {
					if(!visited.contains(neighbor)) {
						toExplore.add(neighbor);
						visited.add(neighbor);
						parent.put(neighbor, currentGeo);
					}
				}
			}
		}
		
		if(!isFound) {
			System.out.println("No path exists");
			return new ArrayList<>();
		}
		
		LinkedList<GeographicPoint> path = new LinkedList<>();
		GeographicPoint currt = goal;
		while(null != currt) {
			path.addFirst(currt);
			currt = parent.get(currt);
		}

		return path;
	}
	
	/**
	 * Get neighbors
	 * @param geo current geographic point
	 * @return
	 */
	private List<GeographicPoint> getNeighbors(GeographicPoint geo) {
		List<GeographicPoint> neighbours = new ArrayList<>();
		List<RoadSegment> edges = grahpic.get(geo);
		if (null != edges && !edges.isEmpty()) {
			edges.forEach( e -> neighbours.add(e.getOtherPoint(geo)));
		}
		return neighbours;
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
		// init
		int maxLoop = 10000;
		int index = 0;
		boolean isFound = false;
		HashSet<GeographicPoint> visited = new HashSet<>();
		HashMap<GeographicPointWithDistance, GeographicPointWithDistance> parent = new HashMap<>();
		GeographicPointWithDistance curGeo = null;
		Comparator<GeographicPointWithDistance> byDistance = (o1, o2) -> (Double.compare(o1.getPriority(), o2.getPriority()));
		PriorityQueue<GeographicPointWithDistance> toExplore = new PriorityQueue<>(byDistance);
		
		// add start point
		toExplore.add(new GeographicPointWithDistance(start, 0.0));
		
		while (!toExplore.isEmpty() && index < maxLoop) {
			curGeo = toExplore.remove();
			nodeSearched.accept(curGeo);
			if(curGeo.equals(goal)) {
				isFound = true;
				break;
			}			
			if(!visited.contains(curGeo.toParent())) {
				visited.add(curGeo.toParent());
				System.out.println("retrieve from q:" + curGeo + ":" + curGeo.getPriority());
				List<GeographicPointWithDistance> curNeighbors = getNeighborsInDistance(curGeo);
				for (GeographicPointWithDistance neighbor : curNeighbors) {
					System.out.println("Adding neighbors to q:" + neighbor + ":" + neighbor.getPriority());
					toExplore.add(neighbor);
					parent.put(neighbor, curGeo);
				}
			}
			index++;
		}
		
		// when not found
		if(!isFound) {
			System.out.println("No path exists, index:" + index);
			return new ArrayList<>();
		}
		
		// found and retrieve the path
		LinkedList<GeographicPoint> path = new LinkedList<>();
		while(null != curGeo) {
			path.addFirst(curGeo);
			curGeo = parent.get(curGeo);
		}
		System.out.println("Search finished , searched times:" + index + "nodes visited : " + visited.size());
		return path;
	}
	
	/**
	 * Get neighbors
	 * @param geo current geographic point
	 * @return
	 */
	private List<GeographicPointWithDistance> getNeighborsInDistance(GeographicPointWithDistance geo) {
		List<GeographicPointWithDistance> neighbours = new ArrayList<>();
		List<RoadSegment> edges = grahpic.get(geo.toParent());
		if (null != edges && !edges.isEmpty()) {
			edges.forEach( e -> {
				neighbours.add(new GeographicPointWithDistance(e.getOtherPoint(geo), geo.getPriority() + e.getLength()));	
			});
		}
		return neighbours;
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

		// init
		int maxLoop = 10000;
		int index = 0;
		boolean isFound = false;
		HashSet<GeographicPoint> visited = new HashSet<>();
		HashMap<GeographicPointWithDistance, GeographicPointWithDistance> parent = new HashMap<>();
		GeographicPointWithDistance curGeo = null;
		Comparator<GeographicPointWithDistance> byDistance = (o1, o2) -> (Double.compare(o1.getPriority(), o2.getPriority()));
		PriorityQueue<GeographicPointWithDistance> toExplore = new PriorityQueue<>(byDistance);
		
		// add start point
		toExplore.add(new GeographicPointWithDistance(start, 0.0));
		
		while (!toExplore.isEmpty() && index < maxLoop) {
			curGeo = toExplore.remove();
			nodeSearched.accept(curGeo);
			if(curGeo.equals(goal)) {
				isFound = true;
				break;
			}			
			if(!visited.contains(curGeo.toParent())) {
				visited.add(curGeo.toParent());
				System.out.println("retrieve from q:" + curGeo + ":" + curGeo.getPriority());
				List<GeographicPointWithDistance> curNeighbors = getNeighborsWithPrioirty(curGeo, goal);
				for (GeographicPointWithDistance neighbor : curNeighbors) {
					System.out.println("Adding neighbors to q:" + neighbor + ":" + neighbor.getPriority());
					toExplore.add(neighbor);
					parent.put(neighbor, curGeo);
				}
			}
			index++;
		}
		
		// when not found
		if(!isFound) {
			System.out.println("No path exists, index:" + index);
			return new ArrayList<>();
		}
		
		// found and retrieve the path
		LinkedList<GeographicPoint> path = new LinkedList<>();
		while(null != curGeo) {
			path.addFirst(curGeo);
			curGeo = parent.get(curGeo);
		}
		System.out.println("Search finished , searched times:" + index + "nodes visited : " + visited.size());
		return path;
	}
	
	
	/**
	 * Get neighbors
	 * @param geo current geographic point
	 * @return
	 */
	private List<GeographicPointWithDistance> getNeighborsWithPrioirty(GeographicPointWithDistance geo, GeographicPoint goal) {
		List<GeographicPointWithDistance> neighbours = new ArrayList<>();
		List<RoadSegment> edges = grahpic.get(geo.toParent());
		if (null != edges && !edges.isEmpty()) {
			edges.forEach( e -> {
				neighbours.add(new GeographicPointWithDistance(e.getOtherPoint(geo), geo.getPriority() + e.getLength() + e.getOtherPoint(geo).distance(goal)));	
			});
		}
		return neighbours;
	}

	
	
	public static void main(String[] args)
	{		
//		System.out.print("Making a new map...");
//		MapGraph firstMap = new MapGraph();
//		System.out.print("DONE. \nLoading the map...");
//		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
//		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
		/*MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);*/
		
		
		
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
