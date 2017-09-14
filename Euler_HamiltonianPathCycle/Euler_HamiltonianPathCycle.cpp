// Euler_HamiltonianPathCycle.cpp : Defines the entry point for the console application.
/*
1. Identify if the given graph contains Euler path and Euler Cycle
2. Idenify if the given graph contains Hamiltonian path and Hamiltonian Cycle
*/
/*
1. EULER PATH: visist every edged exactly once. may visit few nodes more than once.
2. EULER CYCLE: is the Euler path which starts and ends on the same vertex
	A. Undirected Graph: All non-zero degree vertex are strongly connected and all non-zero degree vertices have even degree
	B. Directed Graph: All non-zero degree vertices are strongly connected and IN and OUT degrees are same for all vertices.
3. HAMILTONIAN PATH: visited each vertex exactly once.
4. HAMILTONIAN CYCLE: if there exists and edge between start and end vertext of the Hamiltonian path

STRONGLY CONNECTED : If there exists a path between any 2 nodes (both direction) in graph. 
ALGO to ID count of stringly connected components.
1. DO DFS in original graph and push the vertex in a stack once the DFS is compelete for that node.
2. Now get a transpose of original graph.
3. start popping the vertex from stack and do DFS on transpose graph. Here we would get a SCC for graph.

*/

#include "stdafx.h"
#include <iostream>
#include <vector>
#include <list>
#include <stack>

using namespace std;

class Graph {
	int V; //No of vertices
	bool isDirected;
	list<int> *adj; //adjcency list
	vector<int> InDegree;
	vector<int> OutDegree;

public:
	Graph() {};
	Graph(int n, bool dir=false) : V(n), isDirected(dir) 
	{
		adj = new list<int>[V]; 
		InDegree = vector<int>(V, 0);
		OutDegree = vector<int>(V, 0);
	}
	Graph(const Graph& g); //Copy Constructor
	Graph& operator = (const Graph & g); 
	

	~Graph() { delete[] adj; }

	void addEdge(int u, int v); //add edge from u to v
	void DFS(int v, vector<bool> &visited, bool print);
	void DFS_Util(int v, stack<int> &S, vector<bool> &visited);
	bool isConnected(); //checks if all non-zero degree vertices are connected
	void isEulerian();
	bool isStronglyConnected(); //used for directed graph;
	void isHamiltonianCycle();
	bool HamiltonianUtil(int v, vector<bool> &visited, vector<int> &path, int pos);
	int stronglyConnectedComponents();
	Graph getTranspose(); //Get a transpose or reverse of a graph; Used for identofying strongly connected
};

Graph::Graph(const Graph& g)
{
	//cout << "Copy Constructor called !!" << endl;
	V = g.V;
	isDirected = g.isDirected;
	InDegree = g.InDegree;
	OutDegree = g.OutDegree;
	adj = new list<int>[V];
	for (int i = 0; i < V; i++)
	{
		adj[i] = g.adj[i];
	}
}

Graph& Graph::operator=(const Graph &g)
{
	//cout << "Assignment operator called !!" << endl;
	if (this != &g)
	{
		this->V = g.V;
		this->isDirected = g.isDirected;
		this->InDegree = g.InDegree;
		this->OutDegree = g.OutDegree;
		adj = new list<int>[V];

		for (int i = 0; i < V; i++)
		{
			this->adj[i] = g.adj[i];
		}
	}

	return *this;
}

void Graph::addEdge(int u, int v)
{
	adj[u].push_back(v);
	InDegree[v]++;
	OutDegree[u]++;
	if (!isDirected)
	{
		adj[v].push_back(u);
		InDegree[u]++;
		OutDegree[v]++;
	}
}

void Graph::DFS(int v, vector<bool> &visited, bool print=false)
{
	if (visited[v]) return;
	visited[v] = true;
	if (print) { cout << v << " ";  }

	for (auto it = adj[v].begin(); it != adj[v].end(); ++it)
	{
		DFS(*it, visited, print);
	}
}

void Graph::DFS_Util(int v, stack<int> &S, vector<bool> &visited)
{
	//if (visited[v]) return;
	visited[v] = true;

	for (auto it = adj[v].begin(); it != adj[v].end(); ++it)
	{
		if (!visited[*it])
		{
			DFS_Util(*it, S, visited);
			S.push(*it);
		}
	}
	
}

bool Graph::isConnected()
{
	if (!isDirected)
	{
		//This algo would not work for directed Graph;
		//0->1->2->3 : Below Algo would show as stringly connected but it is not. There is no path from 3 to 1;
		//However it is fine in case of undirected graph

		//do a DFS from non-zero degree node and check if all the non-zero degree nodes can be visited
		vector<bool> visited(V, false);
		//update InDegree and OutDegree for each vertex;
		/*for (int i = 0; i < V; i++)
		{
			list<int> adjList = adj[i];
			for (auto it = adj[i].begin(); it != adj[i].end(); ++it)
			{
				OutDegree[i]++;
				InDegree[*it]++;
			}
		}*/
		//find a node which has non-zero indegree or outdegree
		int i = 0;
		for (i = 0; i < V; i++)
		{
			if (InDegree[i] != 0 || OutDegree[i] != 0)
			{
				break;
			}
		}

		if (i == V) //All nodes are isolated..there are no non-zero degree nodes
		{
			return true;
		}
		//Now do a DFS from i as i is non-zero degree node
		DFS(i, visited);

		//if all the non-zero nodes are visited that means..all the non-zero nodes are connected
		for (i = 0; i < V; i++)
		{
			if ((InDegree[i] != 0 || OutDegree[i] != 0) && !visited[i])
			{
				return false;
			}
		}

		return true;
	}
	else {
		//if each node can be visited from v and v can be visited from all vertices than graph is strongly connected.

		//Get a vertex with non-zero degree
		int i = 0;
		for (i = 0; i < V; i++)
		{
			if (InDegree[i] != 0 || OutDegree[i] != 0) break;
		}

		if (i == V) return false;
		vector<bool> visited(V, false);
		DFS(i, visited);

		for (int v = 0; v < V; v++)
		{
			if ((InDegree[v] != 0 || OutDegree[v] != 0) && !visited[v]) return false;
		}
		//if here...then all the non zero vertices can be visited from vertex i;
		//Now check if i can be visited from all the non-zero degree vertices..
		//i.e. all the non-zero vertices can be visted from i in reverse graph;

		Graph g = this->getTranspose();
		visited = vector<bool>(V, false);
		g.DFS(i, visited);

		for (int v = 0; v < V; v++)
		{
			if ((g.InDegree[v] != 0 || g.OutDegree[v] != 0) && !visited[v]) return false;
		}

		return true;
	}
}

bool Graph::isStronglyConnected() //KOSARAJU's algorithm
{
	if (!isDirected) return isConnected();
	//if each node can be visited from v and v can be visited from all vertices than graph is strongly connected.
	//Below is implementation of Kosaraju's algorithm to check if there is only a strongly connected component in graph
	vector<bool> visited(V, false);

	DFS(0, visited);

	for (int v = 0; v < V; v++)
	{
		if(!visited[v]) return false;
	}
	//if here...then all the vertices can be visited from vertex i;
	//Now check if i can be visited from all the vertices..i.e. all the vertices can be visted from i in reverse graph;

	Graph g = this->getTranspose();
	visited = vector<bool>(V, false);
	g.DFS(0, visited);

	for (int v = 0; v < V; v++)
	{
		if (!visited[v]) return false;
	}

	return true;
}

int Graph::stronglyConnectedComponents() //Identifies the strongly connected components in Graph
{
	//STRONGLY CONNECTED GRAPH: A graph is strongly connected if there is a path between all pairs of vertices.

	//Uses Kosaraju's algoithm. O(V+E)
	//Transpose graph has the same no of Strongly Connected Components (SCCs) as original graph
	//1. Insert each node into stack after doing DFS for it.
	//2. Now get a transpose of the graph
	//3. Pop the nodes from stack and keep visiting the tranposed graph.

	stack<int> S;
	vector<bool> visited(V, false);
	//start pushing the nodes to stack once we finish visiting them
	for (int i = 0; i < V; i++)
	{
		if (!visited[i])
		{
			DFS_Util(i, S, visited);
			S.push(i);
		}
	}
	
	//Reverse the Graph
	Graph g = this->getTranspose();
	
	visited = vector<bool>(V, false);
	int v = 0;
	int sscCount = 0;
	while (!S.empty())
	{
		v = S.top();
		S.pop();
		if (!visited[v])
		{
			//cout << "v"
			sscCount++;
			g.DFS(v, visited, true);
			cout << endl;
		}
	}

	return sscCount;
}

Graph Graph::getTranspose()
{
	Graph g(V, isDirected);
	
	for (int i = 0; i < V; i++)
	{
		for (auto it = adj[i].begin(); it != adj[i].end(); ++it)
		{
			g.adj[*it].push_back(i);
		}
	}
	g.InDegree = this->OutDegree;
	g.OutDegree = this->InDegree;

	return g;
}

void Graph::isEulerian()
{
	int res = 0;
	//all the non-zero vertices should be strongly connected
	if (isConnected() == false) { 
		cout << "Graph does not have Eulerian Path !!" << endl;
		return;
	}

	//All the non-zero vertices are strongly connected;
	if (isDirected)
	{
		//for directed graph..all the indgree and outDegree should be same
		//Count the number of vertices having diff InDegree and OutDegree
		int count = 0;
		for (int i = 0; i < V; i++)
		{
			if (InDegree[i] != OutDegree[i])
			{
				count++;
			}
		}
		if (count == 0)
		{
			res = 1; //Graph has Eularian Cycle
		}
		else if (count == 1) //Should never be the case
		{
			cout << "ERROR: only 1 verext having different InDegree and OutDegree !!" << endl;
			res = 0;
		}
		else if (count == 2)
		{
			res = 2; //Graph has Eulerian Path but not Eulerian Cycle
		}
		else {
			res = 0; //Graph does not have Eulerian Cycle.
		}
	}
	else //For undirected graph
	{
		//Count no of vertices with Odd degree
		int count = 0;
		for (int i = 0; i < V; i++)
		{
			if (InDegree[i] % 2 == 1)
			{
				count++;
			}
		}
		if (count == 0)
		{
			res= 1; //Graph is Eulerian
		}
		else if (count == 2)
		{
			res= 2; //Graph has Eularian Path but is not Eulerian
		}
		else {
			res= 0; //Graph does not have Eulerian Path
		}
	}

	if (res == 1)
	{
		cout << "Graph has Eulerian Cycle !!" << endl;
	}
	else if (res == 2)
	{
		cout << "Graph has Eulerian path but not Eulerian Cycle !!" << endl;
	}
	else {
		cout << "Graph does not have Eulerian Path!!" << endl;
	}
}

void Graph::isHamiltonianCycle()
{
	//HAMILTONIAN PATH: a path which visits every vertex exactly once
	//HAMILTONIAN CYCLE: a hamiltonian path such that there is an edge from last vertext to first vertex

	vector<bool> visited;
	vector<int> path;
	int res = 0;
	for (int v = 0; v < V; v++)
	{
		visited = vector<bool>(V, false);
		path = vector<int>(V, -1);
		path[0] = v;
		if (HamiltonianUtil(v, visited, path, 1))
		{
			res = 2;
			//Now check if there is an edge between last and first vertex of the path
			int lastV = path[V - 1];
			int firstV = path[0];
			list<int> adjList = this->adj[lastV];
			
			for (auto it = adjList.begin(); it != adjList.end(); ++it)
			{
				if (*it == firstV)
				{
					res = 1;
					break;
				}
			}

			if (res == 1)
			{
				cout << "Graph has Hamiltonian Cycle !!" << endl;
			}
			else { //res == 2;
				cout << "Graph has Hamiltonian path !!" << endl;
			}
			//print path
			for (int i = 0; i < V; i++)
			{
				cout << path[i] << " ";
			}
			cout << endl;
			break;
		}
	}

	if (res == 0)
	{
		cout << "Graph does not have any Hamiltonian path !!" << endl;
	}
}

bool Graph::HamiltonianUtil(int v, vector<bool> &visited, vector<int> &path, int pos)
{
	if (pos == V) return true;
	//if (visited[v]) return;
	//mark v as visited
	visited[v] = true;

	//Now visit the unvisited adj nodes of v
	list<int> adjList = this->adj[v];
	for (auto it = adjList.begin(); it != adjList.end(); ++it)
	{
		if (!visited[*it])
		{
			path[pos] = *it;
			if (HamiltonianUtil(*it, visited, path, pos+1))
			{
				return true;
			}
			else {
				//do backtrack..
				path[pos] = -1;
				visited[*it] = false;
			}
		}
	}

	return false;
}

int main()
{
	int i = 1; 
	cout << "GRAPH - " << i++ << endl;
	Graph g1(5, true);
	g1.addEdge(1, 0);
	g1.addEdge(0, 2);
	g1.addEdge(2, 1);
	g1.addEdge(0, 3);
	g1.addEdge(3, 4);
	g1.isEulerian();
	cout << "SSC : " << g1.stronglyConnectedComponents() << endl;
	g1.isHamiltonianCycle();

	cout << "GRAPH - " << i++ << endl;
	Graph g2(5, true);
	g2.addEdge(1, 0);
	g2.addEdge(0, 2);
	g2.addEdge(2, 1);
	g2.addEdge(0, 3);
	g2.addEdge(3, 4);
	g2.addEdge(4, 0);
	g2.isEulerian();
	g2.isHamiltonianCycle();

	cout << "GRAPH - " << i++ << endl;
	Graph g3(5, true);
	g3.addEdge(1, 0);
	g3.addEdge(0, 2);
	g3.addEdge(2, 1);
	g3.addEdge(0, 3);
	g3.addEdge(3, 4);
	g3.addEdge(1, 3);
	g3.isEulerian();
	cout << "SSC : " << g3.stronglyConnectedComponents() << endl;
	g3.isHamiltonianCycle();
	// Let us create a graph with 3 vertices
	// connected in the form of cycle
	cout << "GRAPH - " << i++ << endl;
	Graph g4(3);
	g4.addEdge(0, 1);
	g4.addEdge(1, 2);
	g4.addEdge(2, 0);
	g4.isEulerian();
	g4.isHamiltonianCycle();
	// Let us create a graph with all veritces
	// with zero degree
	cout << "GRAPH - " << i++ << endl;
	Graph g5(3);
	g5.isEulerian();
	g5.isHamiltonianCycle();

	cout << "GRAPH - " << i++ << endl;
	Graph g6(6, true);
	g6.addEdge(0, 1);
	g6.addEdge(1, 2);
	g6.addEdge(2, 0);
	g6.addEdge(3, 4);
	g6.addEdge(4, 5);
	g6.addEdge(5, 3);
	g6.isEulerian();
	g6.isHamiltonianCycle();

	cout << "GRAPH - " << i++ << endl;
	Graph g7(6, true);
	g7.addEdge(4, 2);
	g7.addEdge(2, 3);
	g7.addEdge(3, 0);
	g7.addEdge(0, 5);
	g7.addEdge(5, 1);
	g7.addEdge(1, 4);
	g7.isEulerian();
	cout << "SSC : " << g7.stronglyConnectedComponents() << endl;
	g7.isHamiltonianCycle();

	cout << "GRAPH - " << i++ << endl;
	Graph g8(4, true);
	g8.addEdge(0, 1);
	g8.addEdge(1, 2);
	g8.addEdge(2, 0);
	g8.addEdge(0, 3);
	g8.addEdge(3, 2);
	g8.isEulerian();
	cout << "SSC : " << g8.stronglyConnectedComponents() << endl;
	g8.isHamiltonianCycle();

	cout << "GRAPH - " << i++ << endl;
	Graph g9(5, true);
	g9.addEdge(0, 1);
	g9.addEdge(1, 2);
	g9.addEdge(2, 0);
	g9.addEdge(0, 3);
	g9.addEdge(3, 2);
	g9.addEdge(1, 4);
	g9.addEdge(4, 2);
	g9.isEulerian();
	cout << "SSC : " << g9.stronglyConnectedComponents() << endl;
	g9.isHamiltonianCycle();

	cout << "GRAPH - " << i++ << endl;
	Graph g10(8, true);
	g10.addEdge(0, 1);
	g10.addEdge(1, 2);
	g10.addEdge(2, 0);

	g10.addEdge(3, 4);
	g10.addEdge(4, 5);
	g10.addEdge(5, 3);
	g10.isEulerian();
	cout << "g10->SSC : " << g10.stronglyConnectedComponents() << endl;
	g10.isHamiltonianCycle();

	cout << "GRAPH - " << i++ << endl;
	Graph g11(10);
	g11.addEdge(1, 0);
	g11.addEdge(9, 1);
	g11.addEdge(5, 2);
	g11.addEdge(4, 3);
	g11.addEdge(9, 4);
	g11.addEdge(9, 5);
	g11.addEdge(5, 6);
	g11.addEdge(5, 7);
	g11.addEdge(9, 8);
	g11.addEdge(3, 8);
	g11.addEdge(2, 7);
	g11.addEdge(2, 6);
	g11.addEdge(4, 8);
	g11.addEdge(5, 4);
	g11.isHamiltonianCycle();


    return 0;
}

