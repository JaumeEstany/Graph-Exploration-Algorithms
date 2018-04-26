#include "stdafx.h"
#include "CGraph.h"
#include <algorithm>
#include <queue>

//function that recieves 2 pointers to vertex and returns true if the first vertex was bigger: it's used by std::sort in the dijkstra without queue
inline bool vertexDistanceComparison(CVertex* v1, CVertex* v2) {
	return (v1->m_DijkstraDistance > v2->m_DijkstraDistance);
}

//class tupla that stores a pointer to vertex and the dijkstra value it had when was added
class tupla {
	//everything public
public:
	//constructor: recieves a pointer to vertex and copies it inside de tuple. Also copies the current value, so if its changed it doesnt matter
	tupla(CVertex* const vertex){
		this->vertex = vertex;
		this->distance = vertex->m_DijkstraDistance;
	}

	CVertex* vertex;
	double distance;
};

//class myComparison that it's only used to be able to pass a class that recieves 2 tupla references and returns a bool saying if the first one is bigger than the other one
//used by the priority_queue; passed as a template parameter so it can recognise the bigger out of 2
class myComparison {
public:
	inline bool operator() (tupla &v1, tupla &v2){
		return (v1.distance > v2.distance);
	}

};


// =============================================================================
// Dijkstra ====================================================================
// =============================================================================

void CGraph::Dijkstra(CVertex *pStart)
{
	//we create an empty vector that will be our dijkstra list
	vector<CVertex*> pendantList;
	/*
	for every vertex existing in the graph, we init all its values:
	m_DijkstraDistance = infinite;
	m_DijkstraVisit = false;
	m_pDijkstraPrevious = nullptr;
	*/
	for (CVertex &vert : m_Vertices) {
		vert.m_DijkstraDistance = numeric_limits<double>::max(); //Returns the max value a double can hold
		vert.m_DijkstraVisit = false;
		vert.m_pDijkstraPrevious = nullptr;

		//we also add it to the pendant list
		pendantList.push_back(&vert);
	}

	//we set the value of the starting node at 0
	pStart->m_DijkstraDistance = 0;

	//while the list is not empty (not finished)
	while (!pendantList.empty()) {
		//we sort the vector in order to get the lesser valued node at the front
		sort(pendantList.begin(), pendantList.end(), vertexDistanceComparison);

		//we get the reference "closer" and set its visited value to true
		CVertex* const curr_vertex = pendantList.back();
		curr_vertex->m_DijkstraVisit = true;
		//we delete it from the list
		pendantList.pop_back();

		//for every neighbour of the current vertex:
		for (CVertex* neigh : curr_vertex->m_Neighbords) {
			//if it hasnt been visited yet
			if (!neigh->m_DijkstraVisit) {
				//calculate the edge cost and the total cost (edge+accumulated)
				CGPoint aux = neigh->m_Point;
				aux -= curr_vertex->m_Point;
				const double edge_distance = aux.Module();
				const double total_cost = edge_distance + curr_vertex->m_DijkstraDistance;

				//if it's smaller than the current cost of that node, replace it's cost and the previous node to the current ones
				if (total_cost < neigh->m_DijkstraDistance) {
					neigh->m_DijkstraDistance = total_cost;
					neigh->m_pDijkstraPrevious = curr_vertex;
				}
			}
		}	
	}

}

// =============================================================================
// DijkstraQueue ===============================================================
// =============================================================================

void CGraph::DijkstraQueue(CVertex *pStart)
{
	//we create an empty priority queue, containing tupla elements, using a std::vector as internal container and using myComparison to compare elements
	priority_queue<tupla, vector<tupla>, myComparison> nodeQueue;

	//for every vertex in the graph we init its values just as before (infinite)(false)(nullptr)
	//PS: we are not adding them all this time
	for (CVertex &vert : m_Vertices) {
		vert.m_DijkstraDistance = numeric_limits<double>::max(); //Returns the max value a double can hold
		vert.m_DijkstraVisit = false;
		vert.m_pDijkstraPrevious = nullptr;
	}

	//we init the starting node value at 0 and push it into the priority queue
	pStart->m_DijkstraDistance = 0;
	nodeQueue.push(tupla(pStart));

	//while the queue is not empty:
	while (!nodeQueue.empty()) {

		//get the value "closer"
		tupla curr_vertex = nodeQueue.top();
		//pop it out
		nodeQueue.pop();

		//if it hasnt been visited yet
		if (!curr_vertex.vertex->m_DijkstraVisit) {
			//set it as visited
			curr_vertex.vertex->m_DijkstraVisit = true;

			//and for every node it has
			for (CVertex* neigh : curr_vertex.vertex->m_Neighbords) {
				//if it hasnt been visited
				if (!neigh->m_DijkstraVisit) {
					//calculate the edge cost and the total cost (edge+accumulated)
					CGPoint aux = neigh->m_Point;
					aux -= curr_vertex.vertex->m_Point;
					const double edge_distance = aux.Module();
					const double total_cost = edge_distance + curr_vertex.vertex->m_DijkstraDistance;

					//if the cost is better, update its values and push it in the queue
					if (total_cost < neigh->m_DijkstraDistance) {
						neigh->m_DijkstraDistance = total_cost;
						neigh->m_pDijkstraPrevious = curr_vertex.vertex;

						nodeQueue.push(tupla(neigh));
					}
				}
			}
		}
	}
}


