#include "stdafx.h"
#include "CGraph.h"
#include <stack>

// SalesmanTrackGreedy =========================================================

//#define CALC_AND_PRINT_GREEDY_COST

//this function gets the closest pendant to visit node pointer, removes it from the CVisits and returns it
CVertex* getAndDeleteClosestVisit(CVisits &visits) {

	//initial state to get the min
	CVertex* min_vertex = nullptr;
	list<CVertex*>::iterator min_iter;
	double min_value = numeric_limits<double>::max();

	//iter
	list<CVertex*>::iterator iter;

	//for every node in the list
	for (iter = visits.m_Vertices.begin(); iter != visits.m_Vertices.end(); ++iter) {
		//if the value is smaller than the min, change the min
		if ((*iter)->m_DijkstraDistance < min_value) {
			min_value = (*iter)->m_DijkstraDistance;
			min_vertex = (*iter);
			min_iter = iter;
		}
	}

	//remove it from the list
	visits.m_Vertices.erase(min_iter);

	//return it
	return min_vertex;
}

//this function adds the unroll of the vertex into the track
//will do it in inverse order, so it makes sense
void addPathToTrack(CTrack &track, CVertex* vertex) {
	
	//stack to push the pointers to get them in inverse order
	stack<CVertex*, vector<CVertex*>> pila;
	//curr pointer that will be the iterator
	CVertex* curr = vertex;
	
	//while the previous of this node is not null (the starting node)
	while (curr->m_pDijkstraPrevious != nullptr) {
		pila.push(curr);
		curr = curr->m_pDijkstraPrevious;
	}

	//add every element in the stack until it's empty
	while (!pila.empty()) {
		track.AddLast(pila.top());
		pila.pop();
	}

}

//main function
CTrack CGraph::SalesmanTrackGreedy(CVisits &visits) 
{
	//create an empty track
	CTrack track(this);

	//add to it the first node and remove it from visits
	track.AddLast(visits.m_Vertices.front());
	visits.m_Vertices.pop_front();
		
	//store the finalVertex here and remove it from visits
	CVertex* const finalVertex = visits.m_Vertices.back();
	visits.m_Vertices.pop_back();
	
	//while visits is not empty
	while (!visits.m_Vertices.empty()) {
		
		DijkstraQueue(track.m_Vertices.back());			//do the dijkstra
		
		//get the closest node
		CVertex* const nextVisit = getAndDeleteClosestVisit(visits);
		
		//add all the path until there to the track
		addPathToTrack(track, nextVisit);

	}

	//do the same as before with the finalVertex
	DijkstraQueue(track.m_Vertices.back());
	addPathToTrack(track, finalVertex);

#ifdef CALC_AND_PRINT_GREEDY_COST

	list<CVertex*> &nodes = track.m_Vertices;

	auto iter1 = nodes.begin();
	auto iter2 = iter1;
	iter1++;

	double acc = 0.0;

	while (iter1!=nodes.end()) {

		CGPoint aux = (*iter1)->m_Point;

		aux -= (*iter2)->m_Point;

		acc += aux.Module();

		iter2 = iter1;
		iter1++;
	}

	cout << acc << endl;

#endif // CALC_AND_PRINT_GREEDY_COST


	return track;
}
