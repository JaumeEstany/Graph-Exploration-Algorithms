#include "stdafx.h"
#include "CGraph.h"
#include <unordered_set>
#include <map>

// SalesmanTrackBacktracking ===================================================


void PureBacktracking_getToLastNodeBacktracking(CVertex* const currVertex, CVertex* const lastVertex, unordered_set<CVertex*> &visitedVertexes,
												CTrack &currSolution, CTrack &bestSolution, double acc_cost, double &best_cost) {

	const bool onTarget = (currVertex == lastVertex);


	if (acc_cost >= best_cost) {
		return;
	}

	if (onTarget) {

		bestSolution = currSolution;
		best_cost = acc_cost;
		bestSolution.m_Vertices.push_back(currVertex);
		return;
	}

	currSolution.m_Vertices.push_back(currVertex);
	visitedVertexes.insert(currVertex);
	//============================================================================================================


	for (CVertex* const neigh : currVertex->m_Neighbords) {
		if (!visitedVertexes.count(neigh)) {
			CGPoint aux = neigh->m_Point;
			aux -= currVertex->m_Point;
			const double move_cost = aux.Module();

			PureBacktracking_getToLastNodeBacktracking(neigh, lastVertex, visitedVertexes, currSolution, bestSolution, acc_cost + move_cost, best_cost);
		}
	}


	//============================================================================================================
	visitedVertexes.erase(currVertex);
	currSolution.m_Vertices.pop_back();


}

double PureBacktracking_getToLastNode(CVertex* const startVertex, CVertex* const lastVertex, CTrack &solution) {


	unordered_set<CVertex*> visitedVertexes;
	CTrack currSolution;
	double best_cost = numeric_limits<double>::max();

	PureBacktracking_getToLastNodeBacktracking(startVertex, lastVertex, visitedVertexes, currSolution, solution, 0, best_cost);

	return best_cost;
}

void PureBacktracking(CVertex* const currVertex, CVertex* const lastVertex, unordered_set<CVertex*> &visitedSinceLastTarget, unordered_set<CVertex*> &remainingTargets,
						CTrack &currSolution, CTrack &bestSolution, double acc_cost, double &best_cost) {

	const bool onNewTarget = (remainingTargets.count(currVertex));

	if (acc_cost >= best_cost) {
		return;
	}

	unordered_set<CVertex*> &visitedSinceLastTargetBelow = (onNewTarget) ? (unordered_set<CVertex*>()) : (visitedSinceLastTarget);

	if (onNewTarget) {

		if (remainingTargets.size()==1) {		//this is a candidate solution
			
			CTrack finalPartTrack;
			const double lastPartCost = PureBacktracking_getToLastNode(currVertex, lastVertex, finalPartTrack);

			if (acc_cost + lastPartCost < best_cost) {		//this is a solution
				//update the best solution found
				best_cost = acc_cost + lastPartCost;
				bestSolution = currSolution;
				bestSolution.m_Vertices.splice(bestSolution.m_Vertices.end(), finalPartTrack.m_Vertices);
			}

			return;
		}

		remainingTargets.erase(currVertex);
	}

	currSolution.m_Vertices.push_back(currVertex);
	visitedSinceLastTargetBelow.insert(currVertex);
	//============================================================================================================


	for (CVertex* const neigh : currVertex->m_Neighbords) {
		if (!visitedSinceLastTargetBelow.count(neigh)) {
			CGPoint aux = neigh->m_Point;
			aux -= currVertex->m_Point;
			const double move_cost = aux.Module();

			PureBacktracking(neigh, lastVertex, visitedSinceLastTargetBelow, remainingTargets, currSolution, bestSolution, acc_cost+move_cost, best_cost);
		}
	}


	//============================================================================================================
	visitedSinceLastTargetBelow.erase(currVertex);
	currSolution.m_Vertices.pop_back();
	if (onNewTarget) {
		remainingTargets.insert(currVertex);
	}

}

CTrack CGraph::SalesmanTrackBacktracking(CVisits &visits)
{

	CVisits toVisit = visits;

	//add to it the first node and remove it from visits
	CVertex* const firstVertex = toVisit.m_Vertices.front();
	toVisit.m_Vertices.pop_front();

	//store the finalVertex here and remove it from visits
	CVertex* const lastVertex = toVisit.m_Vertices.back();
	toVisit.m_Vertices.pop_back();


	unordered_set<CVertex*> visitedSinceLastTarget;
	unordered_set<CVertex*> remainingTargets(toVisit.m_Vertices.size());
	CTrack currSolution;
	CTrack bestSolution;
	double best_cost = numeric_limits<double>::max();

	for (CVertex* const v : toVisit.m_Vertices) {
		remainingTargets.insert(v);
	}


	if (toVisit.m_Vertices.size()) {
		PureBacktracking(firstVertex, lastVertex, visitedSinceLastTarget, remainingTargets, currSolution, bestSolution, 0, best_cost);
	}
	else {
		PureBacktracking_getToLastNode(firstVertex, lastVertex, bestSolution);
	}
	
	return bestSolution;
}

// SalesmanTrackBacktrackingGreedy =============================================

void GreedyBacktracking(CVertex* const currVertex, CVertex* const lastVertex, list<CVertex*> &toVisit,
	vector<CVertex*> &currSolution, vector<CVertex*> &bestSolution, double acc_cost, double &best_cost, int steps) {


	//if it already costs more, step back
	if (acc_cost >= best_cost) {
		return;
	}

	if (steps >= toVisit.size()) {		//if we've added all the intermediate vertexes;  the -1 is because of the firstVertex also being added

		const double solution_cost = acc_cost + currVertex->m_toLastVertexCost;

		if (solution_cost<best_cost) {		//if it's best solution
			bestSolution = currSolution;
			bestSolution.push_back(currVertex);
			best_cost = solution_cost;
		}

		return;
	}


	currSolution.push_back(currVertex);
	currVertex->m_VertexToVisit = false;

	for (CVertex* vert : toVisit) {
		if (vert->m_VertexToVisit) {
			pair<CVertex*, CVertex*> aux(currVertex, vert);
			GreedyBacktracking(vert, lastVertex, toVisit, currSolution, bestSolution, acc_cost + currVertex->m_fasterDijkstraCosts[vert->m_visitID], best_cost, steps + 1);
		}
	}

	currVertex->m_VertexToVisit = true;
	currSolution.pop_back();


}

CTrack CGraph::SalesmanTrackBacktrackingGreedy(CVisits &visits)
{

	CVisits toVisit = visits;
	{
		int i = 0;
		for (CVertex* v : visits.m_Vertices) {
			v->m_visitID = i;
			i++;
		}
	}

	{
		const int NUM_OF_VISITS = (int)visits.m_Vertices.size();
		for (CVertex* first : visits.m_Vertices) {
			DijkstraQueue(first);
			vector<double> &aux = first->m_fasterDijkstraCosts;

			aux.insert(aux.end(), NUM_OF_VISITS, 0.0);
			for (CVertex* second : visits.m_Vertices) {
				aux[second->m_visitID] = second->m_DijkstraDistance;
			}
		}
	}
	//add to it the first node and remove it from visits
	CVertex* const firstVertex = toVisit.m_Vertices.front();
	toVisit.m_Vertices.pop_front();

	//store the finalVertex here and remove it from visits
	CVertex* const lastVertex = toVisit.m_Vertices.back();
	toVisit.m_Vertices.pop_back();

	DijkstraQueue(lastVertex);

	for (CVertex* v : visits.m_Vertices) {
		v->m_VertexToVisit = true;
		v->m_toLastVertexCost = v->m_DijkstraDistance;
	}

	vector<CVertex*> currSolution;
	vector<CVertex*> bestCombination;
	currSolution.reserve(visits.m_Vertices.size());
	currSolution.reserve(visits.m_Vertices.size());

	double best_cost = numeric_limits<double>::max();


	GreedyBacktracking(firstVertex, lastVertex, toVisit.m_Vertices, currSolution, bestCombination, 0.0, best_cost, 0);

	bestCombination.push_back(lastVertex);

	CTrack ret;

	ret.m_Vertices.push_back(firstVertex);
	for (int i = 0; i<bestCombination.size() - 1; i++) {

		DijkstraQueue(bestCombination[i]);

		list<CVertex*> aux;
		CVertex* iter = bestCombination[i + 1];

		while (iter != nullptr) {

			aux.push_front(iter);
			iter = iter->m_pDijkstraPrevious;
		}
		aux.pop_front();

		ret.m_Vertices.splice(ret.m_Vertices.end(), aux);
	}

	cout << best_cost << endl;

	return ret;
}


