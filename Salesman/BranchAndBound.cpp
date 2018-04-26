#include "stdafx.h"
#include "CGraph.h"

#include <memory>
#include <utility>


typedef int indexType;

typedef double costType;

int NUM_OF_VISITS = 0;

struct Heuristic3Tuple {

	CVertex* vertex;
	costType cost;

};

class PartialSolution {

public:

		indexType* indexesInsidePath;
		int visitedTargets;

		costType heuristic;
		costType acc_cost;

		CVertex* current;

		PartialSolution() {}

		~PartialSolution() {
			delete[] indexesInsidePath;
		}

		bool isCompleteSolution() const {
			return (visitedTargets >= NUM_OF_VISITS);
		}

		virtual void assignHeuristic() =0;

};


class PartialSolution1 : public PartialSolution {
	public:

		PartialSolution1(list<CVertex*> intermediateVisits, CVertex* firstVertex) {

			visitedTargets = 1;
			indexesInsidePath = new indexType[NUM_OF_VISITS];

			for (int i = 1; i < NUM_OF_VISITS; i++) {
				indexesInsidePath[i] = -1;
			}
			indexesInsidePath[0] = 0;
			indexesInsidePath[NUM_OF_VISITS - 1] = NUM_OF_VISITS - 1;

			current = firstVertex;
			heuristic = 0.0;
			acc_cost = 0.0;

		}

		PartialSolution1(const PartialSolution1 &father, CVertex* nextVertex) {

			indexesInsidePath = new indexType[NUM_OF_VISITS];

			for (int i = 0; i<NUM_OF_VISITS; i++) {
				indexesInsidePath[i] = father.indexesInsidePath[i];
			}

			indexesInsidePath[nextVertex->m_visitID] = father.visitedTargets;

			visitedTargets = father.visitedTargets + 1;

			current = nextVertex;
			heuristic = 0.0;

			acc_cost = father.acc_cost + father.current->m_fasterDijkstraCosts[nextVertex->m_visitID];

		}

		void assignHeuristic() {
			heuristic = acc_cost;
		}

};

class PartialSolution2 : public PartialSolution {
public:

	costType heuristic2yet;

	PartialSolution2(list<CVertex*> intermediateVisits, CVertex* firstVertex, costType sum_min_vertexs_cost) {

		visitedTargets = 1;
		indexesInsidePath = new indexType[NUM_OF_VISITS];

		for (int i = 1; i < NUM_OF_VISITS; i++) {
			indexesInsidePath[i] = -1;
		}
		indexesInsidePath[0] = 0;
		indexesInsidePath[NUM_OF_VISITS - 1] = NUM_OF_VISITS - 1;

		current = firstVertex;
		heuristic = 0.0;
		acc_cost = 0.0;

		heuristic2yet = sum_min_vertexs_cost;
	}

	PartialSolution2(const PartialSolution2 &father, CVertex* nextVertex, costType* min_vertexs_cost) {

		indexesInsidePath = new indexType[NUM_OF_VISITS];

		for (int i = 0; i<NUM_OF_VISITS; i++) {
			indexesInsidePath[i] = father.indexesInsidePath[i];
		}

		indexesInsidePath[nextVertex->m_visitID] = father.visitedTargets;

		visitedTargets = father.visitedTargets + 1;

		current = nextVertex;
		heuristic = 0.0;

		heuristic2yet = father.heuristic2yet - min_vertexs_cost[nextVertex->m_visitID];
		acc_cost = father.acc_cost + father.current->m_fasterDijkstraCosts[nextVertex->m_visitID];


	}

	void assignHeuristic() {
		heuristic = acc_cost + heuristic2yet;
	}

};

class PartialSolution3 : public PartialSolution {
public:

	costType heuristic3yet;

	Heuristic3Tuple* heuristic3_min_costs;

	PartialSolution3(list<CVertex*> intermediateVisits, CVertex* firstVertex, costType sum_min_vertexs_cost, Heuristic3Tuple* heuristic3_min_costs_p) {

		visitedTargets = 1;
		heuristic3_min_costs = heuristic3_min_costs_p;
		indexesInsidePath = new indexType[NUM_OF_VISITS];

		for (int i = 1; i < NUM_OF_VISITS; i++) {
			indexesInsidePath[i] = -1;
		}
		indexesInsidePath[0] = 0;
		indexesInsidePath[NUM_OF_VISITS - 1] = NUM_OF_VISITS - 1;

		current = firstVertex;
		heuristic = 0.0;
		acc_cost = 0.0;

		heuristic3yet = sum_min_vertexs_cost;
	}

	PartialSolution3(const PartialSolution3 &father, CVertex* nextVertex, CVisits &visits) {

		indexesInsidePath = new indexType[NUM_OF_VISITS];
		heuristic3_min_costs = new Heuristic3Tuple[NUM_OF_VISITS];

		for (int i = 0; i<NUM_OF_VISITS; i++) {
			indexesInsidePath[i] = father.indexesInsidePath[i];
			heuristic3_min_costs[i] = father.heuristic3_min_costs[i];
		}

		indexesInsidePath[nextVertex->m_visitID] = father.visitedTargets;

		visitedTargets = father.visitedTargets + 1;

		current = nextVertex;
		heuristic = 0.0;

		heuristic3yet = father.heuristic3yet - heuristic3_min_costs[nextVertex->m_visitID].cost;

		//update

		for (CVertex* target : visits.m_Vertices) {
			const int index = target->m_visitID;
			if (indexesInsidePath[index] == -1 && heuristic3_min_costs[index].vertex == nextVertex) {		//if node not visited AND it's closest is being added to the path

				CVertex* curr_min_vertex = nullptr;
				costType curr_min_dist = numeric_limits<costType>::max();

				for (CVertex* second : visits.m_Vertices) {		//iterate through every target
					const int index2 = second->m_visitID;
					if (indexesInsidePath[index]) {				//if it's not visited
						const costType currDist = target->m_fasterDijkstraCosts[second->m_visitID];
						const bool cond = (currDist < curr_min_dist && target != second);
						curr_min_dist = (cond) ? (currDist) : (curr_min_dist);
						curr_min_vertex = (cond) ? (second) : (curr_min_vertex);
					}
				}
			}
		}
		


		acc_cost = father.acc_cost + father.current->m_fasterDijkstraCosts[nextVertex->m_visitID];


	}

	~PartialSolution3() {
		delete[] heuristic3_min_costs;
	}

	void assignHeuristic() {
		heuristic = acc_cost + heuristic3yet;
	}

};


class PartialSolutionComparison1 {
public:
	inline bool operator() (unique_ptr<PartialSolution1> &v1, unique_ptr<PartialSolution1> &v2) {
		return (v1->heuristic > v2->heuristic);
	}

};
class PartialSolutionComparison2 {
public:
	inline bool operator() (unique_ptr<PartialSolution2> &v1, unique_ptr<PartialSolution2> &v2) {
		return (v1->heuristic > v2->heuristic);
	}

};
class PartialSolutionComparison3 {
public:
	inline bool operator() (unique_ptr<PartialSolution3> &v1, unique_ptr<PartialSolution3> &v2) {
		return (v1->heuristic > v2->heuristic);
	}

};


// SalesmanTrackBranchAndBound1 =================================================================================================================================
CTrack CGraph::SalesmanTrackBranchAndBound1(CVisits &visits)
{
	//give every target an index
	{
		int i = 0;
		for (CVertex* v : visits.m_Vertices) {
			v->m_visitID = i;
			i++;
		}
	}

	//num to visit including first and last
	NUM_OF_VISITS = (int)visits.m_Vertices.size();

	//init all the m_fasterDijkstraCosts for the visited nodes
	{
		int i = 0;
		for (CVertex* first : visits.m_Vertices) {
			DijkstraQueue(first);
			vector<costType> &aux = first->m_fasterDijkstraCosts;
			aux.insert(aux.end(), NUM_OF_VISITS, 0.0);

			costType curr_min_dist = numeric_limits<costType>::max();

			for (CVertex* second : visits.m_Vertices) {
				const costType currDist = second->m_DijkstraDistance;
				aux[second->m_visitID] = currDist;

				curr_min_dist = (currDist<curr_min_dist && first!=second) ? (currDist) : (curr_min_dist);
			}

			i++;
		}
	}

	CVisits toVisit = visits;

	//add to it the first node and remove it from visits
	CVertex* const firstVertex = toVisit.m_Vertices.front();
	toVisit.m_Vertices.pop_front();

	//store the finalVertex here and remove it from visits
	CVertex* const lastVertex = toVisit.m_Vertices.back();
	toVisit.m_Vertices.pop_back();

	priority_queue<unique_ptr<PartialSolution1>, vector<unique_ptr<PartialSolution1>>, PartialSolutionComparison1> queue;

	//===============================================================================================================================================


	//push init solution
	{
		unique_ptr<PartialSolution1> aux(new PartialSolution1(toVisit.m_Vertices, firstVertex));
		aux->assignHeuristic();
		queue.push( std::move(aux) );
	}


	const PartialSolution1 *curr = nullptr;
	//expand the queue
	while ( (curr = queue.top().get()), !(curr->isCompleteSolution()) ) {

		if (curr->visitedTargets == (NUM_OF_VISITS-1)) {		//if only the last remains
			unique_ptr<PartialSolution1> aux(new PartialSolution1(*curr, lastVertex));
			aux->assignHeuristic();
			queue.pop();
			queue.push( std::move(aux) );
		}
		else {

			vector<PartialSolution1*> auxList;
			auxList.reserve(toVisit.m_Vertices.size() - curr->visitedTargets + 1);

			for (CVertex* target : toVisit.m_Vertices) {
				if (curr->indexesInsidePath[target->m_visitID] == -1) {
					auxList.push_back(new PartialSolution1(*curr, target));
				}				
			}

			queue.pop();

			for (PartialSolution1 *child : auxList) {
				child->assignHeuristic();
				queue.push( std::move(unique_ptr<PartialSolution1>(child)) );
			}

		}

	}

	//===============================================================================================================================================

	//fill ret with the whole path
	vector<CVertex*> bestCombination;
	bestCombination.resize(NUM_OF_VISITS, nullptr);

	for (CVertex* target : toVisit.m_Vertices) {
		bestCombination[curr->indexesInsidePath[target->m_visitID]] = target;
	}

	bestCombination[0] = firstVertex;
	bestCombination[bestCombination.size() - 1] = lastVertex;

	CTrack ret;

	ret.m_Vertices.push_back(firstVertex);
	for (int i = 0; i<((int)bestCombination.size() - 1); i++) {

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

	cout << curr->acc_cost << endl;

	return ret;
}

// SalesmanTrackBranchAndBound2 =================================================================================================================================
CTrack CGraph::SalesmanTrackBranchAndBound2(CVisits &visits)
{
	//give every target an index
	{
		int i = 0;
		for (CVertex* v : visits.m_Vertices) {
			v->m_visitID = i;
			i++;
		}
	}

	//num to visit including first and last
	NUM_OF_VISITS = (int)visits.m_Vertices.size();

	costType* const min_vertexs_cost = new costType[NUM_OF_VISITS];
	costType sum_min_vertexs_cost = 0.0;

	//init all the m_fasterDijkstraCosts for the visited nodes
	{
		int i = 0;
		for (CVertex* first : visits.m_Vertices) {
			DijkstraQueue(first);
			vector<costType> &aux = first->m_fasterDijkstraCosts;
			aux.insert(aux.end(), NUM_OF_VISITS, 0.0);

			costType curr_min_dist = numeric_limits<costType>::max();

			for (CVertex* second : visits.m_Vertices) {
				const costType currDist = second->m_DijkstraDistance;
				aux[second->m_visitID] = currDist;

				curr_min_dist = (currDist<curr_min_dist && first != second) ? (currDist) : (curr_min_dist);
			}

			sum_min_vertexs_cost += curr_min_dist;
			min_vertexs_cost[i] = curr_min_dist;

			i++;
		}
	}

	CVisits toVisit = visits;

	//add to it the first node and remove it from visits
	CVertex* const firstVertex = toVisit.m_Vertices.front();
	toVisit.m_Vertices.pop_front();

	//store the finalVertex here and remove it from visits
	CVertex* const lastVertex = toVisit.m_Vertices.back();
	toVisit.m_Vertices.pop_back();

	priority_queue<unique_ptr<PartialSolution2>, vector<unique_ptr<PartialSolution2>>, PartialSolutionComparison2> queue;

	//===============================================================================================================================================


	//push init solution
	{
		unique_ptr<PartialSolution2> aux(new PartialSolution2(toVisit.m_Vertices, firstVertex, sum_min_vertexs_cost));
		aux->assignHeuristic();
		queue.push(std::move(aux));
	}


	const PartialSolution2 *curr = nullptr;
	//expand the queue
	while ((curr = queue.top().get()), !(curr->isCompleteSolution())) {

		if (curr->visitedTargets == (NUM_OF_VISITS - 1)) {		//if only the last remains
			unique_ptr<PartialSolution2> aux(new PartialSolution2(*curr, lastVertex, min_vertexs_cost));
			aux->assignHeuristic();
			queue.pop();
			queue.push(std::move(aux));
		}
		else {

			vector<PartialSolution2*> auxList;
			auxList.reserve(toVisit.m_Vertices.size() - curr->visitedTargets + 1);

			for (CVertex* target : toVisit.m_Vertices) {
				if (curr->indexesInsidePath[target->m_visitID] == -1) {
					auxList.push_back(new PartialSolution2(*curr, target, min_vertexs_cost));
				}
			}

			queue.pop();

			for (PartialSolution2 *child : auxList) {
				child->assignHeuristic();
				queue.push(std::move(unique_ptr<PartialSolution2>(child)));
			}

		}

	}

	//===============================================================================================================================================

	//fill ret with the whole path
	vector<CVertex*> bestCombination;
	bestCombination.resize(NUM_OF_VISITS, nullptr);

	for (CVertex* target : toVisit.m_Vertices) {
		bestCombination[curr->indexesInsidePath[target->m_visitID]] = target;
	}

	bestCombination[0] = firstVertex;
	bestCombination[bestCombination.size() - 1] = lastVertex;

	CTrack ret;

	ret.m_Vertices.push_back(firstVertex);
	for (int i = 0; i<((int)bestCombination.size() - 1); i++) {

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

	cout << curr->acc_cost << endl;

	delete[] min_vertexs_cost;

	return ret;
}

// SalesmanTrackBranchAndBound3 =================================================================================================================================
CTrack CGraph::SalesmanTrackBranchAndBound3(CVisits &visits)
{
	//give every target an index
	{
		int i = 0;
		for (CVertex* v : visits.m_Vertices) {
			v->m_visitID = i;
			i++;
		}
	}

	//num to visit including first and last
	NUM_OF_VISITS = (int)visits.m_Vertices.size();

	//init all the m_fasterDijkstraCosts for the visited nodes
	{
		int i = 0;
		for (CVertex* first : visits.m_Vertices) {
			DijkstraQueue(first);
			vector<costType> &aux = first->m_fasterDijkstraCosts;
			aux.insert(aux.end(), NUM_OF_VISITS, 0.0);

			for (CVertex* second : visits.m_Vertices) {
				const costType currDist = second->m_DijkstraDistance;
				aux[second->m_visitID] = currDist;

			}

		}
	}

	Heuristic3Tuple* const heuristic3_min_costs = new Heuristic3Tuple[NUM_OF_VISITS];
	costType sum_min_vertexs_cost = 0.0;

	{
		int i = 0;
		for (CVertex* first : visits.m_Vertices) {

			costType curr_min_dist = numeric_limits<costType>::max();
			CVertex* curr_min_vertex = nullptr;

			list<CVertex*>::iterator iter = visits.m_Vertices.begin();
			iter++;
			for (;iter!=visits.m_Vertices.end();++iter) {
				CVertex* const second = (*iter);
				const costType currDist = first->m_fasterDijkstraCosts[second->m_visitID];

				const bool cond = (currDist < curr_min_dist && first != second);
				curr_min_dist = (cond) ? (currDist) : (curr_min_dist);
				curr_min_vertex = (cond) ? (second) : (curr_min_vertex);
			}

			sum_min_vertexs_cost += curr_min_dist;

			heuristic3_min_costs[i].vertex = curr_min_vertex;
			heuristic3_min_costs[i].cost = curr_min_dist;

			i++;
		}
	}

	CVisits toVisit = visits;

	//add to it the first node and remove it from visits
	CVertex* const firstVertex = toVisit.m_Vertices.front();
	toVisit.m_Vertices.pop_front();

	//store the finalVertex here and remove it from visits
	CVertex* const lastVertex = toVisit.m_Vertices.back();
	toVisit.m_Vertices.pop_back();

	priority_queue<unique_ptr<PartialSolution3>, vector<unique_ptr<PartialSolution3>>, PartialSolutionComparison3> queue;

	//===============================================================================================================================================


	//push init solution
	{
		unique_ptr<PartialSolution3> aux(new PartialSolution3(toVisit.m_Vertices, firstVertex, sum_min_vertexs_cost, heuristic3_min_costs));
		aux->assignHeuristic();
		queue.push(std::move(aux));
	}


	const PartialSolution3 *curr = nullptr;
	//expand the queue
	while ((curr = queue.top().get()), !(curr->isCompleteSolution())) {

		if (curr->visitedTargets == (NUM_OF_VISITS - 1)) {		//if only the last remains
			unique_ptr<PartialSolution3> aux(new PartialSolution3(*curr, lastVertex, toVisit));
			aux->assignHeuristic();
			queue.pop();
			queue.push(std::move(aux));
		}
		else {

			vector<PartialSolution3*> auxList;
			auxList.reserve(toVisit.m_Vertices.size() - curr->visitedTargets + 1);

			for (CVertex* target : toVisit.m_Vertices) {
				if (curr->indexesInsidePath[target->m_visitID] == -1) {
					auxList.push_back(new PartialSolution3(*curr, target, toVisit));
				}
			}

			queue.pop();

			for (PartialSolution3 *child : auxList) {
				child->assignHeuristic();
				queue.push(std::move(unique_ptr<PartialSolution3>(child)));
			}

		}

	}

	//===============================================================================================================================================

	//fill ret with the whole path
	vector<CVertex*> bestCombination;
	bestCombination.resize(NUM_OF_VISITS, nullptr);

	for (CVertex* target : toVisit.m_Vertices) {
		bestCombination[curr->indexesInsidePath[target->m_visitID]] = target;
	}

	bestCombination[0] = firstVertex;
	bestCombination[bestCombination.size() - 1] = lastVertex;

	CTrack ret;

	ret.m_Vertices.push_back(firstVertex);
	for (int i = 0; i<((int)bestCombination.size() - 1); i++) {

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

	cout << curr->acc_cost << endl;

	return ret;
}
