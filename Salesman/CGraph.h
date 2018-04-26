#pragma once



// =============================================================================
// CGPoint =====================================================================
// =============================================================================

class CGPoint
{

public:
	double m_X, m_Y;

	CGPoint() {}
	CGPoint(const double x, const double y = 0) : m_X(x), m_Y(y) {}
	CGPoint(const CGPoint&p) : m_X(p.m_X), m_Y(p.m_Y) {}
	CGPoint(const CPoint &p) : m_X(p.x), m_Y(p.y) {}
	operator CPoint() { return CPoint(int(m_X), int(m_Y)); }
	CGPoint& operator=(const CGPoint &p) {
		m_X = p.m_X;
		m_Y = p.m_Y;
		return *this;
	}
	CGPoint operator+(const CGPoint &p2)  const {
		return CGPoint(m_X + p2.m_X, m_Y + p2.m_Y);
	}
	CGPoint operator+=(const CGPoint &p2) {
		m_X += p2.m_X;
		m_Y += p2.m_Y;
		return *this;
	}
	CGPoint operator-(const CGPoint &p2)  const {
		return CGPoint(m_X - p2.m_X, m_Y - p2.m_Y);
	}
	CGPoint operator-=(const CGPoint &p2) {
		m_X -= p2.m_X;
		m_Y -= p2.m_Y;
		return *this;
	}
	CGPoint operator*=(const double n) {
		m_X *= n;
		m_Y *= n;
		return *this;
	}
	CGPoint operator/=(const double n) {
		m_X /= n;
		m_Y /= n;
		return *this;
	}
	CGPoint operator-()  const {
		return CGPoint(-m_X, -m_Y);
	}
	double operator*(const CGPoint &p2)  const {
		return m_X*p2.m_X + m_Y*p2.m_Y;
	}
	bool operator==(const CGPoint& p)  const {
		return m_X == p.m_X && m_Y == p.m_Y;
	}
	bool operator!=(const CGPoint& p)  const {
		return m_X != p.m_X || m_Y != p.m_Y;
	}
	double Module() { return sqrt(m_X*m_X + m_Y*m_Y); }
};

inline CGPoint operator*(double v, const CGPoint& p) {
	return CGPoint(v*p.m_X, v*p.m_Y);
}

inline CGPoint operator*(const CGPoint& p, double v) {
	return CGPoint(v*p.m_X, v*p.m_Y);
}

inline CGPoint operator/(const CGPoint& p, double v) {
	return CGPoint(p.m_X / v, p.m_Y / v);
}

inline ostream& operator<< (ostream& s, const CGPoint& p) {
	s << "(" << p.m_X << ", " << p.m_Y << ")";
	return s;
}

inline CGPoint Min(const CGPoint& p1, const CGPoint& p2) {
	return CGPoint(min(p1.m_X, p2.m_X), min(p1.m_Y, p2.m_Y));
}

inline CGPoint Max(const CGPoint& p1, const CGPoint& p2) {
	return CGPoint(max(p1.m_X, p2.m_X), max(p1.m_Y, p2.m_Y));
}


// =============================================================================
// CVertex =====================================================================
// =============================================================================

class CVertex {
public:
	CGPoint m_Point;
	list<CVertex*> m_Neighbords;
	bool m_Saved;
	// Campos para el algoritmo de Dijkstra
	double m_DijkstraDistance;
	bool m_DijkstraVisit;
	CVertex *m_pDijkstraPrevious;

	// Campos para el algoritmo de backtracking
	bool m_VertexToVisit; // Vertice a visitar (en lista visits)
	double m_toLastVertexCost;
	int m_visitID;
	vector<double> m_fasterDijkstraCosts;
public:
	CVertex(double x, double y) : m_Point(x, y), m_DijkstraDistance(0.0) {}
	CVertex() : m_DijkstraDistance(0.0) {}
	bool NeighbordP(CVertex *pVertex);
};

// =============================================================================
// CGraph ======================================================================
// =============================================================================

class CTrack;
class CVisits;

class CGraph
{
public:
	list<CVertex> m_Vertices; 
public:
	CGraph();
	~CGraph();
	void Clear();
	CVertex* FindVertex(double x, double y);
	CVertex* FindVertex(const CGPoint &p) { return FindVertex(p.m_X, p.m_Y); }
	CVertex* GetVertex(double x, double y);
	CVertex* GetVertex(const CGPoint &p) { return GetVertex(p.m_X, p.m_Y);  }
	CVertex* GetVertex(int indice);
	int GetNVertex();
	bool MemberP(CVertex *pVertex);
	void AddEdge(double x1, double y1, double x2, double y2);
	void Write(const char* filename);
	void Read(const char* filename);
	void WriteDistances(const char* filename);
	void ReadDistances(const char* filename);
	void CrearAleatorio(int nVertices, int nEdges);

	void Dijkstra(CVertex *pStart);
	void DijkstraQueue(CVertex *pStart);

	CTrack ShortestTrack(CVertex *pStart, CVertex *pEnd);
	CTrack SalesmanTrackGreedy(CVisits &visits);

	CTrack SalesmanTrackBacktracking(CVisits &visits);
	CTrack SalesmanTrackBacktrackingGreedy(CVisits &visits);

	CTrack SalesmanTrackBranchAndBound1(CVisits &visits);
	CTrack SalesmanTrackBranchAndBound2(CVisits &visits);
	CTrack SalesmanTrackBranchAndBound3(CVisits &visits);
};

// =============================================================================
// CVisits =====================================================================
// =============================================================================


class CVisits {
public:
	list<CVertex*> m_Vertices;
	CGraph *m_pGraph;
	CVisits() {
		m_pGraph = NULL;
	}
	CVisits(CGraph *pGraph) {
		m_pGraph = pGraph;
	}
	void SetGraph(CGraph *pGraph) {
		Clear();
		m_pGraph = pGraph;

	}

	bool MemberP(CVertex *pVertex);
	void Add(CVertex *pVertex) { m_Vertices.push_back(pVertex); }
	void Clear() { m_Vertices.clear(); }
	void Write(const char* filename);
	void Read(const char* filename);
	void CrearAleatorio(int nVisits, bool ciclo);

	/*
	// Print -------------------------------------------------------------------
	public String toString() {
		String str = "[";
		for (Iterator<CPoint> iter = m_Points.iterator(); iter.hasNext();) {
			CPoint p = iter.next();
			str = str + "(" + p.m_X + "," + p.m_Y + ")";
			if (iter.hasNext()) str = str + ",";
		}
		return str + "]";
	}
	*/
};


class CTrack {
public:
	list<CVertex*> m_Vertices;
	CGraph *m_pGraph;
	CTrack() {
		m_pGraph = NULL;
	}
	CTrack(CGraph *pGraph) {
		m_pGraph = pGraph;
	}
	void SetGraph(CGraph *pGraph) {
		Clear();
		m_pGraph = pGraph;

	}
	void AddLast(double x, double y) {
		CVertex *pVertex = m_pGraph->GetVertex(x, y);
		m_Vertices.push_back(pVertex);
	}
	void AddLast(CGPoint p)  {
		CVertex *pVertex = m_pGraph->GetVertex(p);
		m_Vertices.push_back(pVertex);
	}
	void AddLast(CVertex *pVertex) {
		assert(m_pGraph->MemberP(pVertex));
		m_Vertices.push_back(pVertex);
	}
	void AddFirst(CVertex *pVertex) {
		assert(m_pGraph->MemberP(pVertex));
		m_Vertices.push_front(pVertex);
	}
	void Clear() {
		m_Vertices.clear();
	}
	void Append(CTrack &t);
	void AppendBefore(CTrack &t);

	// Files ----------------------------------------------------------------	
	void Write(const char* filename);
	void Read(const char* filename);

	// Print -------------------------------------------------------------------
	/*
	public String toString() {
		String str = "[";
		for (Iterator<CVertex> iter = m_Vertices.iterator(); iter.hasNext();) {
			CVertex v = iter.next();
			CPoint p = v.m_Point;
			str = str + "(" + p.m_X + "," + p.m_Y + ")";
			if (iter.hasNext()) str = str + ",";
		}
		return str + "]";
	}
	*/
	// Length -------------------------------------------------------------------
	double Length();
};
