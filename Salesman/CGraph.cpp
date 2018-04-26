#include "stdafx.h"
#include "CGraph.h"


// =============================================================================
// CVertex ======================================================================
// =============================================================================

// NeighbordP ===================================================================

bool CVertex::NeighbordP(CVertex *pVertex)
{
	for (list<CVertex*>::const_iterator iter = m_Neighbords.cbegin(); iter != m_Neighbords.cend(); ++iter) {
		if (*iter == pVertex) return true;
	}
	return false;
}

// =============================================================================
// CGraph ======================================================================
// =============================================================================

// CGraph ======================================================================

CGraph::CGraph()
{
	
}


// ~CGraph ====================================================================

CGraph::~CGraph()
{
	m_Vertices.clear();
}

// Clear ======================================================================

void CGraph::Clear()
{
	m_Vertices.clear();
}


// FindVertex ==================================================================

CVertex* CGraph::FindVertex(double x, double y)
{
	for (CVertex &v : m_Vertices) {
		if (v.m_Point.m_X == x && v.m_Point.m_Y == y) return &v;
	}
	return NULL;
}

// GetVertex ===================================================================

CVertex* CGraph::GetVertex(double x, double y)
{
	for (CVertex &v : m_Vertices) {
		if (v.m_Point.m_X == x && v.m_Point.m_Y == y) return &v;
	}
	char msg[256];
	sprintf_s(msg,"CGraph::GetVertex: no se ha encontrado un vértice del grafo con coordenadas: %f,%f", x, y);
	throw exception(msg);
}


// GetVertex ===================================================================

CVertex* CGraph::GetVertex(int indice)
{
	int indice0 = indice;
	for (CVertex &v : m_Vertices) {
		if (indice ==0) return &v;
		--indice;
	}
	char msg[256];
	sprintf_s(msg, "CGraph::GetVertex: no se ha encontrado un vértice del grafo con indice: %d", indice0);
	throw exception(msg);
}


// MemberP =====================================================================

bool CGraph::MemberP(CVertex *pVertex)
{
	for (CVertex &v : m_Vertices) {
		if (&v==pVertex) return true;
	}
	return false;
}

// GetNVertex ==================================================================

int CGraph::GetNVertex()
{
	int n = 0;
	for (CVertex &_ : m_Vertices) ++n;
	return n;
}

// AddEdge =====================================================================

void CGraph::AddEdge(double x1, double y1, double x2, double y2)
{
	CVertex *pV1 = FindVertex(x1, y1);
	CVertex *pV2 = FindVertex(x2, y2);
	if (pV1 && pV2 && pV1->NeighbordP(pV2)) {
		char msg[256];
		sprintf_s(msg, "CGraph::AddEdge: edge duplicado (%f,%f)-(%f,%f)", x1, y1, x2, y2);
		throw exception(msg);
	}
	if (pV1 == NULL) {
		m_Vertices.push_back(CVertex(x1,y1));
		pV1 = &m_Vertices.back();
	}
	if (pV2 == NULL) {
		m_Vertices.push_back(CVertex(x2, y2));
		pV2 = &m_Vertices.back();
	}
	pV1->m_Neighbords.push_back(pV2);
	pV2->m_Neighbords.push_back(pV1);
}

// Write =======================================================================

void CGraph::Write(const char* filename)
{
	for (CVertex &v : m_Vertices) v.m_Saved = false;
	ofstream  f(filename);
	f << "GRAPH" << endl;
	for (CVertex &v1 : m_Vertices) {
		for (CVertex *pV2 : v1.m_Neighbords) {
			if (!pV2->m_Saved) {
				f << v1.m_Point.m_X << " " << v1.m_Point.m_Y << " " << pV2->m_Point.m_X << " " << pV2->m_Point.m_Y << endl;
			}
		}
		v1.m_Saved = true;
	}
	f.close();
}

// Read ========================================================================

void CGraph::Read(const char* filename)
{
	Clear();
	ifstream  f(filename);
	if (!f.good()) throw exception(CString("Error opening graph ") + filename);
	try {
		char buf[100];
		f.getline(buf,100);
		if (strcmp(buf,"GRAPH")!=0) throw exception(CString("CGraph::Read: el fichero no tiene formato de fichero de grafos: ") +filename);
		// leer vértices
		while (!f.eof()) {
			f.getline(buf, 100);
			if (buf[0] == '\0') break;
			double x1, y1, x2, y2;
			sscanf_s(buf, "%lf %lf %lf %lf", &x1, &y1, &x2, &y2);
			//cout << "(" << x1 << "," << y1 << ")-(" << x2 << "," << y2 << ")" << endl;
			AddEdge(x1, y1, x2, y2);
		}
		f.close();
	}
	catch (exception) {
		f.close();
		Clear();
		throw;
	}
}

// WriteDistances ==============================================================

void CGraph::WriteDistances(const char* filename)
{
	ofstream  f(filename);
	if (!f.good()) throw exception(CString("Error opening distances ") + filename);
	f << "DISTANCES" << endl;
	for (CVertex &v1 : m_Vertices) {
		f << fixed << setprecision(2) << v1.m_DijkstraDistance << endl;
	}
	f.close();
}

// ReadDistances ===============================================================

void CGraph::ReadDistances(const char* filename)
{
	ifstream  f(filename);
	if (!f.good()) throw exception(CString("Error opening distances ") + filename);
	try {
		char buf[100];
		f.getline(buf, 100);
		if (strcmp(buf, "DISTANCES") != 0) throw exception(CString("CGraph::ReadDistances: el fichero no tiene formato de fichero de distancias: ")+filename);
		// leer vértices
		for (CVertex &v1 : m_Vertices) {
			f.getline(buf, 100);
			if (buf[0] == '\0') throw exception(CString("CGraph::ReadDistances: error de lectura de distancias: ") + filename);
			sscanf_s(buf, "%lf", &v1.m_DijkstraDistance);
		}
	}
	catch (exception) {
		f.close();
		Clear();
		throw;
	}
}

// CrearAleatorio ==============================================================

void CGraph::CrearAleatorio(int nVertices, int nEdges)
{
	Clear();
	srand((unsigned)time(NULL));
	vector<CGPoint> vertices(nVertices);
	for (int i = 0; i<nVertices; ++i) {
		double x, y;
		bool encontrado;
		double minDist = 16000.0 * 16000.0*0.2*0.2;
		do {
			x = rand() % 16000; 
			y = rand() % 16000;
			encontrado = false;
			for (int j = 0; j<i; ++j) {
				double dx = vertices[j].m_X - x;
				double dy = vertices[j].m_Y - y;
				if (dx*dx + dy*dy<minDist) {
					encontrado = true;
					break;
				}
			}
			minDist *= 0.75;
		} while (encontrado);
		vertices[i] = CGPoint(x, y);
		if (i != 0) {
			int j = rand() % i;
			AddEdge(vertices[j].m_X, vertices[j].m_Y, x, y);
			--nEdges;
		}
	}
	int rep = 0;
	while (nEdges>0) {
		int i = rand() % nVertices;
		int j = rand() % nVertices;
		if (i != j && !GetVertex(vertices[i])->NeighbordP(GetVertex(vertices[j]))) {
			AddEdge(vertices[i].m_X, vertices[i].m_Y, vertices[j].m_X, vertices[j].m_Y);
			--nEdges;
			rep = 0;
		}
		else {
			++rep;
			if (rep > 100) throw exception("CGraph::CrearAleatorio: No se puede encontrar un nuevo edge");
		}
	}
}


// =============================================================================
// CVisits =====================================================================
// =============================================================================

// MemberP =====================================================================

bool CVisits::MemberP(CVertex *pVertex)
{
	for (CVertex* pV : m_Vertices) 	if (pV== pVertex) return true;
	return false;
}

// Write =======================================================================

void CVisits::Write(const char* filename) {
	ofstream  f(filename);
	if (!f.good()) throw exception(CString("Error opening visits ") + filename);
	f << "VISITS" << endl;
	for (CVertex* pV : m_Vertices) {
		f << pV->m_Point.m_X << " " << pV->m_Point.m_Y << endl;
	}
	f.close();
}

// Read ========================================================================

void CVisits::Read(const char* filename) {
	Clear();
	ifstream  f(filename);
	if (!f.good()) throw exception(CString("Error opening visits ") + filename);
	try {
		char buf[100];
		f.getline(buf, 100);
		if (strcmp(buf, "VISITS") != 0) throw exception(CString("CVisits::Read: el fichero no tiene formato de fichero de visits: ")+filename);
		// leer vértices
		while (!f.eof()) {
			f.getline(buf, 100);
			if (buf[0] == '\0') break;
			double x, y;
			sscanf_s(buf, "%lf %lf", &x, &y);
			Add(m_pGraph->GetVertex(x,y));
		}
		f.close();
	}
	catch (exception) {
		f.close();
		Clear();
		throw;
	}
}

// CrearAleatorio ==============================================================

void CVisits::CrearAleatorio(int nVisits, bool ciclo)
{
	Clear();
	srand((unsigned)time(NULL));
	int n = m_pGraph->GetNVertex();
	if (n==0) throw exception("CVisits::CrearAleatorio no puede crear visitas si no hay grafo");

	for (int i = 0; i < nVisits; ++i) {
		CVertex *pVertex;
		int j = 0;
		do {
			int index = rand() % n;
			pVertex = m_pGraph->GetVertex(index);
			++j;
			if (j > 1000) throw exception("CVisits::CrearAleatorio no encuentra un vértice diferente a los ya añadidos");
		} while (MemberP(pVertex));
		Add(pVertex);
	}
	if (ciclo) Add(m_Vertices.front());
}



// =============================================================================
// CTrack ======================================================================
// =============================================================================

// Append ======================================================================

void CTrack::Append(CTrack &t) 
{
	assert(m_pGraph == t.m_pGraph);
	for (list<CVertex*>::const_iterator iter = t.m_Vertices.cbegin(); iter != t.m_Vertices.cend(); ++iter) {
		m_Vertices.push_back(*iter);
	}
}

// AppendBefore ================================================================

void CTrack::AppendBefore(CTrack &t) 
{
	assert(m_pGraph == t.m_pGraph);
	for (list<CVertex*>::const_reverse_iterator iter = t.m_Vertices.crbegin(); iter != t.m_Vertices.crend(); ++iter) {
		m_Vertices.push_front(*iter);
	}
}

// Write =======================================================================

void CTrack::Write(const char* filename)
{
	ofstream  f(filename);
	if (!f.good()) throw exception(CString("Error opening track ") + filename);
	f << "TRACK" << endl;
	for (list<CVertex*>::const_iterator iter = m_Vertices.cbegin(); iter != m_Vertices.cend(); ++iter) {
		const CGPoint p1 = (*iter)->m_Point;
		f << p1.m_X << " " << p1.m_Y << endl;
	}
	f.close();
}

// Read ========================================================================

void CTrack::Read(const char* filename)
{
	Clear();
	ifstream  f(filename);
	if (!f.good()) throw exception(CString("Error opening track ") + filename);
	try {
		char buf[100];
		f.getline(buf, 100);
		if (strcmp(buf, "TRACK") != 0) throw exception(CString("CTrack::Read: el fichero no tiene formato de fichero de track: ")+filename);
		// leer vértices
		while (!f.eof()) {
			f.getline(buf, 100);
			if (buf[0] == '\0') break;
			double x, y;
			sscanf_s(buf, "%lf %lf", &x, &y);
			AddLast(x, y);
		}
		f.close();
	}
	catch (exception) {
		f.close();
		Clear();
		throw;
	}
}

// Length ======================================================================

double CTrack::Length() {
	double l = 0.0;
	for (list<CVertex*>::const_iterator iter = m_Vertices.cbegin(); iter != m_Vertices.cend();) {
		const CGPoint p1 = (*iter)->m_Point;
		++iter;
		if (iter != m_Vertices.cend()) {
			const CGPoint p2 = (*iter)->m_Point;
			l += (p2 - p1).Module();
		}
		else break;
	}
	return l;
}
