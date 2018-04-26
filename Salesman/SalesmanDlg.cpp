
// SalesmanDlg.cpp: archivo de implementación
//

#include "stdafx.h"
#include "Salesman.h"
#include "SalesmanDlg.h"
#include "afxdialogex.h"
#include "GraphDimensionsDlg.h"
#include "VisitsDimensionDlg.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// Cuadro de diálogo de CSalesmanDlg



CSalesmanDlg::CSalesmanDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_SALESMAN_DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CSalesmanDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_GRAPH, m_GraphCtrl);
}

BEGIN_MESSAGE_MAP(CSalesmanDlg, CDialogEx)
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BUTTON_LEER_GRAFO, &CSalesmanDlg::OnBnClickedButtonLeerGrafo)
	ON_BN_CLICKED(IDC_BUTTON_GUARDAR_GRAFO, &CSalesmanDlg::OnBnClickedButtonGuardarGrafo)
	ON_BN_CLICKED(IDC_BUTTON_GUARDAR_VISITAS, &CSalesmanDlg::OnBnClickedButtonGuardarVisitas)
	ON_BN_CLICKED(IDC_BUTTON_LEER_VISITAS, &CSalesmanDlg::OnBnClickedButtonLeerVisitas)
	ON_BN_CLICKED(IDC_BUTTON_LEER_TRACK, &CSalesmanDlg::OnBnClickedButtonLeerTrack)
	ON_BN_CLICKED(IDC_BUTTON_GUARDAR_TRACK, &CSalesmanDlg::OnBnClickedButtonGuardarTrack)
	ON_BN_CLICKED(IDC_BUTTON_CREAR_GRAFO_ALEATORIO, &CSalesmanDlg::OnBnClickedButtonCrearGrafoAleatorio)
	ON_BN_CLICKED(IDC_BUTTON_CREAR_VISITAS_ALEATORIAS, &CSalesmanDlg::OnBnClickedButtonCrearVisitasAleatorias)
	ON_BN_CLICKED(IDC_BUTTON_DIJKSTRA, &CSalesmanDlg::OnBnClickedButtonDijkstra)
	ON_BN_CLICKED(IDC_BUTTON_DIJKSTRA_QUEUE, &CSalesmanDlg::OnBnClickedButtonDijkstraQueue)
	ON_BN_CLICKED(IDC_BUTTON_SALESMAN_GREEDY, &CSalesmanDlg::OnBnClickedButtonSalesmanGreedy)
	ON_BN_CLICKED(IDC_BUTTON_SALESMAN_BACKTRACKING_PURO, &CSalesmanDlg::OnBnClickedButtonSalesmanBacktrackingPuro)
	ON_BN_CLICKED(IDC_BUTTON_SALESMAN_BACKTRACKING_GREEDY, &CSalesmanDlg::OnBnClickedButtonSalesmanBacktrackingGreedy)
	ON_BN_CLICKED(IDC_BUTTON_SALESMAN_BRANCH_AND_BOUND_1, &CSalesmanDlg::OnBnClickedButtonSalesmanBranchAndBound1)
	ON_BN_CLICKED(IDC_BUTTON_SALESMAN_BRANCH_AND_BOUND_2, &CSalesmanDlg::OnBnClickedButtonSalesmanBranchAndBound2)
	ON_BN_CLICKED(IDC_BUTTON_SALESMAN_BRANCH_AND_BOUND_3, &CSalesmanDlg::OnBnClickedButtonSalesmanBranchAndBound3)
	ON_BN_CLICKED(IDC_BUTTON_LEER_DISTANCIAS, &CSalesmanDlg::OnBnClickedButtonLeerDistancias)
	ON_BN_CLICKED(IDC_BUTTON_GUARDAR_DISTANCIAS, &CSalesmanDlg::OnBnClickedButtonGuardarDistancias)
END_MESSAGE_MAP()


// Controladores de mensaje de CSalesmanDlg

BOOL CSalesmanDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// Establecer el icono para este cuadro de diálogo.  El marco de trabajo realiza esta operación
	//  automáticamente cuando la ventana principal de la aplicación no es un cuadro de diálogo
	SetIcon(m_hIcon, TRUE);			// Establecer icono grande
	SetIcon(m_hIcon, FALSE);		// Establecer icono pequeño

	m_GraphCtrl.m_pGraph = &m_Graph;
	m_GraphCtrl.m_pVisits = &m_Visits;
	m_GraphCtrl.m_pTrack = &m_Track;
	m_Visits.SetGraph(&m_Graph);
	m_Track.SetGraph(&m_Graph);


	return TRUE;  // Devuelve TRUE  a menos que establezca el foco en un control
}

// Si agrega un botón Minimizar al cuadro de diálogo, necesitará el siguiente código
//  para dibujar el icono.  Para aplicaciones MFC que utilicen el modelo de documentos y vistas,
//  esta operación la realiza automáticamente el marco de trabajo.

void CSalesmanDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // Contexto de dispositivo para dibujo

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Centrar icono en el rectángulo de cliente
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Dibujar el icono
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// El sistema llama a esta función para obtener el cursor que se muestra mientras el usuario arrastra
//  la ventana minimizada.
HCURSOR CSalesmanDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


// OnBnClickedButtonLeerGrafo ==================================================

void CSalesmanDlg::OnBnClickedButtonLeerGrafo()
{
	CFileDialog dlg(true, NULL, NULL, OFN_FILEMUSTEXIST,
		"Fichero de grafo (*.txt)|*.txt|"
		"All Files (*.*)|*.*||"
	);

	if (dlg.DoModal() == IDOK) {
		try {
			m_Visits.Clear();
			m_Track.Clear();
			m_Graph.Read(dlg.GetPathName());
			cout << "Grafo: " << dlg.GetPathName() << endl;
			m_GraphCtrl.m_DrawDijkstraDistances = false;
		}
		catch (exception &ex) {
			AfxMessageBox(CString(ex.what()), MB_OK | MB_ICONERROR);
		}
		m_GraphCtrl.Invalidate();
	}
}

// OnBnClickedButtonCrearGrafoAleatorio ========================================

void CSalesmanDlg::OnBnClickedButtonCrearGrafoAleatorio()
{
	CGraphDimensionsDlg dlg;
	if (dlg.DoModal() == IDOK) {
		try {
			m_Graph.CrearAleatorio(dlg.m_Vertices, dlg.m_Aristas);
			cout << "Grafo aleatorio: " << dlg.m_Vertices << "vertices " << dlg.m_Aristas << "aristas" << endl;
			m_GraphCtrl.m_DrawDijkstraDistances = false;
		}
		catch (exception &ex) {
			AfxMessageBox(CString(ex.what()), MB_OK | MB_ICONERROR);
		}
		m_GraphCtrl.Invalidate();
	}
}

// OnBnClickedButtonGuardarGrafo ===============================================

void CSalesmanDlg::OnBnClickedButtonGuardarGrafo()
{
	CFileDialog dlg(false, ".txt", NULL, OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
		"Fichero de grafo (*.txt)|*.txt|"
		"All Files (*.*)|*.*||"
	);

	if (dlg.DoModal() == IDOK) {
		try {
			m_Graph.Write(dlg.GetPathName());
		}
		catch (exception &ex) {
			AfxMessageBox(CString(ex.what()), MB_OK | MB_ICONERROR);
		}
	}
}

// OnBnClickedButtonLeerDistancias =============================================

void CSalesmanDlg::OnBnClickedButtonLeerDistancias()
{
	if (m_Graph.GetNVertex() == 0) {
		AfxMessageBox("Se necesita un grafo para leer distancias", MB_OK | MB_ICONERROR);
		return;
	}

	CFileDialog dlg(true, NULL, NULL, OFN_FILEMUSTEXIST,
		"Fichero de distancias (*.txt)|*.txt|"
		"All Files (*.*)|*.*||"
	);

	if (dlg.DoModal() == IDOK) {
		try {
			m_Graph.ReadDistances(dlg.GetPathName());
			cout << "Distancias: " << dlg.GetPathName() << endl;
			m_GraphCtrl.m_DrawDijkstraDistances = true;
		}
		catch (exception &ex) {
			AfxMessageBox(CString(ex.what()), MB_OK | MB_ICONERROR);
		}
		m_GraphCtrl.Invalidate();
	}
}

// OnBnClickedButtonGuardarDistancias ==========================================

void CSalesmanDlg::OnBnClickedButtonGuardarDistancias()
{
	if (m_Graph.GetNVertex() == 0) {
		AfxMessageBox("Se necesita un grafo para guardar distancias", MB_OK | MB_ICONERROR);
		return;
	}
	bool distanciasCalculadas = false;
	for (CVertex &v : m_Graph.m_Vertices) {
		if (v.m_DijkstraDistance > 0) {
			distanciasCalculadas = true;
			break;
		}
	}
	if (m_Graph.m_Vertices.front().m_DijkstraDistance != 0.0) distanciasCalculadas = false;
	if (!distanciasCalculadas) {
		AfxMessageBox("Las distancias del grafo no están calculadas", MB_OK | MB_ICONERROR);
		return;
	}

	CFileDialog dlg(false, ".txt", NULL, OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
		"Fichero de distancias(*.txt)|*.txt|"
		"All Files (*.*)|*.*||"
	);

	if (dlg.DoModal() == IDOK) {
		try {
			m_Graph.WriteDistances(dlg.GetPathName());
		}
		catch (exception &ex) {
			AfxMessageBox(CString(ex.what()), MB_OK | MB_ICONERROR);
		}
	}
}


// OnBnClickedButtonLeerVisitas ================================================

void CSalesmanDlg::OnBnClickedButtonLeerVisitas()
{
	CFileDialog dlg(true, NULL, NULL, OFN_FILEMUSTEXIST,
		"Fichero de visitas (*.txt)|*.txt|"
		"All Files (*.*)|*.*||"
	);

	if (dlg.DoModal() == IDOK) {
		try {
			m_Visits.Read(dlg.GetPathName());
			cout << "Visitas: " << dlg.GetPathName() << endl;
			m_GraphCtrl.m_pVisits = &m_Visits;
		}
		catch (exception &ex) {
			AfxMessageBox(CString(ex.what()), MB_OK | MB_ICONERROR);
		}
		m_GraphCtrl.Invalidate();
	}
}

// OnBnClickedButtonCrearVisitasAleatorias =====================================

void CSalesmanDlg::OnBnClickedButtonCrearVisitasAleatorias()
{
	CVisitsDimensionDlg dlg;
	if (dlg.DoModal() == IDOK) {
		try {
			m_Visits.CrearAleatorio(dlg.m_Visitas, dlg.m_Ciclo);
			cout << "Vistas aleatorias: " << dlg.m_Visitas << "visitas " << (dlg.m_Ciclo? "ciclicas" : "") << endl;
		}
		catch (exception &ex) {
			AfxMessageBox(CString(ex.what()), MB_OK | MB_ICONERROR);
		}
		m_GraphCtrl.Invalidate();
	}
}

// OnBnClickedButtonGuardarVisitas =============================================

void CSalesmanDlg::OnBnClickedButtonGuardarVisitas()
{
	CFileDialog dlg(false, ".txt", NULL, OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
		"Fichero de visitas (*.txt)|*.txt|"
		"All Files (*.*)|*.*||"
	);

	if (dlg.DoModal() == IDOK) {
		try {
			m_Visits.Write(dlg.GetPathName());
		}
		catch (exception &ex) {
			AfxMessageBox(CString(ex.what()), MB_OK | MB_ICONERROR);
		}
	}
}


// OnBnClickedButtonLeerTrack ==================================================

void CSalesmanDlg::OnBnClickedButtonLeerTrack()
{
	CFileDialog dlg(true, NULL, NULL, OFN_FILEMUSTEXIST,
		"Fichero de track (*.txt)|*.txt|"
		"All Files (*.*)|*.*||"
	);

	if (dlg.DoModal() == IDOK) {
		try {
			m_Track.Read(dlg.GetPathName());
			cout << "Track: " << dlg.GetPathName() << endl;
		}
		catch (exception &ex) {
			AfxMessageBox(CString(ex.what()), MB_OK | MB_ICONERROR);
		}
		m_GraphCtrl.Invalidate();
	}
}


// OnBnClickedButtonGuardarTrack ===============================================

void CSalesmanDlg::OnBnClickedButtonGuardarTrack()
{
	CFileDialog dlg(false, ".txt", NULL, OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
		"Fichero de track (*.txt)|*.txt|"
		"All Files (*.*)|*.*||"
	);

	if (dlg.DoModal() == IDOK) {
		try {
			m_Track.Write(dlg.GetPathName());
		}
		catch (exception &ex) {
			AfxMessageBox(CString(ex.what()), MB_OK | MB_ICONERROR);
		}
	}
}

// OnBnClickedButtonDijkstra ===================================================

void CSalesmanDlg::OnBnClickedButtonDijkstra()
{
	if (m_Graph.GetNVertex() == 0) {
		AfxMessageBox("El algoritmo Dijkstra necesita un grafo para ejecutarse", MB_OK | MB_ICONERROR);
		return;
	}
	double t0 = Clock();
	m_Graph.Dijkstra(&m_Graph.m_Vertices.front());
	double t1 = Clock();
	cout << "Dijkstra. Tiempo: " << t1-t0 << "seg." << endl;
	m_GraphCtrl.m_DrawDijkstraDistances = true;
	m_GraphCtrl.Invalidate();
}

// OnBnClickedButtonDijkstraQueue ===============================================

void CSalesmanDlg::OnBnClickedButtonDijkstraQueue()
{
	if (m_Graph.GetNVertex() == 0) {
		AfxMessageBox("El algoritmo DijkstraHeap necesita un grafo para ejecutarse", MB_OK | MB_ICONERROR);
		return;
	}
	double t0 = Clock();
	m_Graph.DijkstraQueue(&m_Graph.m_Vertices.front());
	double t1 = Clock();
	cout << "DijkstraQueue. Tiempo: " << t1 - t0 << "seg." << endl;
	m_GraphCtrl.m_DrawDijkstraDistances = true;
	m_GraphCtrl.Invalidate();
}

// OnBnClickedButtonSalesmanGreedy =============================================

void CSalesmanDlg::OnBnClickedButtonSalesmanGreedy()
{
	if (m_Graph.GetNVertex() == 0) {
		AfxMessageBox("El algoritmo SalesmanGreedy necesita un grafo para ejecutarse", MB_OK | MB_ICONERROR);
		return;
	}
	if (m_Visits.m_Vertices.empty()) {
		AfxMessageBox("El algoritmo SalesmanGreedy necesita unas visitias para ejecutarse", MB_OK | MB_ICONERROR);
		return;
	}
	double t0 = Clock();
	m_Track=m_Graph.SalesmanTrackGreedy(m_Visits);
	double t1 = Clock();
	cout << "SalesmanTrackGreedy. Tiempo: " << t1 - t0 << "seg." << endl;
	m_GraphCtrl.m_DrawDijkstraDistances = false;
	m_GraphCtrl.Invalidate();
}

// OnBnClickedButtonSalesmanBacktrackingPuro ===================================

void CSalesmanDlg::OnBnClickedButtonSalesmanBacktrackingPuro()
{
	if (m_Graph.GetNVertex() == 0) {
		AfxMessageBox("El algoritmo Backtracking necesita un grafo para ejecutarse", MB_OK | MB_ICONERROR);
		return;
	}
	if (m_Visits.m_Vertices.empty()) {
		AfxMessageBox("El algoritmo Backtracking necesita unas visitias para ejecutarse", MB_OK | MB_ICONERROR);
		return;
	}
	double t0 = Clock();
	m_Track = m_Graph.SalesmanTrackBacktracking(m_Visits);
	double t1 = Clock();
	cout << "SalesmanTrackBacktracking. Tiempo: " << t1 - t0 << "seg." << endl;
	m_GraphCtrl.m_DrawDijkstraDistances = false;
	m_GraphCtrl.Invalidate();
}

// OnBnClickedButtonSalesmanBacktrackingGreedy =================================

void CSalesmanDlg::OnBnClickedButtonSalesmanBacktrackingGreedy()
{
	if (m_Graph.GetNVertex() == 0) {
		AfxMessageBox("El algoritmo BacktrackingGreedy necesita un grafo para ejecutarse", MB_OK | MB_ICONERROR);
		return;
	}
	if (m_Visits.m_Vertices.empty()) {
		AfxMessageBox("El algoritmo BacktrackingGreedy necesita unas visitias para ejecutarse", MB_OK | MB_ICONERROR);
		return;
	}
	double t0 = Clock();
	m_Track = m_Graph.SalesmanTrackBacktrackingGreedy(m_Visits);
	double t1 = Clock();
	cout << "SalesmanTrackBacktrackingGreedy. Tiempo: " << t1 - t0 << "seg." << endl;
	m_GraphCtrl.m_DrawDijkstraDistances = false;
	m_GraphCtrl.Invalidate();
}

// OnBnClickedButtonSalesmanBranchAndBound1 ====================================

void CSalesmanDlg::OnBnClickedButtonSalesmanBranchAndBound1()
{
	if (m_Graph.GetNVertex() == 0) {
		AfxMessageBox("El algoritmo BranchAndBound1 necesita un grafo para ejecutarse", MB_OK | MB_ICONERROR);
		return;
	}
	if (m_Visits.m_Vertices.empty()) {
		AfxMessageBox("El algoritmo BranchAndBound1 necesita unas visitias para ejecutarse", MB_OK | MB_ICONERROR);
		return;
	}
	double t0 = Clock();
	m_Track = m_Graph.SalesmanTrackBranchAndBound1(m_Visits);
	double t1 = Clock();
	cout << "SalesmanTrackBranchAndBound1. Tiempo: " << t1 - t0 << "seg." << endl;
	m_GraphCtrl.m_DrawDijkstraDistances = false;
	m_GraphCtrl.Invalidate();
}

// OnBnClickedButtonSalesmanBranchAndBound2 ====================================

void CSalesmanDlg::OnBnClickedButtonSalesmanBranchAndBound2()
{
	if (m_Graph.GetNVertex() == 0) {
		AfxMessageBox("El algoritmo BranchAndBound2 necesita un grafo para ejecutarse", MB_OK | MB_ICONERROR);
		return;
	}
	if (m_Visits.m_Vertices.empty()) {
		AfxMessageBox("El algoritmo BranchAndBound2 necesita unas visitias para ejecutarse", MB_OK | MB_ICONERROR);
		return;
	}
	double t0 = Clock();
	m_Track = m_Graph.SalesmanTrackBranchAndBound2(m_Visits);
	double t1 = Clock();
	cout << "SalesmanTrackBranchAndBound2. Tiempo: " << t1 - t0 << "seg." << endl;
	m_GraphCtrl.m_DrawDijkstraDistances = false;
	m_GraphCtrl.Invalidate();
}

// OnBnClickedButtonSalesmanBranchAndBound3 ====================================

void CSalesmanDlg::OnBnClickedButtonSalesmanBranchAndBound3()
{
	if (m_Graph.GetNVertex() == 0) {
		AfxMessageBox("El algoritmo BranchAndBound3 necesita un grafo para ejecutarse", MB_OK | MB_ICONERROR);
		return;
	}
	if (m_Visits.m_Vertices.empty()) {
		AfxMessageBox("El algoritmo BranchAndBound3 necesita unas visitias para ejecutarse", MB_OK | MB_ICONERROR);
		return;
	}
	double t0 = Clock();
	m_Track = m_Graph.SalesmanTrackBranchAndBound3(m_Visits);
	double t1 = Clock();
	cout << "SalesmanTrackBranchAndBound3. Tiempo: " << t1 - t0 << "seg." << endl;
	m_GraphCtrl.m_DrawDijkstraDistances = false;
	m_GraphCtrl.Invalidate();
}


