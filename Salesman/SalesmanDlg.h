
// SalesmanDlg.h: archivo de encabezado
//

#pragma once
#include "CGraphCtrl.h"


// Cuadro de diálogo de CSalesmanDlg
class CSalesmanDlg : public CDialogEx
{
// Construcción
public:
	CSalesmanDlg(CWnd* pParent = NULL);	// Constructor estándar

// Datos del cuadro de diálogo
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_SALESMAN_DIALOG };
#endif
	CGraphCtrl m_GraphCtrl;

	CGraph m_Graph;
	CVisits m_Visits;
	CTrack m_Track;

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// Compatibilidad con DDX/DDV


// Implementación
protected:
	HICON m_hIcon;

	// Funciones de asignación de mensajes generadas
	virtual BOOL OnInitDialog();
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedButtonLeerGrafo();
	afx_msg void OnBnClickedButtonGuardarGrafo();
	afx_msg void OnBnClickedButtonGuardarVisitas();
	afx_msg void OnBnClickedButtonLeerVisitas();
	afx_msg void OnBnClickedButtonLeerTrack();
	afx_msg void OnBnClickedButtonGuardarTrack();
	afx_msg void OnBnClickedButtonCrearGrafoAleatorio();
	afx_msg void OnBnClickedButtonCrearVisitasAleatorias();
	afx_msg void OnBnClickedButtonDijkstra();
	afx_msg void OnBnClickedButtonDijkstraQueue();
	afx_msg void OnBnClickedButtonSalesmanGreedy();
	afx_msg void OnBnClickedButtonSalesmanBacktrackingPuro();
	afx_msg void OnBnClickedButtonSalesmanBacktrackingGreedy();
	afx_msg void OnBnClickedButtonSalesmanBranchAndBound1();
	afx_msg void OnBnClickedButtonSalesmanBranchAndBound2();
	afx_msg void OnBnClickedButtonSalesmanBranchAndBound3();
	afx_msg void OnBnClickedButtonLeerDistancias();
	afx_msg void OnBnClickedButtonGuardarDistancias();
};
