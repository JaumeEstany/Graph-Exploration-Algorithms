// GraphDimensionsDlg.cpp: archivo de implementación
//

#include "stdafx.h"
#include "Salesman.h"
#include "GraphDimensionsDlg.h"
#include "afxdialogex.h"


// Cuadro de diálogo de CGraphDimensionsDlg

IMPLEMENT_DYNAMIC(CGraphDimensionsDlg, CDialogEx)

CGraphDimensionsDlg::CGraphDimensionsDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_DIALOG_GRAPH_DIM, pParent)
	, m_Vertices(0)
	, m_Aristas(0)
{

}

CGraphDimensionsDlg::~CGraphDimensionsDlg()
{
}

void CGraphDimensionsDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_EDIT_VERTICES, m_Vertices);
	DDV_MinMaxInt(pDX, m_Vertices, 1, 999999);
	DDX_Text(pDX, IDC_EDIT_ARISTAS, m_Aristas);
	DDV_MinMaxInt(pDX, m_Aristas, 1, 9999999);
}


BEGIN_MESSAGE_MAP(CGraphDimensionsDlg, CDialogEx)
END_MESSAGE_MAP()


// Controladores de mensajes de CGraphDimensionsDlg




BOOL CGraphDimensionsDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	m_Vertices = 10;
	m_Aristas = 20;
	UpdateData(false);

	return TRUE;  // return TRUE unless you set the focus to a control
				  // EXCEPCIÓN: las páginas de propiedades OCX deben devolver FALSE
}

void CGraphDimensionsDlg::OnOK()
{
	UpdateData(true);
	CDialogEx::OnOK();
}
