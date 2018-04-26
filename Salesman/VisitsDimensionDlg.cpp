// VisitsDimensionDlg.cpp: archivo de implementación
//

#include "stdafx.h"
#include "Salesman.h"
#include "VisitsDimensionDlg.h"
#include "afxdialogex.h"


// Cuadro de diálogo de CVisitsDimensionDlg

IMPLEMENT_DYNAMIC(CVisitsDimensionDlg, CDialogEx)

CVisitsDimensionDlg::CVisitsDimensionDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_DIALOG_VISITS, pParent)
	, m_Visitas(0)
	, m_Ciclo(FALSE)
{

}

CVisitsDimensionDlg::~CVisitsDimensionDlg()
{
}

void CVisitsDimensionDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_EDIT_VISITAS, m_Visitas);
	DDV_MinMaxInt(pDX, m_Visitas, 1, 999999);
	DDX_Check(pDX, IDC_CHECK_CICLO, m_Ciclo);
}


BEGIN_MESSAGE_MAP(CVisitsDimensionDlg, CDialogEx)
END_MESSAGE_MAP()


// Controladores de mensajes de CVisitsDimensionDlg


BOOL CVisitsDimensionDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	m_Visitas = 5;
	m_Ciclo = false;
	UpdateData(false);

	return TRUE;  // return TRUE unless you set the focus to a control
				  // EXCEPCIÓN: las páginas de propiedades OCX deben devolver FALSE
}


void CVisitsDimensionDlg::OnOK()
{
	CDialogEx::OnOK();
}
