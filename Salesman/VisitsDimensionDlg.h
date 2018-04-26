#pragma once


// Cuadro de diálogo de CVisitsDimensionDlg

class CVisitsDimensionDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CVisitsDimensionDlg)

public:
	CVisitsDimensionDlg(CWnd* pParent = NULL);   // Constructor estándar
	virtual ~CVisitsDimensionDlg();

// Datos del cuadro de diálogo
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_DIALOG_VISITS };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // Compatibilidad con DDX/DDV

	DECLARE_MESSAGE_MAP()
public:
	int m_Visitas;
	BOOL m_Ciclo;
	virtual BOOL OnInitDialog();
	virtual void OnOK();
};
