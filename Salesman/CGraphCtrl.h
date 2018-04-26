#pragma once
#include "afxwin.h"
#include "CGraph.h"

class CGraphCtrl : public CStatic
{
	DECLARE_DYNAMIC(CGraphCtrl)
public:
	CGraph *m_pGraph;
	CVisits *m_pVisits;
	CTrack *m_pTrack;
	bool m_DrawDijkstraDistances;
public:
	CGraphCtrl();
	virtual ~CGraphCtrl();

protected:
	DECLARE_MESSAGE_MAP()
	// Generated message map functions
	afx_msg void OnPaint();

};

