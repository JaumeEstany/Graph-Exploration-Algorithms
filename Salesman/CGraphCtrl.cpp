#include "stdafx.h"
#include "CGraphCtrl.h"

IMPLEMENT_DYNAMIC(CGraphCtrl, CStatic)


CGraphCtrl::CGraphCtrl()
{
	m_pGraph = NULL;
	m_pVisits = NULL;
	m_pTrack = NULL;
	m_DrawDijkstraDistances = false;
}


CGraphCtrl::~CGraphCtrl()
{
}

BEGIN_MESSAGE_MAP(CGraphCtrl, CStatic)
	ON_WM_PAINT()
END_MESSAGE_MAP()


// OnPaint =====================================================================

void CGraphCtrl::OnPaint()
{
	CRect upRect;
	GetUpdateRect(&upRect);
	CPaintDC dc(this); // device context for painting

	CRect rect;
	GetClientRect(&rect);
	dc.IntersectClipRect(upRect);
	dc.Rectangle(rect);
	rect.left += 50;
	rect.right -= 50;
	rect.top += 50;
	rect.bottom -= 50;

	// Calcular el retangulo mundo
	CGPoint mins(1e99, 1e99);
	CGPoint maxs(-1e99, -1e99);
	for (CVertex &v : m_pGraph->m_Vertices) {
		const CGPoint point = v.m_Point;
		mins = Min(mins, point);
		maxs = Max(maxs, point);
	}

	// Calcular la escala
	double escX = rect.Width() / (maxs.m_X - mins.m_X);
	double escY = rect.Height() / (maxs.m_Y - mins.m_Y);
	double esc = escX;
	if (escY<escX) esc = escY;

	// Dibujar el grafo -----------------------------------------------------------
	// Dibujar aristas
	{
		CPen pen(PS_DOT, 1, RGB(100, 100, 100));
		CPen *oldpen = dc.SelectObject(&pen);
		for (list<CVertex>::const_iterator iter = m_pGraph->m_Vertices.cbegin(); iter != m_pGraph->m_Vertices.cend(); ++iter) {
			CPoint point1(int((iter->m_Point.m_X - mins.m_X)*esc + rect.left), int((iter->m_Point.m_Y - mins.m_Y)*esc + rect.top));
			for (list<CVertex*>::const_iterator iter2 = iter->m_Neighbords.cbegin(); iter2 != iter->m_Neighbords.cend(); ++iter2) {
				CPoint point2(int(((*iter2)->m_Point.m_X - mins.m_X)*esc + rect.left), int(((*iter2)->m_Point.m_Y - mins.m_Y)*esc + rect.top));
				dc.MoveTo(point1);
				dc.LineTo(point2);
			}
		}
		dc.SelectObject(oldpen);
		// Dibujar los vértices
		CBrush br(RGB(0, 128, 255));
		CBrush *pBr = dc.SelectObject(&br);

		for (list<CVertex>::const_iterator iter = m_pGraph->m_Vertices.cbegin(); iter != m_pGraph->m_Vertices.cend(); ++iter) {
			CPoint point(int((iter->m_Point.m_X - mins.m_X)*esc + rect.left), int((iter->m_Point.m_Y - mins.m_Y)*esc + rect.top));
			dc.Ellipse(point.x - 4, point.y - 4, point.x + 4, point.y + 4);
		}
		dc.SelectObject(pBr);
	}
	// Dibujar distancias ---------------------------------------------------------
	if (m_DrawDijkstraDistances) {
		dc.SetTextColor(RGB(0, 0, 0));
		for (list<CVertex>::const_iterator iter = m_pGraph->m_Vertices.cbegin(); iter != m_pGraph->m_Vertices.cend(); ++iter) {
			CPoint point(int((iter->m_Point.m_X - mins.m_X)*esc + rect.left), int((iter->m_Point.m_Y - mins.m_Y)*esc + rect.top));
			CString str;
			str.Format("%.2lf", iter->m_DijkstraDistance);
			dc.TextOut(point.x + 10, point.y, str);
		}
	}
	// Dibujar CTrack -------------------------------------------------------------
	{
		// Dibujar las aristas del track
		int x0 = numeric_limits<int>::lowest();
		int y0 = numeric_limits<int>::lowest();
		CVertex *pV0 = NULL;
		for (CVertex *pV1 : m_pTrack->m_Vertices) {
			CGPoint p = pV1->m_Point;
			int x1 = int((p.m_X - mins.m_X)*esc + rect.left);
			int y1 = int((p.m_Y - mins.m_Y)*esc + rect.top);
			if (x0 != numeric_limits<int>::lowest()) {
				CPen pen;
				if (pV0->NeighbordP(pV1)) pen.CreatePen(PS_SOLID,3,RGB(0, 200, 0));
				else pen.CreatePen(PS_SOLID, 3, RGB(200, 0, 0));
				CPen *oldpen = dc.SelectObject(&pen);
				dc.MoveTo(x0, y0); dc.LineTo(x1, y1);
				int x = (x0 * 2 + x1) / 3;
				int y = (y0 * 2 + y1) / 3;
				double dx = x1 - x0;
				double dy = y1 - y0;
				double d = sqrt(dx*dx + dy*dy);
				dx = dx / d;
				dy = dy / d;
				double nx = dy;
				double ny = -dx;
				dc.MoveTo((int)(x + nx * 7 - dx * 7 + 0.5), (int)(y + ny * 7 - dy * 7 + 0.5)); dc.LineTo(x, y);
				dc.MoveTo((int)(x - nx * 7 - dx * 7 + 0.5), (int)(y - ny * 7 - dy * 7 + 0.5));  dc.LineTo(x, y);
				dc.SelectObject(oldpen);
			}
			x0 = x1;
			y0 = y1;
			pV0 = pV1;
		}
		// Dibujar los vértices del track
		bool first = true;
		for (list<CVertex*>::iterator iter = m_pTrack->m_Vertices.begin(); iter!= m_pTrack->m_Vertices.end();) {
			CGPoint p = (*iter)->m_Point;
			int x1 = int((p.m_X - mins.m_X)*esc + rect.left);
			int y1 = int((p.m_Y - mins.m_Y)*esc + rect.top);
			++iter;
			if (first) {
				CBrush brush(RGB(255, 0, 0));
				CBrush *pBr = dc.SelectObject(&brush);
				dc.Rectangle(x1 - 8, y1 - 8, x1+8,y1+8);
				dc.SelectObject(pBr);
				first = false;
			}
			else if (iter == m_pTrack->m_Vertices.end()) {
				CBrush brush(RGB(0, 255, 0));
				CBrush *pBr = dc.SelectObject(&brush);
				dc.Ellipse(x1 - 8, y1 - 8, x1 + 8, y1 + 8);
				dc.SelectObject(pBr);
				break;
			}
			else {
				CBrush brush(RGB(0, 255, 0));
				CBrush *pBr = dc.SelectObject(&brush);
				dc.Rectangle(x1 - 8, y1 - 8, x1 + 8, y1 + 8);
				dc.SelectObject(pBr);
			}
		}
		// Dibujar el orden de los vértices del track
		int i2 = 0;
		list<CVertex*> apariciones;
		for (list<CVertex*>::iterator iter = m_pTrack->m_Vertices.begin(); iter != m_pTrack->m_Vertices.end();++iter,++i2) {
			CGPoint p = (*iter)->m_Point;
			int x1 = int((p.m_X - mins.m_X)*esc + rect.left);
			int y1 = int((p.m_Y - mins.m_Y)*esc + rect.top);
			int ap = 0;
			for (CVertex *pV : apariciones) {
				if (pV == *iter) ++ap;
			}
			dc.SetTextColor(RGB(255, 64, 0));
			CString str;
			str.Format("%d", i2);
			dc.TextOut(x1 + 15, y1 + 15 * ap, str);
			apariciones.push_back(*iter);
		}


	}
	// Dibujar las visitas  -------------------------------------------------------
	{
		CBrush br(RGB(255, 128, 64));
		CBrush *pBr = dc.SelectObject(&br);
		bool start = true;
		for (list<CVertex*>::const_iterator iter = m_pVisits->m_Vertices.cbegin(); iter != m_pVisits->m_Vertices.cend();) {
			CPoint point(int(((*iter)->m_Point.m_X - mins.m_X)*esc + rect.left), int(((*iter)->m_Point.m_Y - mins.m_Y)*esc + rect.top));
			++iter;
			if (start) {
				CBrush br(RGB(0, 0, 0));
				CBrush *pBr = dc.SelectObject(&br);
				dc.Rectangle(point.x - 5, point.y - 5, point.x + 5, point.y + 5);
				dc.SelectObject(pBr);
				start = false;
			}
			else if (iter != m_pVisits->m_Vertices.cend()) dc.Ellipse(point.x - 5, point.y - 5, point.x + 5, point.y + 5);
			else {
				CBrush br(RGB(255, 255, 255));
				CBrush *pBr = dc.SelectObject(&br);
				dc.Rectangle(point.x - 5, point.y - 5, point.x + 5, point.y + 5);
				dc.SelectObject(pBr);
				break;
			}
		}
		dc.SelectObject(pBr);
	}
}
