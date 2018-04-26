
// Salesman.h: archivo de encabezado principal para la aplicación PROJECT_NAME
//

#pragma once

#ifndef __AFXWIN_H__
	#error "incluir 'stdafx.h' antes de incluir este archivo para PCH"
#endif

#include "resource.h"		// Símbolos principales

#include "CGraph.h"


// CSalesmanApp:
// Consulte la sección Salesman.cpp para obtener información sobre la implementación de esta clase
//

class CSalesmanApp : public CWinApp
{
public:
	CSalesmanApp();

// Reemplazos
public:
	virtual BOOL InitInstance();

// Implementación

	DECLARE_MESSAGE_MAP()
};

extern CSalesmanApp theApp;

double Clock();
