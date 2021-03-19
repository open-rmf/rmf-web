import React from 'react';
import { ReportDashboard } from './reports/report-dashboard';
import { buildReportMenuStructure } from './reports/reporter-side-bar-structure';
import { ReportContainer } from './reports/report-list';

function App() {
  return (
    <div className="App">
      <ReportDashboard
        buildMenuReportStructure={buildReportMenuStructure}
        reportContainer={ReportContainer}
      />
    </div>
  );
}

export default App;
