import React from 'react';
import { ReportDashboard } from './report-dashboard';
import { buildReportMenuStructure } from './reporter-side-bar-structure';
import { ReportContainer } from './report-list';

function Dashboard() {
  return (
    <div className="App">
      <ReportDashboard
        buildMenuReportStructure={buildReportMenuStructure}
        reportContainer={ReportContainer}
      />
    </div>
  );
}

export default Dashboard;
