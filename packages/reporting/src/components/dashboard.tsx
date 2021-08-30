import React from 'react';
import { ReportDashboard } from './report-dashboard';
import { buildReportMenuStructure } from './reporter-side-bar-structure';

function Dashboard() {
  return (
    <div className="App">
      <ReportDashboard buildMenuReportStructure={buildReportMenuStructure} />
    </div>
  );
}

export default Dashboard;
