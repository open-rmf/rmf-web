import React from 'react';
import { ReportDashboard } from '../../components/report-dashboard';
import { buildReportMenuStructure } from '../../components/reporter-side-bar-structure';

export default {
  title: 'Reports',
  component: ReportDashboard,
};

export const StaticReportsView = () => (
  <ReportDashboard buildMenuReportStructure={buildReportMenuStructure} />
);
