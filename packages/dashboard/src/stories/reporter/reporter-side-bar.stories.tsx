import React from 'react';
import { Reporter } from '../../components/reports';
import { ReportContainer } from '../../components/reports/report-list';
import { buildReportMenuStructure } from '../../components/reports/reporter-side-bar-structure';

export default {
  title: 'Reports',
  component: Reporter,
};

export const StaticReportsView = () => (
  <Reporter buildMenuReportStructure={buildReportMenuStructure} reportContainer={ReportContainer} />
);
