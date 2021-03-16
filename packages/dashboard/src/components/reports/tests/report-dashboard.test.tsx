import { render, screen, cleanup } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { ReportDashboard } from '../report-dashboard';
import { buildReportMenuStructure } from '../reporter-side-bar-structure';

it('smoke test', () => {
  const ReportContainer: Record<string, React.ReactElement> = {
    queryAllLogs: <h1>Still not implemented</h1>,
  };

  render(
    <ReportDashboard
      buildMenuReportStructure={buildReportMenuStructure}
      reportContainer={ReportContainer}
    />,
  );

  cleanup();
});

it('shows the report picker side bar', () => {
  const ReportContainer: Record<string, React.ReactElement> = {
    queryAllLogs: <h1>QueryAllLogs</h1>,
  };

  render(
    <ReportDashboard
      buildMenuReportStructure={buildReportMenuStructure}
      reportContainer={ReportContainer}
    />,
  );
  expect(screen.getByText('All logs')).toBeTruthy();
  cleanup();
});

it('renders the main screen correctly', () => {
  const ReportContainer: Record<string, React.ReactElement> = {
    queryAllLogs: <h1>QueryAllLogs</h1>,
  };

  render(
    <ReportDashboard
      buildMenuReportStructure={buildReportMenuStructure}
      reportContainer={ReportContainer}
    />,
  );
  expect(screen.getByText('QueryAllLogs')).toBeTruthy();
  cleanup();
});

it('picks a different report and renders correctly', () => {
  const ReportContainer: Record<string, React.ReactElement> = {
    queryAllLogs: <h1>QueryAllLogs</h1>,
    showChargerStateReport: <h1>Test</h1>,
  };

  render(
    <ReportDashboard
      buildMenuReportStructure={buildReportMenuStructure}
      reportContainer={ReportContainer}
    />,
  );

  userEvent.click(screen.getByText('Charger states'));
  expect(screen.getByText('Test'));
  cleanup();
});
