import { render, screen } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { Reporter } from '..';
import { buildReportMenuStructure } from '../reporter-side-bar-structure';

it('smoke test', () => {
  const ReportContainer: Record<string, React.ReactElement> = {
    queryAllLogs: <h1>Still not implemented</h1>,
  };

  render(
    <Reporter
      buildMenuReportStructure={buildReportMenuStructure}
      reportContainer={ReportContainer}
    />,
  );
});

it('shows the report picker bar', () => {
  const ReportContainer: Record<string, React.ReactElement> = {
    queryAllLogs: <h1>QueryAllLogs</h1>,
  };

  render(
    <Reporter
      buildMenuReportStructure={buildReportMenuStructure}
      reportContainer={ReportContainer}
    />,
  );
  expect(screen.getByText('All logs')).toBeTruthy();
});

it('renders main screen correctly', () => {
  const ReportContainer: Record<string, React.ReactElement> = {
    queryAllLogs: <h1>QueryAllLogs</h1>,
  };

  render(
    <Reporter
      buildMenuReportStructure={buildReportMenuStructure}
      reportContainer={ReportContainer}
    />,
  );
  expect(screen.getByText('QueryAllLogs')).toBeTruthy();
});

it('picks a different report and renders correctly', () => {
  const ReportContainer: Record<string, React.ReactElement> = {
    queryAllLogs: <h1>QueryAllLogs</h1>,
    showChargerStateReport: <h1>Test</h1>,
  };

  render(
    <Reporter
      buildMenuReportStructure={buildReportMenuStructure}
      reportContainer={ReportContainer}
    />,
  );

  userEvent.click(screen.getByText('Charger states'));
  expect(screen.getByText('Test'));
});
