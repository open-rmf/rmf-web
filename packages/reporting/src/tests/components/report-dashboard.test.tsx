import { render, screen, cleanup } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { ReportDashboard } from '../../components/report-dashboard';
import { buildReportMenuStructure } from '../../components/reporter-side-bar-structure';
import { ReportConfigProps } from 'react-components';

describe('ReportDashboard', () => {
  beforeEach(() => {
    const ReportContainer: Record<string, (props: ReportConfigProps) => JSX.Element> = {
      queryAllLogs: () => <h1>QueryAllLogs</h1>,
    };

    render(
      <ReportDashboard
        buildMenuReportStructure={buildReportMenuStructure}
        reportContainer={ReportContainer}
      />,
    );
  });

  afterEach(() => cleanup());

  it('shows the report picker side bar on start', () => {
    expect(screen.getByText('All logs')).toBeTruthy();
  });

  it('closes the side-bar correctly', async () => {
    // To check that it's open
    expect(screen.getByText('All logs')).toBeTruthy();
    userEvent.click(screen.getByLabelText('close drawer'));
    expect(screen.queryByText('All logs')).toBeFalsy();
  });

  it('it closes side-bar and opens it correctly', async () => {
    // To check that it's open
    expect(screen.getByText('All logs')).toBeTruthy();
    userEvent.click(screen.getByLabelText('close drawer'));
    // To check that it's closed
    expect(screen.queryByText('All logs')).toBeFalsy();

    userEvent.click(screen.getByLabelText('open drawer'));
    expect(screen.queryByText('All logs')).toBeTruthy();
  });
});

it('picks a different report and renders correctly', () => {
  const ReportContainer: Record<string, (props: ReportConfigProps) => JSX.Element> = {
    queryAllLogs: () => <h1>QueryAllLogs</h1>,
    showDoorStateReport: () => <h1>Test</h1>,
  };

  render(
    <ReportDashboard
      buildMenuReportStructure={buildReportMenuStructure}
      reportContainer={ReportContainer}
    />,
  );
  userEvent.click(screen.getByText('Doors'));
  expect(screen.getByText('Test'));
});
