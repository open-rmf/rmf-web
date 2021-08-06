import { render, screen, cleanup } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { ReportDashboard } from '../../components/report-dashboard';
import { buildReportMenuStructure } from '../../components/reporter-side-bar-structure';

describe('ReportDashboard', () => {
  beforeEach(() => {
    render(<ReportDashboard buildMenuReportStructure={buildReportMenuStructure} />);
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
  render(<ReportDashboard buildMenuReportStructure={buildReportMenuStructure} />);
  userEvent.click(screen.getByText('Doors'));
  expect(screen.getByText('Reports - Door State Report'));
});
