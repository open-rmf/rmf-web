import { screen, render, RenderResult } from '@testing-library/react';
import React from 'react';
import moment from 'moment';
import { LogRowsType, LogLevel, LogTable } from '../../lib';

const rows = [] as LogRowsType;

const logLevels = Object.values(LogLevel);

const getRandomLogLevel = () => {
  const number = Math.floor(Math.random() * logLevels.length);
  return logLevels[number];
};

for (let i = 0; i < 100; i++) {
  rows.push({
    message: 'Test' + i,
    level: getRandomLogLevel().toUpperCase(),
    timestamp: 'Mon Jan  1 00:00:02 UTC 2001',
  });
}

describe('Log table test', () => {
  let root: RenderResult;
  beforeEach(() => {
    root = render(<LogTable rows={rows} />);
  });

  it('formats dates correctly', async () => {
    const tableFirstDateElement = (await root.getAllByTestId('log-table-date'))[0];
    expect(tableFirstDateElement.innerHTML).toBe(
      moment('Mon Jan  1 00:00:02 UTC 2001').format('lll'),
    );
  });

  it('shows the correct number of rows', () => {
    const allRows = root.container.querySelectorAll('tr').length;
    // Fixme: -2. 1 from the tr of the table header and the other for the footer.
    expect(allRows - 2).toBe(50);
  });
});

describe('Applies styles to labels correctly', () => {
  const styleRows = [] as LogRowsType;

  for (let i = 0; i < logLevels.length; i++) {
    styleRows.push({
      message: 'Test' + i,
      level: logLevels[i].toUpperCase(),
      timestamp: 'Mon Jan  1 00:00:02 UTC 2001',
    });
  }

  beforeEach(() => {
    render(<LogTable rows={styleRows} />);
  });

  it('set the style correctly when the label ERROR ', () => {
    expect(screen.getByText('ERROR').className).toContain('makeStyles-error');
  });

  it('set the style correctly when the label DEBUG ', () => {
    expect(screen.getByText('DEBUG').className).toContain('makeStyles-debug');
  });

  it('set the style correctly when the label WARN ', () => {
    expect(screen.getByText('WARN').className).toContain('makeStyles-warn');
  });

  it('set the style correctly when the label FATAL ', () => {
    expect(screen.getByText('FATAL').className).toContain('makeStyles-error');
  });
});
