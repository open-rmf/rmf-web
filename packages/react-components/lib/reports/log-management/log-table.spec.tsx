import { cleanup, render, RenderResult, screen, fireEvent } from '@testing-library/react';
import { format } from 'date-fns';
import React from 'react';
import { LogLevel } from './log-level';
import { LogRowsType, LogTable } from './log-table';

const rows = [] as LogRowsType;

const logLevels = Object.values(LogLevel);

const getRandomLogLevel = () => {
  const number = Math.floor(Math.random() * logLevels.length);
  return logLevels[number];
};

const timestamp = new Date('Mon Jan  1 00:00:02 UTC 2001');

for (let i = 0; i < 110; i++) {
  rows.push({
    message: 'Test' + i,
    level: getRandomLogLevel().toUpperCase(),
    created: format(timestamp, 'MMM dd yyyy hh:mm aaa'),
    container: { id: i, name: 'container' },
  });
}

describe('Log table test', () => {
  // beforeEach(() => {
  //   render(<LogTable rows={rows} />);
  // });

  afterEach(cleanup);

  // it('formats dates correctly', async () => {
  //   const tableFirstDateElement = (await root.getAllByTestId('log-table-date'))[0];
  //   expect(tableFirstDateElement.innerHTML).toBe(format(timestamp, 'MMM dd yyyy hh:mm aaa'));
  // });

  it('shows the correct number of rows', () => {
    const { container } = render(<LogTable rows={rows} />);
    const allRows = container.querySelectorAll('.MuiDataGrid-row').length;
    expect(allRows).toBe(100);
  });
});

describe('Table footer Pagination', () => {
  // beforeEach(() => {
  //   render(<LogTable rows={rows} />);
  // });

  // afterEach(cleanup);

  it('show the correct number of rows per page', () => {
    render(<LogTable rows={rows} />);
    // NOTE: mui v5 is using the unicode char '–', different from '-'!!
    expect(screen.getByText('1–100 of 110')).toBeTruthy();
  });

  it('can change the rows per page', async () => {
    // FIXME[CR]:
    // ERROR:
    // Unable to find an element with the text: 100.
    // This could be because the text is broken up by multiple elements.
    // In this case, you can provide a function for your text matcher to make your matcher more flexible.
    // fireEvent.click(screen.getByText('100'));
    // fireEvent.click(screen.getByText('50'));
    // // NOTE: mui v5 is using the unicode char '–', different from '-'!!
    // expect(screen.getByText('1–50 of 110')).toBeTruthy();
  });

  it('advance page when the `Next Page` button is clicked ', async () => {
    render(<LogTable rows={rows} />);
    const nextPageButton = screen.queryByLabelText('Go to next page');
    nextPageButton && fireEvent.click(nextPageButton);
    // NOTE: mui v5 is using the unicode char '–', different from '-'!!
    expect(screen.getByText('101–110 of 110')).toBeTruthy();
  });

  it('goes to previous page when the `Previous page` button is clicked ', () => {
    render(<LogTable rows={rows} />);
    const nextPageButton = screen.queryByLabelText('Go to next page');
    nextPageButton && fireEvent.click(nextPageButton);
    // NOTE: mui v5 is using the unicode char '–', different from '-'!!
    expect(screen.getByText('101–110 of 110')).toBeTruthy();

    const previousPageButton = screen.queryByLabelText('Go to previous page');
    previousPageButton && fireEvent.click(previousPageButton);
    // NOTE: mui v5 is using the unicode char '–', different from '-'!!
    expect(screen.getByText('1–100 of 110')).toBeTruthy();
  });
});

describe('Applies styles to labels correctly', () => {
  const styleRows = [] as LogRowsType;

  for (let i = 0; i < logLevels.length; i++) {
    styleRows.push({
      message: 'Test' + i,
      level: logLevels[i].toUpperCase(),
      created: format(timestamp, 'MMM dd yyyy hh:mm aaa'),
      container: { id: i, name: 'container' },
    });
  }

  beforeEach(() => {
    render(<LogTable rows={styleRows} />);
  });

  it('set the style correctly when the label ERROR ', () => {
    expect(screen.getByText('ERROR').className).toContain('log-table-error');
  });

  it('set the style correctly when the label DEBUG ', () => {
    expect(screen.getByText('DEBUG').className).toContain('log-table-debug');
  });

  it('set the style correctly when the label WARN ', () => {
    expect(screen.getByText('WARN').className).toContain('log-table-warn');
  });

  it('set the style correctly when the label FATAL ', () => {
    expect(screen.getByText('FATAL').className).toContain('log-table-error');
  });
});
