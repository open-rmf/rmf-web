import { render, RenderResult } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { TableFooterPagination } from '../../lib';

describe('Table footer Pagination', () => {
  let container: RenderResult;

  beforeEach(() => {
    container = render(
      <TableFooterPagination
        count={110}
        rowsPerPage={50}
        currentPage={1}
        onChangePage={() => jest.fn()}
        onChangeRowsPerPage={() => jest.fn()}
      />,
    );
  });

  it('show the correct number of rows per page', () => {
    expect(container.findByLabelText('1-50 of 110')).toBeTruthy();
  });

  it('can change the rows per page', async () => {
    userEvent.click(container.getByText('50'));
    userEvent.click(container.getByText('100'));

    expect(container.findByLabelText('1-100 of 110')).toBeTruthy();
  });

  it('advance page when the `Next page` button is clicked ', () => {
    userEvent.click(container.getByLabelText('Next page'));
    expect(container.findByLabelText('51-100 of 110')).toBeTruthy();
  });

  it('goes to previous page when the `Previous page` button is clicked ', () => {
    userEvent.click(container.getByLabelText('Next page'));
    expect(container.findByLabelText('51-100 of 110')).toBeTruthy();
    userEvent.click(container.getByLabelText('Previous page'));
    expect(container.findByLabelText('1-50 of 110')).toBeTruthy();
  });
});
