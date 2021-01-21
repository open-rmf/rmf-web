import React from 'react';
import { TableFooter, TablePagination, TableRow } from '@material-ui/core';
import TablePaginationActions from '@material-ui/core/TablePagination/TablePaginationActions';

/**
 * count: total number of rows
 * rowsPerPage: number of rows per page
 * currentPage: page currently in
 * onChangePage: callback to run when a change of page ocurred
 * onChangeRowsPerPage: callback to run when a change rows per page ocurred
 */

export interface TableFooterPaginationProps {
  count: number;
  rowsPerPage: number;
  currentPage: number;
  onChangePage: (event: React.MouseEvent<HTMLButtonElement> | null, newPage: number) => void;
  onChangeRowsPerPage: (event: React.ChangeEvent<HTMLInputElement | HTMLTextAreaElement>) => void;
}

export const TableFooterPagination = (props: TableFooterPaginationProps): React.ReactElement => {
  const { count, currentPage, rowsPerPage, onChangePage, onChangeRowsPerPage } = props;
  return (
    <TableFooter>
      <TableRow>
        <TablePagination
          rowsPerPageOptions={[50, 100, 200, { label: 'All', value: -1 }]}
          colSpan={5}
          count={count}
          rowsPerPage={rowsPerPage}
          page={currentPage}
          SelectProps={{
            inputProps: { 'aria-label': 'rows per page' },
            native: true,
          }}
          onChangePage={onChangePage}
          onChangeRowsPerPage={onChangeRowsPerPage}
          ActionsComponent={TablePaginationActions}
        />
      </TableRow>
    </TableFooter>
  );
};
