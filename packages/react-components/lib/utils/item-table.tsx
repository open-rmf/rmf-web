import React from 'react';
import { styled, TableCellProps, TableCell } from '@mui/material';

const commonStyles = {
  overflow: 'hidden',
  height: 31,
};

const prefix = 'item-table-cell';
export const useFixedTableCellStylesClasses = {
  fixedTableCell: `${prefix}-fix-cell`,
  fixedLastTableCell: `${prefix}-last-fix-cell`,
};
export const ItemTableCell = styled((props: TableCellProps) => <TableCell {...props} />)(() => ({
  [`&.${useFixedTableCellStylesClasses.fixedTableCell}`]: {
    flex: '1 0 0',
    ...commonStyles,
  },
  [`&.${useFixedTableCellStylesClasses.fixedLastTableCell}`]: {
    flex: '1.5 0 0',
    ...commonStyles,
  },
}));
