import { makeStyles } from '@material-ui/core';

const commonStyles = {
  overflow: 'hidden',
  height: 31,
};
export const useFixedTableCellStyles = makeStyles(() => ({
  fixedTableCell: {
    flex: '1 0 0',
    ...commonStyles,
  },
  fixedLastTableCell: {
    flex: '1.5 0 0',
    ...commonStyles,
  },
}));
