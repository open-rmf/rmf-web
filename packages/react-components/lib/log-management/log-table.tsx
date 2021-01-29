import React from 'react';
import TableContainer from '@material-ui/core/TableContainer';
import {
  makeStyles,
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableRow,
  TablePagination,
  TableFooter,
} from '@material-ui/core';
import Paper from '@material-ui/core/Paper';
import TablePaginationActions from '@material-ui/core/TablePagination/TablePaginationActions';
import { LogLevel } from '.';
import moment from 'moment';

export type LogRowsType = { level: string; message: string; timestamp: string }[];
export interface LogTableProps {
  rows: LogRowsType | [];
}
const useStyles = makeStyles((theme) => ({
  table: {
    minWidth: 650,
  },
  container: {
    maxHeight: '55vh',
  },
  root: {
    width: '100%',
  },
  dateColumn: {
    width: '8.5rem',
  },
  textColumn: {
    whiteSpace: 'pre-wrap',
    wordWrap: 'break-word',
    width: '65rem',
  },
  dateLabel: {
    width: '8.5rem',
  },
  messageLabel: {
    width: '65rem',
  },
  levelLabel: {
    width: '3rem',
  },
  error: {
    color: theme.palette.error.main,
  },
  debug: {
    color: theme.palette.secondary.dark,
  },
  warn: {
    color: theme.palette.warning.light,
  },
  info: {
    color: theme.palette.info.main,
  },
}));

export const LogTable = (props: LogTableProps): React.ReactElement => {
  const { rows } = props;
  const classes = useStyles();

  const [page, setPage] = React.useState(0);
  const [rowsPerPage, setRowsPerPage] = React.useState(50);

  const emptyRows = rowsPerPage - Math.min(rowsPerPage, rows.length - page * rowsPerPage);

  const handleChangePage = (event: React.MouseEvent<HTMLButtonElement> | null, newPage: number) => {
    setPage(newPage);
  };

  const handleChangeRowsPerPage = (
    event: React.ChangeEvent<HTMLInputElement | HTMLTextAreaElement>,
  ) => {
    setRowsPerPage(parseInt(event.target.value, 10));
    setPage(0);
  };

  const getLogLevelStyle = (level: string): string | undefined => {
    level = level.toLowerCase();
    switch (level) {
      case LogLevel.Error:
        return classes.error;
      case LogLevel.Warn:
        return classes.warn;
      case LogLevel.Fatal:
        return classes.error;
      case LogLevel.Debug:
        return classes.debug;
      case LogLevel.Info:
        return classes.info;
      default:
        return undefined;
    }
  };

  return (
    <Paper className={classes.root}>
      <TableContainer className={classes.container}>
        <Table className={classes.table} size="small" stickyHeader={true} aria-label="sticky table">
          <TableHead>
            <TableRow>
              <TableCell align="center" className={classes.levelLabel}>
                Level
              </TableCell>
              <TableCell align="center" className={classes.messageLabel}>
                Message
              </TableCell>
              <TableCell align="center" className={classes.dateLabel}>
                Timestamp
              </TableCell>
            </TableRow>
          </TableHead>
          <TableBody>
            {(rowsPerPage > 0
              ? rows.slice(page * rowsPerPage, page * rowsPerPage + rowsPerPage)
              : rows
            ).map((row) => (
              <TableRow key={row.message + row.timestamp}>
                <TableCell
                  align="center"
                  className={`${getLogLevelStyle(row.level)} ${classes.levelLabel}`}
                >
                  {row.level}
                </TableCell>
                <TableCell align="left" className={classes.textColumn}>
                  {row.message}
                </TableCell>
                <TableCell
                  className={classes.dateColumn}
                  align="center"
                  data-testid={'log-table-date'}
                >
                  {moment(row.timestamp).format('lll')}
                </TableCell>
              </TableRow>
            ))}
            {emptyRows > 0 && (
              <TableRow style={{ height: 53 * emptyRows }}>
                <TableCell colSpan={6} />
              </TableRow>
            )}
          </TableBody>
        </Table>
      </TableContainer>
      <Table>
        <TableFooter>
          <TableRow>
            <TablePagination
              rowsPerPageOptions={[50, 100, 200, { label: 'All', value: -1 }]}
              colSpan={5}
              count={rows.length}
              rowsPerPage={rowsPerPage}
              page={page}
              SelectProps={{
                inputProps: { 'aria-label': 'rows per page' },
                native: true,
              }}
              onChangePage={handleChangePage}
              onChangeRowsPerPage={handleChangeRowsPerPage}
              ActionsComponent={TablePaginationActions}
            />
          </TableRow>
        </TableFooter>
      </Table>
    </Paper>
  );
};
