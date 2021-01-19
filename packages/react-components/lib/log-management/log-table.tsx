import React from 'react';
import TableContainer from '@material-ui/core/TableContainer';
import { makeStyles, Table, TableBody, TableCell, TableHead, TableRow } from '@material-ui/core';
import Paper from '@material-ui/core/Paper';
import { TableFooterPagination } from '../tables';
import { LogLevel } from '.';
import moment from 'moment';

interface LogTableProps {
  rows: { level: string; message: string; timestamp: string }[];
}
const useStyles = makeStyles((theme) => ({
  table: {
    minWidth: 650,
  },
  error: {
    color: theme.palette.error.main,
  },
  debug: {
    color: theme.palette.success.main,
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
    if (level === LogLevel.Error) return classes.error;
    if (level === LogLevel.Warn) return classes.warn;
    if (level === LogLevel.Fatal) return classes.error;
    if (level === LogLevel.Debug) return classes.debug;
    if (level === LogLevel.Info) return classes.info;

    return undefined;
  };

  return (
    <TableContainer component={Paper}>
      <Table className={classes.table} size="small" stickyHeader aria-label="sticky table">
        <TableHead>
          <TableRow>
            <TableCell align="left">Level</TableCell>
            <TableCell align="left">Message</TableCell>
            <TableCell align="right">Timestamp</TableCell>
          </TableRow>
        </TableHead>
        <TableBody>
          {(rowsPerPage > 0
            ? rows.slice(page * rowsPerPage, page * rowsPerPage + rowsPerPage)
            : rows
          ).map((row) => (
            <TableRow key={row.message + row.timestamp}>
              <TableCell align="left" className={getLogLevelStyle(row.level)}>
                {row.level}
              </TableCell>
              <TableCell align="left">{row.message}</TableCell>
              <TableCell align="right">{moment(row.timestamp).format('lll')}</TableCell>
            </TableRow>
          ))}
          {emptyRows > 0 && (
            <TableRow style={{ height: 53 * emptyRows }}>
              <TableCell colSpan={6} />
            </TableRow>
          )}
        </TableBody>

        <TableFooterPagination
          count={rows.length}
          rowsPerPage={rowsPerPage}
          currentPage={page}
          onChangePage={handleChangePage}
          onChangeRowsPerPage={handleChangeRowsPerPage}
        />
      </Table>
    </TableContainer>
  );
};
{
  /* <td>{moment(timestamp).format('LLLL')}</td>; */
}
// return (
//   <div>
//     <div style={{ height: '60%', overflowY: 'scroll' }}>
//     </div>
//   </div>
// );
