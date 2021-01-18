import React from 'react';
import TableContainer from '@material-ui/core/TableContainer';
import {
  makeStyles,
  Table,
  TableBody,
  TableCell,
  TableFooter,
  TableHead,
  TablePagination,
  TableRow,
} from '@material-ui/core';
import Paper from '@material-ui/core/Paper';

interface LogTableProps {
  rows: { level: 'string'; message: 'string'; timestamp: 'string' }[];
}
const useStyles = makeStyles({
  table: {
    minWidth: 650,
  },
});

export const LogTable = (props: LogTableProps): React.ReactElement => {
  const { rows } = props;
  const classes = useStyles();

  const [page, setPage] = React.useState(0);
  const [rowsPerPage, setRowsPerPage] = React.useState(100);

  const handleChangePage = (event: React.MouseEvent<HTMLButtonElement> | null, newPage: number) => {
    setPage(newPage);
  };

  const handleChangeRowsPerPage = (
    event: React.ChangeEvent<HTMLInputElement | HTMLTextAreaElement>,
  ) => {
    setRowsPerPage(parseInt(event.target.value, 10));
    setPage(0);
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
              {/* <TableCell component="th" scope="row">
                {row.name}
              </TableCell> */}
              <TableCell align="left">{row.level}</TableCell>
              <TableCell align="left">{row.message}</TableCell>
              <TableCell align="right">{row.timestamp}</TableCell>
            </TableRow>
          ))}
        </TableBody>
        <TableFooter>
          <TableRow>
            <TablePagination
              rowsPerPageOptions={[100, 200, { label: 'All', value: -1 }]}
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
              // ActionsComponent={TablePaginationActions}
            />
          </TableRow>
        </TableFooter>
      </Table>
    </TableContainer>
  );
};

// return (
//   <div>
//     <div style={{ height: '60%', overflowY: 'scroll' }}>
//       <Table>
//         <thead>
//         <tbody>
//           {this.logs.map(({ level, message, timestamp }) => (
//             <tr>
//               <td className={this.getLevel(level).class + ' font-weight-bold '}>{level}</td>
//               <td>{message}</td>
//               <td>{moment(timestamp).format('LLLL')}</td>
//             </tr>
//           ))}
//         </tbody>
//       </Table>
//     </div>
//     {this.pagination}
//   </div>
// );
