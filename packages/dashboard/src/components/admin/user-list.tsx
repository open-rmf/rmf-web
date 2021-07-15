import {
  Button,
  Card,
  CardHeader,
  Dialog,
  DialogContent,
  DialogTitle,
  IconButton,
  makeStyles,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TablePagination,
  TableRow,
  TextField,
  Typography,
} from '@material-ui/core';
import AccountIcon from '@material-ui/icons/AccountCircle';
import AddIcon from '@material-ui/icons/AddCircle';
import DeleteIcon from '@material-ui/icons/Delete';
import React from 'react';
import { ConfirmDialogActions } from '../../../../react-components/dist';

const ItemsPerPage = 20;

const useConfirmDialogActionsStyles = makeStyles({
  button: {
    width: 80,
  },
});

const useStyles = makeStyles((theme) => ({
  controlsButton: {
    float: 'right',
  },
}));

export interface UserListCardProps {
  searchUsers?: (search: string, limit: number, offset: number) => Promise<string[]> | string[];
  deleteUser?: (user: string) => Promise<void> | void;
}

export function UserListCard({ searchUsers, deleteUser }: UserListCardProps): JSX.Element {
  const classes = useStyles();
  const confirmDialogActionsClasses = useConfirmDialogActionsStyles();
  const [users, setUsers] = React.useState<string[]>([]);
  const [selectedUser, setSelectedUser] = React.useState<string | null>(null);
  const [search, setSearch] = React.useState('');
  const [searchInput, setSearchInput] = React.useState('');
  const [page, setPage] = React.useState(0);
  const [hasMore, setHasMore] = React.useState(false);
  const [openDialog, setOpenDialog] = React.useState(false);
  const [deleting, setDeleting] = React.useState(false);
  const searchTimer = React.useRef<number | undefined>(undefined);

  React.useEffect(() => {
    if (!searchUsers) {
      return;
    }
    (async () => {
      const results = await searchUsers(search, ItemsPerPage + 1, ItemsPerPage * page);
      setHasMore(results.length > ItemsPerPage);
      setUsers(results.slice(0, ItemsPerPage));
    })();
  }, [searchUsers, search, page]);

  return (
    <Card variant="outlined">
      <CardHeader
        title="Users"
        titleTypographyProps={{ variant: 'h5' }}
        avatar={<AccountIcon fontSize="large" />}
        action={
          <>
            <TextField
              variant="outlined"
              id="search-users"
              label="Search Users"
              value={searchInput}
              onChange={(ev) => {
                setSearchInput(ev.target.value);
                clearTimeout(searchTimer.current);
                searchTimer.current = window.setTimeout(() => setSearch(searchInput), 500);
              }}
            />
            <IconButton>
              <AddIcon fontSize="large" color="primary" />
            </IconButton>
          </>
        }
      />
      <TableContainer>
        <Table size="small">
          <TableHead>
            <TableRow>
              <TableCell>Username</TableCell>
              <TableCell />
            </TableRow>
          </TableHead>
          <TableBody>
            {users.map((u) => (
              <TableRow key={u}>
                <TableCell>{u}</TableCell>
                <TableCell>
                  <Button
                    variant="contained"
                    color="secondary"
                    startIcon={<DeleteIcon />}
                    className={classes.controlsButton}
                    onClick={() => {
                      setSelectedUser(u);
                      setOpenDialog(true);
                    }}
                  >
                    Delete
                  </Button>
                </TableCell>
              </TableRow>
            ))}
          </TableBody>
        </Table>
        <TablePagination
          component="div"
          count={hasMore ? -1 : page * ItemsPerPage + users.length}
          page={page}
          rowsPerPage={ItemsPerPage}
          rowsPerPageOptions={[ItemsPerPage]}
          onChangePage={(_, newPage) => setPage(newPage)}
        />
      </TableContainer>
      <Dialog open={openDialog}>
        <DialogTitle>Confirm Delete?</DialogTitle>
        <DialogContent>
          <Typography>{`Are you sure you want to delete ${selectedUser}?`}</Typography>
        </DialogContent>
        <ConfirmDialogActions
          loading={deleting}
          classes={confirmDialogActionsClasses}
          onCancelClick={() => setOpenDialog(false)}
          onConfirmClick={async () => {
            setDeleting(true);
            selectedUser && deleteUser && (await deleteUser(selectedUser));
            setDeleting(false);
            setOpenDialog(false);
          }}
        />
      </Dialog>
    </Card>
  );
}
