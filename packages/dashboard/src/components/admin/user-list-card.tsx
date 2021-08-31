import {
  Button,
  Card,
  CardHeader,
  IconButton,
  InputAdornment,
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
import SearchIcon from '@material-ui/icons/Search';
import React from 'react';
import { ConfirmationDialog, Loading, useAsync } from 'react-components';
import { useHistory, useRouteMatch } from 'react-router';
import { AppControllerContext } from '../app-contexts';
import { CreateUserDialog, CreateUserDialogProps } from './create-user-dialog';

const ItemsPerPage = 20;

const useStyles = makeStyles((theme) => ({
  controlsButton: {
    float: 'right',
  },
  tableRow: {
    cursor: 'pointer',
    '&:hover': {
      backgroundColor: theme.palette.action.hover,
    },
  },
}));

export interface UserListCardProps extends Pick<CreateUserDialogProps, 'createUser'> {
  searchUsers?: (search: string, limit: number, offset: number) => Promise<string[]> | string[];
  deleteUser?: (user: string) => Promise<void> | void;
}

export function UserListCard({
  searchUsers,
  deleteUser,
  createUser,
}: UserListCardProps): JSX.Element {
  const classes = useStyles();
  const safeAsync = useAsync();
  const history = useHistory();
  const match = useRouteMatch();
  const [users, setUsers] = React.useState<string[]>([]);
  const [selectedUser, setSelectedUser] = React.useState<string | null>(null);
  const [search, setSearch] = React.useState('');
  const [searching, setSearching] = React.useState(false);
  const [searchInput, setSearchInput] = React.useState('');
  const searchTimer = React.useRef<number | undefined>(undefined);
  const [page, setPage] = React.useState(0);
  const [hasMore, setHasMore] = React.useState(false);
  const [openDeleteDialog, setOpenDeleteDialog] = React.useState(false);
  const [deleting, setDeleting] = React.useState(false);
  const [openCreateDialog, setOpenCreateDialog] = React.useState(false);
  const { showErrorAlert } = React.useContext(AppControllerContext);

  const refresh = React.useCallback(async () => {
    if (!searchUsers) return;
    setSearching(true);
    (async () => {
      try {
        const results = await safeAsync(searchUsers(search, ItemsPerPage + 1, ItemsPerPage * page));
        setHasMore(results.length > ItemsPerPage);
        setUsers(results.slice(0, ItemsPerPage));
      } catch (e) {
        showErrorAlert(`Failed to get users: ${e.message}`);
      } finally {
        setSearching(false);
      }
    })();
  }, [page, search, searchUsers, showErrorAlert, safeAsync]);

  React.useEffect(() => {
    refresh();
  }, [refresh]);

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
              InputProps={{
                startAdornment: (
                  <InputAdornment position="start">
                    <SearchIcon />
                  </InputAdornment>
                ),
              }}
              value={searchInput}
              onChange={(ev) => {
                const newInput = ev.target.value;
                setSearchInput(newInput);
                clearTimeout(searchTimer.current);
                searchTimer.current = window.setTimeout(() => setSearch(newInput), 300);
              }}
            />
            <IconButton
              color="primary"
              onClick={() => setOpenCreateDialog(true)}
              aria-label="create user"
            >
              <AddIcon fontSize="large" />
            </IconButton>
          </>
        }
      />
      <TableContainer id="admin-user-table">
        <Loading loading={searching}>
          <Table size="small">
            <TableHead>
              <TableRow>
                <TableCell>Username</TableCell>
                <TableCell />
              </TableRow>
            </TableHead>
            <TableBody>
              {users.map((u) => (
                <TableRow
                  key={u}
                  className={classes.tableRow}
                  onClick={() => history.push(`${match.path}/${u}`)}
                >
                  <TableCell>{u}</TableCell>
                  <TableCell>
                    <Button
                      variant="contained"
                      color="secondary"
                      startIcon={<DeleteIcon />}
                      className={classes.controlsButton}
                      onClick={(ev) => {
                        ev.stopPropagation();
                        setSelectedUser(u);
                        setOpenDeleteDialog(true);
                      }}
                    >
                      Delete
                    </Button>
                  </TableCell>
                </TableRow>
              ))}
            </TableBody>
          </Table>
          {users.length === 0 && searching && <div style={{ height: 100 }} />}
        </Loading>
        <TablePagination
          component="div"
          count={hasMore ? -1 : page * ItemsPerPage + users.length}
          page={page}
          rowsPerPage={ItemsPerPage}
          rowsPerPageOptions={[ItemsPerPage]}
          onChangePage={(_, newPage) => setPage(newPage)}
        />
      </TableContainer>
      {openDeleteDialog && (
        <ConfirmationDialog
          open={openDeleteDialog}
          title="Confirm Delete"
          submitting={deleting}
          onClose={() => setOpenDeleteDialog(false)}
          onSubmit={async () => {
            setDeleting(true);
            try {
              selectedUser && deleteUser && (await safeAsync(deleteUser(selectedUser)));
              setDeleting(false);
              setOpenDeleteDialog(false);
              refresh();
            } catch (e) {
              setDeleting(false);
              showErrorAlert(`Failed to delete user: ${e.message}`);
            }
          }}
        >
          <Typography>{`Are you sure you want to delete "${selectedUser}"?`}</Typography>
        </ConfirmationDialog>
      )}
      {openCreateDialog && (
        <CreateUserDialog
          open={openCreateDialog}
          setOpen={setOpenCreateDialog}
          createUser={
            createUser &&
            (async (user) => {
              await createUser(user);
              refresh();
            })
          }
        />
      )}
    </Card>
  );
}
