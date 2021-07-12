import Button from '@material-ui/core/Button';
import Card from '@material-ui/core/Card';
import CardHeader from '@material-ui/core/CardHeader';
import Checkbox from '@material-ui/core/Checkbox';
import Divider from '@material-ui/core/Divider';
import Grid from '@material-ui/core/Grid';
import List from '@material-ui/core/List';
import ListItem from '@material-ui/core/ListItem';
import ListItemIcon from '@material-ui/core/ListItemIcon';
import ListItemText from '@material-ui/core/ListItemText';
import { createStyles, makeStyles, Theme } from '@material-ui/core/styles';
import React from 'react';

const useStyles = makeStyles((theme: Theme) =>
  createStyles({
    root: {
      margin: 'auto',
    },
    cardHeader: {
      padding: theme.spacing(1, 2),
    },
    list: {
      width: 200,
      height: 230,
      backgroundColor: theme.palette.background.paper,
      overflow: 'auto',
    },
    button: {
      margin: theme.spacing(0.5, 0),
    },
  }),
);

interface CustomListProps {
  title: React.ReactNode;
  items: string[];
  checked: Set<number>;
  setChecked: React.Dispatch<React.SetStateAction<Set<number>>>;
}

function CustomList({ title, items, checked, setChecked }: CustomListProps) {
  const classes = useStyles();
  const numberOfChecked = checked.size;

  const handleToggleAllClick = React.useCallback(() => {
    if (numberOfChecked < items.length) {
      setChecked(new Set(items.keys()));
    } else {
      setChecked(new Set());
    }
  }, [items, numberOfChecked, setChecked]);

  return (
    <Card>
      <CardHeader
        className={classes.cardHeader}
        avatar={
          <Checkbox
            onClick={handleToggleAllClick}
            checked={numberOfChecked > 0}
            indeterminate={numberOfChecked > 0 && numberOfChecked < items.length}
            disabled={items.length === 0}
            inputProps={{ 'aria-label': 'all items selected' }}
          />
        }
        title={title}
        subheader={`${numberOfChecked}/${items.length} selected`}
      />
      <Divider />
      <List className={classes.list} dense component="div" role="list">
        {items.map((item, idx) => {
          const labelId = `transfer-list-all-item-${idx}-label`;

          return (
            <ListItem
              key={idx}
              role="listitem"
              button
              onClick={() =>
                setChecked((prev) => {
                  if (prev.has(idx)) {
                    prev.delete(idx);
                  } else {
                    prev.add(idx);
                  }
                  return new Set(prev);
                })
              }
            >
              <ListItemIcon>
                <Checkbox
                  checked={checked.has(idx)}
                  tabIndex={-1}
                  disableRipple
                  inputProps={{ 'aria-labelledby': labelId }}
                />
              </ListItemIcon>
              <ListItemText id={labelId} primary={item} />
            </ListItem>
          );
        })}
        <ListItem />
      </List>
    </Card>
  );
}

export interface TransferListProps {
  title: React.ReactNode;
  leftItems: string[];
  rightItems: string[];
  onTransfer: (leftItems: string[], rightItems: string[]) => void;
}

export function TransferList({
  title,
  leftItems,
  rightItems,
  onTransfer,
}: TransferListProps): JSX.Element {
  const classes = useStyles();
  const [leftChecked, setLeftChecked] = React.useState<Set<number>>(new Set());
  const [rightChecked, setRightChecked] = React.useState<Set<number>>(new Set());

  return (
    <Grid container spacing={2} justify="center" alignItems="center" className={classes.root}>
      <Grid item>
        <CustomList
          title={title}
          items={leftItems}
          checked={leftChecked}
          setChecked={setLeftChecked}
        />
      </Grid>
      <Grid item>
        <Grid container direction="column" alignItems="center">
          <Button
            variant="outlined"
            size="small"
            className={classes.button}
            onClick={() => {
              const newLeft = leftItems.filter((_val, idx) => !leftChecked.has(idx));
              const newRight = rightItems.concat(
                Array.from(leftChecked.keys()).map((idx) => leftItems[idx]),
              );
              onTransfer && onTransfer(newLeft, newRight);
              setLeftChecked(new Set());
            }}
            disabled={leftChecked.size === 0}
            aria-label="move selected right"
          >
            &gt;
          </Button>
          <Button
            variant="outlined"
            size="small"
            className={classes.button}
            onClick={() => {
              const newRight = rightItems.filter((_val, idx) => !rightChecked.has(idx));
              const newLeft = leftItems.concat(
                Array.from(rightChecked.keys()).map((idx) => rightItems[idx]),
              );
              onTransfer && onTransfer(newLeft, newRight);
              setRightChecked(new Set());
            }}
            disabled={rightChecked.size === 0}
            aria-label="move selected left"
          >
            &lt;
          </Button>
        </Grid>
      </Grid>
      <CustomList
        title={title}
        items={rightItems}
        checked={rightChecked}
        setChecked={setRightChecked}
      />
    </Grid>
  );
}
