import Button from '@mui/material/Button';
import Card from '@mui/material/Card';
import CardHeader from '@mui/material/CardHeader';
import Checkbox from '@mui/material/Checkbox';
import Divider from '@mui/material/Divider';
import Grid from '@mui/material/Grid';
import List from '@mui/material/List';
import ListItem from '@mui/material/ListItem';
import ListItemIcon from '@mui/material/ListItemIcon';
import ListItemText from '@mui/material/ListItemText';
import { styled, GridProps } from '@mui/material';
import React from 'react';

const classes = {
  container: 'transfer-list-container',
  cardHeader: 'transfer-list-card-header',
  list: 'transfer-list-list',
  button: 'transfer-list-button',
  transferControls: 'transfer-list-controls',
  cardContainer: 'transfer-list-card-container',
};
const StyledGrid = styled((props: GridProps) => <Grid {...props} />)(({ theme }) => ({
  [`&.${classes.container}`]: {
    height: '100%',
    '& > *': {
      height: '100%',
      flex: '1 1 0',
    },
  },
  [`& .${classes.cardContainer}`]: {
    height: '100%',
    '& > *': {
      height: '100%',
      flex: '1 1 0',
    },
  },
  [`& .${classes.cardHeader}`]: {
    padding: theme.spacing(1, 2),
  },
  [`& .${classes.list}`]: {
    backgroundColor: theme.palette.background.paper,
    overflow: 'auto',
  },
  [`& .${classes.button}`]: {
    margin: theme.spacing(0.5, 0),
  },
  [`& .${classes.transferControls}`]: {
    marginTop: 'auto',
    marginBottom: 'auto',
    height: 'auto',
    flex: '0 0 auto',
  },
}));

interface CustomListProps {
  title: React.ReactNode;
  items: string[];
  checked: Set<string>;
  setChecked: React.Dispatch<React.SetStateAction<Set<string>>>;
}

function CustomList({ title, items, checked, setChecked }: CustomListProps) {
  const numberOfChecked = checked.size;

  const handleToggleAllClick = React.useCallback(() => {
    if (numberOfChecked < items.length) {
      setChecked(new Set(items.values()));
    } else {
      setChecked(new Set());
    }
  }, [items, numberOfChecked, setChecked]);

  return (
    <Card variant="outlined" className={classes.cardContainer}>
      <Grid container direction="column" wrap="nowrap">
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
        <List className={classes.list} dense disablePadding component="div" role="list">
          {items.map((item) => {
            const labelId = `transfer-list-all-item-${item}-label`;

            return (
              <ListItem
                key={item}
                role="listitem"
                button
                onClick={() =>
                  setChecked((prev) => {
                    if (prev.has(item)) {
                      prev.delete(item);
                    } else {
                      prev.add(item);
                    }
                    return new Set(prev);
                  })
                }
              >
                <ListItemIcon>
                  <Checkbox
                    checked={checked.has(item)}
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
      </Grid>
    </Card>
  );
}

export interface TransferListProps {
  leftItems: string[];
  rightItems: string[];
  leftTitle?: React.ReactNode;
  rightTitle?: React.ReactNode;
  onTransfer?: (leftItems: string[], rightItems: string[]) => void;
}

export function TransferList({
  leftItems,
  rightItems,
  leftTitle = 'Choices',
  rightTitle = 'Choices',
  onTransfer,
}: TransferListProps): JSX.Element {
  const [leftChecked, setLeftChecked] = React.useState<Set<string>>(new Set());
  const [rightChecked, setRightChecked] = React.useState<Set<string>>(new Set());

  return (
    <StyledGrid
      container
      spacing={2}
      justifyContent="center"
      alignItems="stretch"
      className={classes.container}
    >
      <Grid item>
        <CustomList
          title={leftTitle}
          items={leftItems}
          checked={leftChecked}
          setChecked={setLeftChecked}
        />
      </Grid>
      <Grid item className={classes.transferControls}>
        <Grid container direction="column" justifyContent="center">
          <Grid item>
            <Button
              variant="outlined"
              size="small"
              className={classes.button}
              onClick={() => {
                const newLeft = leftItems.filter((val) => !leftChecked.has(val));
                const newRight = rightItems.concat(Array.from(leftChecked.values()));
                onTransfer && onTransfer(newLeft, newRight);
                setLeftChecked(new Set());
              }}
              disabled={leftChecked.size === 0}
              aria-label="move selected right"
            >
              &gt;
            </Button>
          </Grid>
          <Grid item>
            <Button
              variant="outlined"
              size="small"
              className={classes.button}
              onClick={() => {
                const newRight = rightItems.filter((val) => !rightChecked.has(val));
                const newLeft = leftItems.concat(Array.from(rightChecked.values()));
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
      </Grid>
      <Grid item>
        <CustomList
          title={rightTitle}
          items={rightItems}
          checked={rightChecked}
          setChecked={setRightChecked}
        />
      </Grid>
    </StyledGrid>
  );
}
