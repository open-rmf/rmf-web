import {
  Divider,
  Drawer,
  DrawerProps,
  FormControl,
  FormLabel,
  makeStyles,
  useMediaQuery,
} from '@material-ui/core';
import React from 'react';
import { getApplicationKeyMap } from 'react-hotkeys';

export default function HotKeysDrawer(props: DrawerProps): React.ReactElement {
  const { ...otherProps } = props;
  const classes = useStyles();
  const drawerAnchor = useMediaQuery('(max-aspect-ratio: 8/10)') ? 'bottom' : 'right';
  const keyMap = getApplicationKeyMap();
  //   Object.keys(keyMap).length !== 0 &&
  return (
    <Drawer PaperProps={{ className: classes.drawer }} anchor={drawerAnchor} {...otherProps}>
      HotKeys
      <Divider />
      {Object.values(keyMap).map((memo) => {
        return <div key={memo.name || 'test'}>{memo.name}</div>;
      })}
    </Drawer>
  );
}

const useStyles = makeStyles((theme) => ({
  drawer: {
    '@media (min-aspect-ratio: 8/10)': {
      width: 300,
    },
    '@media (max-aspect-ratio: 8/10)': {
      width: '100%',
    },
  },
  legendLabel: {
    '@media (min-aspect-ratio: 8/10)': {
      fontSize: theme.typography.h6.fontSize,
      padding: theme.spacing(1),
    },
    '@media (max-aspect-ratio: 8/10)': {
      fontSize: theme.typography.h6.fontSize,
      padding: theme.spacing(1),
    },
  },
  trajGroup: {
    '@media (min-aspect-ratio: 8/10)': {
      flexDirection: 'row',
      paddingLeft: theme.spacing(4),
    },
    '@media (max-aspect-ratio: 8/10)': {
      flexDirection: 'row',
      paddingLeft: theme.spacing(8),
    },
  },
  flexBasis: {
    flexBasis: '40%',
  },
}));
