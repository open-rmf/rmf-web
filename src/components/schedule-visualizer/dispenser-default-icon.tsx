import React from 'react';
import { makeStyles, useTheme } from '@material-ui/core';

const DispenserDefaultIcon = (props: { footprint: number }): React.ReactElement => {
  const { footprint } = props;
  const classes = useStyles();
  const theme = useTheme();
  return (
    <svg
      xmlns="http://www.w3.org/2000/svg"
      x={-footprint}
      y={-footprint}
      width={footprint * 2}
      height={footprint * 2}
      viewBox="0 0 24 24"
    >
      <path
        fill={theme.palette.success.main}
        d="M19 3H4.99c-1.11 0-1.98.9-1.98 2L3 19c0 1.1.88 2 1.99 2H19c1.1 0 2-.9 2-2V5c0-1.1-.9-2-2-2zm0 12h-4c0 1.66-1.35 3-3 3s-3-1.34-3-3H4.99V5H19v10zm-3-5h-2V7h-4v3H8l4 4 4-4z"
        className={classes.clickable}
      />
    </svg>
  );
};

export default DispenserDefaultIcon;

const useStyles = makeStyles(() => ({
  clickable: {
    // I need to use any here to set this property to important. https://stackoverflow.com/questions/53151340/material-ui-and-typescript-how-to-use-important. I'm overriding the default property of Leaflet Path of visible:None.
    pointerEvents: 'visible !important' as any,
  },
}));
