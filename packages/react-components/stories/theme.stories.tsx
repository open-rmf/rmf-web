import { makeStyles, Paper, Typography } from '@material-ui/core';
import { ThemeProvider } from '@material-ui/styles';
import { Meta, Story } from '@storybook/react';
import React from 'react';
import { rmfDark, rmfLight } from '../lib';

export default {
  title: 'Theme',
} as Meta;

interface ColorCardProps {
  color: string | undefined;
  colorName: string;
}

const ColorCard = (props: ColorCardProps): JSX.Element => {
  const { color, colorName } = props;
  const useStyles = makeStyles(() => ({
    paperSize: {
      width: '100px',
      height: '100px',
    },
    colorDescription: {
      width: '100px',
    },
  }));
  const classes = useStyles();

  return (
    <div style={{ margin: '0 1rem' }}>
      <Paper className={`${color} ${classes.paperSize}`} />
      <Typography variant="body1" align="center" className={classes.colorDescription}>
        {colorName}
      </Typography>
    </div>
  );
};

const LightThemeDisplay = (): JSX.Element => {
  const useStyles = makeStyles((theme) => ({
    lightThemeMain: {
      backgroundColor: theme.mainBackground,
    },
    lightThemeSecondary: {
      backgroundColor: theme.secondaryBackground,
    },
    lightThemeFont: {
      backgroundColor: theme.fontColors,
    },
    // non theme related styles
    header: {
      marginLeft: '1rem',
    },
  }));
  const classes = useStyles();

  return (
    <div>
      <Typography variant="h5" className={classes.header}>
        Light Theme Colors
      </Typography>
      <div style={{ display: 'flex' }}>
        <ColorCard color={classes.lightThemeMain} colorName={'Main - White'} />
        <ColorCard color={classes.lightThemeSecondary} colorName={'Secondary - Snow'} />
        <ColorCard color={classes.lightThemeFont} colorName={'Font - Dark Corn Flower Blue'} />
      </div>
    </div>
  );
};

const DarkThemeDisplay = (): JSX.Element => {
  const useStyles = makeStyles((theme) => ({
    darkThemeMain: {
      backgroundColor: theme.mainBackground,
    },
    darkThemeSecondary: {
      backgroundColor: theme.secondaryBackground,
    },
    darkThemeFont: {
      backgroundColor: theme.fontColors,
    },
    // non theme related styles
    header: {
      marginLeft: '1rem',
    },
  }));
  const classes = useStyles();

  return (
    <div>
      <Typography variant="h5" className={classes.header}>
        Dark Theme Colors
      </Typography>
      <div style={{ display: 'flex' }}>
        <ColorCard color={classes.darkThemeMain} colorName={'Main - Dark Corn Flower Blue'} />
        <ColorCard color={classes.darkThemeSecondary} colorName={'Secondary - St Patricks Blue'} />
        <ColorCard color={classes.darkThemeFont} colorName={'Font - Ghost White'} />
      </div>
    </div>
  );
};

const CommonThemeDisplay = (): JSX.Element => {
  const useStyles = makeStyles((theme) => ({
    success: {
      backgroundColor: theme.palette.success.main,
    },
    error: {
      backgroundColor: theme.palette.error.main,
    },
    warning: {
      backgroundColor: theme.palette.warning.main,
    },
    info: {
      backgroundColor: theme.palette.info.main,
    },
    // non theme related styles
    header: {
      marginLeft: '1rem',
    },
  }));
  const classes = useStyles();

  return (
    <div>
      <Typography variant="h5" className={classes.header}>
        Common Theme Colors
      </Typography>
      <div style={{ display: 'flex' }}>
        <ColorCard color={classes.success} colorName={'Success - Light sea green'} />
        <ColorCard color={classes.error} colorName={'Error - Fire Opal'} />
        <ColorCard color={classes.warning} colorName={'Warning - Indian Yellow'} />
        <ColorCard color={classes.info} colorName={'Info - Wisteria'} />
      </div>
    </div>
  );
};

// demonstration of how to inject custom theme
export const LightThemeStory: Story = (args) => {
  return (
    <ThemeProvider theme={rmfLight}>
      <LightThemeDisplay {...args} />
    </ThemeProvider>
  );
};

export const DarkThemeStory: Story = (args) => {
  return (
    <ThemeProvider theme={rmfDark}>
      <DarkThemeDisplay {...args} />
    </ThemeProvider>
  );
};

export const CommonThemeStory: Story = (args) => {
  return (
    <ThemeProvider theme={rmfLight}>
      <CommonThemeDisplay {...args} />
    </ThemeProvider>
  );
};
