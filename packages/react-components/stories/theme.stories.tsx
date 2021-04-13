import { Meta, Story } from '@storybook/react';
import React from 'react';
import { customTheme } from '../lib';
import { Typography, makeStyles, Paper } from '@material-ui/core';
import { ThemeProvider } from '@material-ui/styles';

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

const ColorDisplay = (): JSX.Element => {
  const useStyles = makeStyles((theme) => ({
    main: {
      backgroundColor: theme.palette.primary.main,
    },
    secondary: {
      backgroundColor: theme.palette.secondary.main,
    },
    black: {
      backgroundColor: theme.colors.black,
    },
    xanadu: {
      backgroundColor: theme.colors.xanadu,
    },
    stPatrickBlue: {
      backgroundColor: theme.colors.stPatricksBlue,
    },
    blueYonder: {
      backgroundColor: theme.colors.blueYonder,
    },
    ghostWhite: {
      backgroundColor: theme.colors.ghostWhite,
    },
    indianYellow: {
      backgroundColor: theme.colors.indianYellow,
    },
    fireOpal: {
      backgroundColor: theme.colors.fireOpal,
    },
  }));
  const classes = useStyles();

  return (
    <div style={{ display: 'flex' }}>
      <ColorCard color={classes.main} colorName={'Dark Corn Flower Blue (Main)'} />
      <ColorCard color={classes.secondary} colorName={'Light Sea Green (Secondary)'} />
      <ColorCard color={classes.black} colorName={'Black'} />
      <ColorCard color={classes.xanadu} colorName={'Xanadu'} />
      <ColorCard color={classes.stPatrickBlue} colorName={"St Patrick's Blue"} />
      <ColorCard color={classes.blueYonder} colorName={'Blue Yonder'} />
      <ColorCard color={classes.ghostWhite} colorName={'Ghost White'} />
      <ColorCard color={classes.indianYellow} colorName={'Indian Yellow'} />
      <ColorCard color={classes.fireOpal} colorName={'Fire Opal'} />
    </div>
  );
};

// demonstration of how to inject custom theme
export const ThemeStory: Story = (args) => {
  return (
    <ThemeProvider theme={customTheme}>
      <ColorDisplay {...args} />
    </ThemeProvider>
  );
};
