import { styled, Paper, Typography } from '@mui/material';
import { Meta, StoryFn } from '@storybook/react';
import React from 'react';

export default {
  title: 'Theme',
} satisfies Meta;

interface ColorCardProps {
  color: string | undefined;
  colorName: string;
}

const prefix = 'theme-story';
const classes = {
  paperSize: `${prefix}-paper-size`,
  colorDescription: `${prefix}-color-description`,
  main: `${prefix}-main`,
  themedefault: `${prefix}-theme-default`,
  themePaper: `${prefix}-theme-paper`,
  themeFont: `${prefix}-theme-font`,
  success: `${prefix}-success`,
  error: `${prefix}-error`,
  warning: `${prefix}-warning`,
  info: `${prefix}-info`,
  divider: `${prefix}-divider`,
  header: `${prefix}-header`,
};

const ColorCardRoot = styled('div')(() => ({
  [`& .${classes.paperSize}`]: {
    width: '100px',
    height: '100px',
  },
  [`& .${classes.colorDescription}`]: {
    width: '100px',
  },
}));

const ThemeDisplayRoot = styled('div')(({ theme }) => ({
  [`& .${classes.main}`]: {
    backgroundColor: theme.palette.primary.main,
  },
  [`& .${classes.themedefault}`]: {
    backgroundColor: theme.palette.background.default,
  },
  [`& .${classes.themePaper}`]: {
    backgroundColor: theme.palette.background.paper,
  },
  [`& .${classes.themeFont}`]: {
    backgroundColor: theme.palette.text.primary,
  },
  [`& .${classes.success}`]: {
    backgroundColor: theme.palette.success.main,
  },
  [`& .${classes.error}`]: {
    backgroundColor: theme.palette.error.main,
  },
  [`& .${classes.warning}`]: {
    backgroundColor: theme.palette.warning.main,
  },
  [`& .${classes.info}`]: {
    backgroundColor: theme.palette.info.main,
  },
  [`& .${classes.divider}`]: {
    backgroundColor: theme.palette.divider,
  },
  [`& .${classes.header}`]: {
    marginLeft: '1rem',
  },
}));

const ColorCard = (props: ColorCardProps): JSX.Element => {
  const { color, colorName } = props;
  return (
    <ColorCardRoot style={{ margin: '0 1rem' }}>
      <Paper className={`${color} ${classes.paperSize}`} />
      <Typography variant="body1" align="center" className={classes.colorDescription}>
        {colorName}
      </Typography>
    </ColorCardRoot>
  );
};

const ThemeDisplay = (): JSX.Element => {
  return (
    <ThemeDisplayRoot>
      <Typography variant="h5" className={classes.header}>
        Theme Colors
      </Typography>
      <div style={{ display: 'flex' }}>
        <ColorCard color={classes.main} colorName={'Main'} />
        <ColorCard color={classes.themedefault} colorName={'Default'} />
        <ColorCard color={classes.themePaper} colorName={'Paper'} />
        <ColorCard color={classes.themeFont} colorName={'Font'} />
        <ColorCard color={classes.success} colorName={'Success'} />
        <ColorCard color={classes.error} colorName={'Error'} />
        <ColorCard color={classes.warning} colorName={'Warning'} />
        <ColorCard color={classes.info} colorName={'Info'} />
        <ColorCard color={classes.divider} colorName={'Divider'} />
      </div>
    </ThemeDisplayRoot>
  );
};

export const ThemeStory: StoryFn = (args) => {
  return <ThemeDisplay {...args} />;
};
