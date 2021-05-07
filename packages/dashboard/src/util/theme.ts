import { makeStyles } from '@material-ui/core';
import { ThemeMode } from '../settings';

const darkThemeStyles = makeStyles((theme) => ({
  components: {
    backgroundColor: theme.palette.primary.dark,
    color: theme.fontColors.darkTheme,
  },
  background: {
    backgroundColor: theme.palette.secondary.dark,
  },
}));

const lightThemeStyles = makeStyles((theme) => ({
  components: {
    backgroundColor: theme.palette.primary.main,
    color: theme.fontColors.lightTheme,
  },
  background: {
    backgroundColor: theme.palette.secondary.main,
  },
}));

export const decideThemeStyle = (themeContext: ThemeMode) => {
  return themeContext === ThemeMode.Light ? lightThemeStyles() : darkThemeStyles();
};
