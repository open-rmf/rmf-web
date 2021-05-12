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
  map: {
    filter:
      'invert(90%) sepia(12%) saturate(5773%) hue-rotate(193deg) brightness(92%) contrast(92%)',
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
  map: {
    // use original img
  },
}));

export const decideThemeStyle = (themeContext: ThemeMode) => {
  return themeContext === ThemeMode.Light ? lightThemeStyles() : darkThemeStyles();
};
