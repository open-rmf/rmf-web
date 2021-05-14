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
  accordian: {
    backgroundColor: theme.palette.secondary.dark,
    color: theme.fontColors.darkTheme,
  },
  headerTab: {
    borderRight: `0.25px solid ${theme.fontColors.darkTheme}`,
    borderLeft: `0.25px solid ${theme.fontColors.darkTheme}`,
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
    // use original img, don't apply styles
  },
  accordian: {
    backgroundColor: theme.palette.secondary.main,
    color: theme.fontColors.lightTheme,
  },
  headerTab: {
    borderRight: `0.25px solid ${theme.fontColors.lightTheme}`,
    borderLeft: `0.25px solid ${theme.fontColors.lightTheme}`,
  },
}));

export const decideThemeStyle = (themeContext: ThemeMode) => {
  return themeContext === ThemeMode.Light ? lightThemeStyles() : darkThemeStyles();
};
