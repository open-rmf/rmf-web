import { makeStyles } from '@material-ui/core';
import { ThemeMode } from '../settings';

const darkThemeStyles = makeStyles((theme) => ({
  components: {
    backgroundColor: theme.palette.primary.dark,
    color: theme.fontColors,
  },
  background: {
    backgroundColor: theme.palette.secondary.dark,
  },
  map: {
    filter:
      'invert(90%) sepia(12%) saturate(5773%) hue-rotate(193deg) brightness(92%) contrast(92%)',
  },
  accordian: {
    backgroundColor: theme.palette.secondary.main,
    color: theme.fontColors,
  },
  headerTab: {
    borderRight: `0.25px solid ${theme.fontColors}`,
    borderLeft: `0.25px solid ${theme.fontColors}`,
  },
  font: {
    color: theme.fontColors,
  },
  leafletControl: {
    color: `${theme.fontColors} !important`,
    backgroundColor: `${theme.palette.primary.dark} !important`,
  },
}));

const lightThemeStyles = makeStyles((theme) => ({
  components: {
    backgroundColor: theme.palette.primary.main,
    color: theme.fontColors,
  },
  background: {
    backgroundColor: theme.palette.secondary.main,
  },
  map: {
    // use original img, don't apply styles
  },
  accordian: {
    backgroundColor: theme.palette.secondary.main,
    color: theme.fontColors,
  },
  headerTab: {
    borderRight: `0.25px solid ${theme.fontColors}`,
    borderLeft: `0.25px solid ${theme.fontColors}`,
  },
  font: {
    color: theme.fontColors,
  },
  leafletControl: {
    color: `${theme.fontColors} !important`,
    backgroundColor: `${theme.palette.primary.main} !important`,
  },
}));

export const decideThemeStyle = (themeContext: ThemeMode) => {
  return themeContext === ThemeMode.Light ? lightThemeStyles() : darkThemeStyles();
};
