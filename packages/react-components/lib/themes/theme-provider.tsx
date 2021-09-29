import { makeStyles, ThemeProvider as ThemeProvider_, ThemeProviderProps } from '@material-ui/core';
import React from 'react';

const useStyles = makeStyles((theme) => ({
  '@global': theme['@global'] || {},
}));

const GlobalStyles: React.FC = () => {
  useStyles();
  return null;
};

/**
 * Wraps material-ui `ThemeProvider` to also apply the theme's global css.
 */
export const ThemeProvider: React.FC<ThemeProviderProps> = ({ children, ...otherProps }) => {
  return (
    <ThemeProvider_ {...otherProps}>
      <GlobalStyles />
      {children}
    </ThemeProvider_>
  );
};
