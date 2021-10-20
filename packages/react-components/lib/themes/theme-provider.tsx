import { ThemeProvider as ThemeProvider_, createTheme } from '@mui/material';
import { ThemeProviderProps } from '@mui/styles';
import React from 'react';

const GlobalStyles: React.FC = () => {
  createTheme();
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
